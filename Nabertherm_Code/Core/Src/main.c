/* USER CODE BEGIN Header */
/**
******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Nabertherm Furnace Control Panel
  * @author         : TRAN NGUYEN HIEN
  * * @architecture_notes
  * - State Machine: Manages UI states and System running states independently.
  * - Non-blocking: Uses HAL_GetTick() for sensor reading and debouncing to
  * prevent blocking the main loop.
  * - Double-Buffering LCD: Prevents I2C bottlenecking and screen flickering.
  * - Auto-step Logic: Uses cumulative target seconds rather than resetting clocks.
  * - PID Control: Time-Proportional Control (Slow PWM) với chu kỳ 1000ms.
  * - Anti-Windup: Supervisory integral zoning plus heater cut-off above trajectory.
  * - Sensor Safety: Hardware/software fault validation and over-temperature trip.
  * - Non-Volatile Storage: Versioned settings with checksum on Flash Page 63.
******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal_I2C.h"
#include "MAX31856.h"
#include "PID_Controller.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stddef.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* * System Operation States
 * Separated from UI states so the furnace can run in the background
 * even if the user navigates away from the main screen (future-proofing).
 */
typedef enum {
    SYS_IDLE,       // Power on or manually stopped. Timer does not increment.
    SYS_RUNNING,    // Active heating phase. Timer increments 1Hz.
    SYS_COMPLETED   // All intervals finished. Timer stops, retains last time.
} SystemState_t;

/* * User Interface (Menu) States
 * Defines exactly which screen the user is currently viewing.
 */
typedef enum {
    UI_STATE_MAIN,
    UI_STATE_SET_INTERVAL,
    UI_STATE_SET_P,
    UI_STATE_SET_TEMP,
    UI_STATE_SET_TIME
} UIState_t;

/* Heating Modes */
typedef enum {
    MODE_MT,    // Maintain Temp: PID holds setpoint constantly.
    MODE_TIOT   // Temp Increases Over Time: Ramp rate applied to PID setpoint.
} IntervalMode_t;

/* Latched control faults. Any active fault forces the SSR OFF. */
typedef enum {
    CONTROL_FAULT_NONE = 0,
    CONTROL_FAULT_MAX31856,
    CONTROL_FAULT_SENSOR_DATA,
    CONTROL_FAULT_OVERTEMP
} ControlFault_t;

/* MODE_MT supervisory phases for a high-inertia furnace. */
typedef enum {
    MT_PHASE_APPROACH = 0, /* Heat toward the target with power tapering. */
    MT_PHASE_COAST,        /* SSR OFF while stored heat continues to raise temperature. */
    MT_PHASE_HOLD          /* Tight PID regulation around the requested Setpoint. */
} MTControlPhase_t;

/* * Data structure for a single heating interval (Segment).
 * Grouped into a struct for memory contiguity and easy array management.
 */
typedef struct {
    uint8_t Mode;                 // Stored as a fixed-width value for stable Flash layout.
    uint16_t Temp;
    uint8_t Time_Hour;
    uint8_t Time_Min;
    uint8_t Time_Sec;
} Interval_TypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 200ms software debounce prevents mechanical button switch bouncing
// from triggering multiple interrupts.
#define DEBOUNCE_DELAY 200

// Max segments allowed by the system memory/requirements.
#define MAX_INTERVALS 9

// Variable for Time-Proportional Control (Slow PWM)
#define WINDOW_SIZE 1000

/* Scheduling, sensor validation and safety limits. */
#define SENSOR_SAMPLE_MS                  250U
#define SENSOR_INVALID_LIMIT              3U
#define SENSOR_RECOVERY_VALID_SAMPLES     4U
#define SENSOR_MIN_VALID_C               (-50.0f)
#define SENSOR_MAX_VALID_C               (1800.0f)
#define PROCESS_MAX_TEMP_C                1280U
#define OVERTEMP_TRIP_C                  (1300.0f)
#define OVERTEMP_RESET_C                 (1250.0f)
#define TEMP_FILTER_ALPHA                (0.20f)

/* TIOT keeps the original PID zoning. MODE_MT uses its own tighter controller. */
#define INTEGRAL_ENABLE_BAND_C             4.0f
#define INTEGRAL_DISABLE_BAND_C           10.0f
#define HEATER_CUTOFF_ABOVE_SP_C           1.5f
#define OUTPUT_DEADBAND_MS                20.0f

/* MODE_MT predictive anti-overshoot and hold-control parameters.
 * These are conservative starting values for the 3 kW Nabertherm furnace.
 * Final +/-1 degC performance must be verified and tuned on the real furnace/load.
 */
#define TEMP_RATE_UPDATE_MS             5000U
#define TEMP_RATE_FILTER_ALPHA             0.25f
#define TEMP_RATE_MAX_ABS_C_PER_MIN       80.0f

#define MT_TARGET_TOLERANCE_C              1.0f
#define MT_HARD_CUTOFF_C                   0.50f
#define MT_PREDICT_MARGIN_C                0.30f
#define MT_THERMAL_LOOKAHEAD_MIN           4.0f
#define MT_PREDICT_ACTIVE_BAND_C          12.0f
#define MT_MIN_RISING_RATE_C_PER_MIN       0.08f
#define MT_HOLD_ENTRY_ERROR_C              1.50f
#define MT_HOLD_ENTRY_RATE_C_PER_MIN       0.20f
#define MT_REHEAT_ERROR_C                  3.00f
#define MT_COAST_RELEASE_ERROR_C           1.00f
#define MT_COAST_RELEASE_RATE_C_PER_MIN    0.15f
#define MT_HOLD_MIN_PULSE_MS              40.0f
#define MT_HOLD_BIAS_FRACTION              0.25f

/* Gain scheduling: no integral during approach/coast; stronger PI action in hold. */
#define MT_APPROACH_KP                    20.0f
#define MT_APPROACH_KD                   250.0f
#define MT_HOLD_KP                        80.0f
#define MT_HOLD_KI                         0.40f
#define MT_HOLD_KD                       300.0f

/* Maximum SSR ON time in each 1000 ms window while approaching Setpoint. */
#define MT_MAX_OUTPUT_FAR_MS            1000.0f
#define MT_MAX_OUTPUT_40_20_MS           800.0f
#define MT_MAX_OUTPUT_20_10_MS           550.0f
#define MT_MAX_OUTPUT_10_5_MS            320.0f
#define MT_MAX_OUTPUT_5_3_MS             220.0f
#define MT_MAX_OUTPUT_3_2_MS             160.0f
#define MT_MAX_OUTPUT_2_1_MS             120.0f
#define MT_MAX_OUTPUT_NEAR_MS             90.0f

// Flash Storage Architecture
// STM32F103C8T6 has 64KB Flash. Page 63 is the very last page (1KB size).
// Using the last page ensures we do not accidentally overwrite application code.
#define FLASH_STORAGE_ADDR 0x0800FC00
#define FLASH_MAGIC_WORD   0xAABBCCDE
#define FLASH_DATA_VERSION 2U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
MAX31856_HandleTypeDef max31856;
PID_TypeDef pid_heater;

/* * Flash Storage Wrapper Structure
 * This struct perfectly aligns our variables for easy writing/reading
 * to the internal flash using 32-bit Words.
 */
typedef struct {
    uint32_t MagicWord;      // Validation flag.
    uint32_t Version;
    uint32_t TotalIntervals; // Cast from uint8_t to uint32_t for alignment.
    Interval_TypeDef Intervals[MAX_INTERVALS]; // Payload.
    uint32_t Checksum;
} Flash_Data_t;

// ================= PID PARAMETTERS & NOISE FILTER =================
float Input, Output, Setpoint;
float Kp = 20.0f, Ki = 0.02f, Kd = 250.0f;
uint32_t windowStartTime = 0;
float active_output = 0;
float filtered_temp = -1.0f;
float raw_temp = 0.0f;
bool ki_is_active = true;
float last_setpoint = 0.0f;
uint8_t max31856_fault_bits = 0;
uint8_t invalid_temp_count = 0;
uint8_t valid_recovery_count = 0;
bool sensor_has_valid_sample = false;
bool max31856_ready = false;
volatile ControlFault_t Control_Fault = CONTROL_FAULT_NONE;

//========== Multi-Pin Independent Debounce Variables ==========//
/*
 * Button Event Flags
 * WHY: The EXTI ISR should be as short as possible. We only set flags here.
 * The heavy logic (state changing, flash writing) is deferred to Process_Buttons()
 * in the main loop.
 */
volatile uint32_t last_time_PA8 = 0;
volatile uint32_t last_time_PA9 = 0;
volatile uint32_t last_time_PA10 = 0;
volatile uint32_t last_time_PA11 = 0;
volatile uint32_t last_time_PB15 = 0;

volatile uint8_t button_setting = 0;    // PA8 -> Setting/Run
volatile uint8_t button_redirect = 0;   // PA9 -> Redirect
volatile uint8_t button_select = 0;     // PB15 -> OK/Select
volatile uint8_t button_tang = 0;       // PA11 -> UP
volatile uint8_t button_giam = 0;       // PA10 -> DOWN

// ================= SYSTEM LOGIC & UI =================
/* System Logic & Payload Variables */
Interval_TypeDef Intervals[MAX_INTERVALS];
uint8_t Total_Intervals = 1;

volatile SystemState_t System_Run_State = SYS_IDLE;
volatile UIState_t Current_UI_State = UI_STATE_MAIN;
uint8_t Current_Interval = 1;

// Single atomic source of truth for the global run clock.
volatile uint32_t Run_Total_Seconds = 0;

/*
 * Target_Run_Seconds is the absolute cumulative target time.
 * WHY: Instead of resetting the clock to 0 at the start of a new interval
 * (which looks bad on the UI), we calculate the absolute total seconds needed
 * to finish the current interval and compare it against the running clock.
 */
uint32_t Target_Run_Seconds = 0;
float Current_Temp = 0.0f;

// Linear interpolation TIOT variables
uint32_t Current_Interval_Start_Sec = 0;
float Current_Interval_Start_Temp = 0.0f;

/* Menu Navigation Variables */
uint8_t Setting_P_Index = 1;        // Tracks which interval (P1...Pn) is being edited.
uint8_t Menu_Cursor = 0;            // 0: Mode, 1: Set Temp, 2: Set Time.
uint8_t Temp_Digit_Index = 0;       // For individual digit blinking (Thousands, Hundreds, etc.).
uint8_t Time_Field_Index = 0;       // 0: Hour, 1: Min, 2: Sec.

/* Temporary Editing Buffers */
// Edits are stored here first, and only pushed to the actual Interval struct upon pressing 'Select'.
uint16_t Temp_Edit_Val = 0;
uint8_t Time_Edit_H = 0, Time_Edit_M = 0, Time_Edit_S = 0;

/* Display Optimization Variables */
volatile bool LCD_Needs_Update = true;
char prev_lcd_row1[17] = {0}; // Cache for Row 1
char prev_lcd_row2[17] = {0}; // Cache for Row 2
uint32_t last_temp_read_time = 0;
bool flash_write_ok = true;

/* MODE_MT state, temperature-rate estimator and transition memory. */
float temp_rate_c_per_min = 0.0f;
float temp_rate_last_temp = 0.0f;
uint32_t temp_rate_last_time_ms = 0U;
bool temp_rate_initialized = false;
MTControlPhase_t mt_control_phase = MT_PHASE_APPROACH;
float mt_predicted_temp_c = 0.0f;
float mt_output_limit_ms = 0.0f;
float mt_precoast_output_ms = 0.0f;
uint8_t last_control_interval = 0U;
uint8_t last_control_mode = 0xFFU;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void Update_LCD(void);
void Process_Buttons(void);
void Init_Default_Intervals(void);
void Save_Settings_To_Flash(void);
void Load_Settings_From_Flash(void);
static void Copy_To_LCD_Row(char row[17], const char *text);
static uint32_t Interval_Duration_Seconds(uint8_t interval_index);
static void Sanitize_Settings(void);
static void Commit_Pending_Edit(void);
static void Stop_Heating_Control(void);
static bool Start_Profile(void);
static void Advance_Profile_If_Needed(uint32_t current_run_seconds);
static float Calculate_Profile_Setpoint(uint32_t current_run_seconds);
static void Reset_MT_Control(uint32_t now_ms);
static void Update_Temperature_Rate(uint32_t now_ms);
static void Apply_PID_Tunings(float kp, float ki, float kd);
static float MT_Hold_Output_Cap(float target_temp);
static void Update_MT_Control_Phase(float target_temp, float input_temp);
static float Limit_MT_Output(float requested_output, float target_temp, float input_temp);
static void Read_Temperature_Task(uint32_t now_ms);
static void Update_PID_And_SSR(uint32_t now_ms, uint32_t current_run_seconds);
static void Trip_Control_Fault(ControlFault_t fault);
static uint32_t Flash_Checksum(const Flash_Data_t *data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  EXTI Line Interrupt Callback
  * @note   Implements non-blocking software debounce.
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_8 && (current_time - last_time_PA8 > DEBOUNCE_DELAY)) {
        button_setting = 1; last_time_PA8 = current_time;
    } else if (GPIO_Pin == GPIO_PIN_9 && (current_time - last_time_PA9 > DEBOUNCE_DELAY)) {
        button_redirect = 1; last_time_PA9 = current_time;
    } else if (GPIO_Pin == GPIO_PIN_15 && (current_time - last_time_PB15 > DEBOUNCE_DELAY)) {
        button_select = 1; last_time_PB15 = current_time;
    } else if (GPIO_Pin == GPIO_PIN_11 && (current_time - last_time_PA11 > DEBOUNCE_DELAY)) {
        button_tang = 1; last_time_PA11 = current_time;
    } else if (GPIO_Pin == GPIO_PIN_10 && (current_time - last_time_PA10 > DEBOUNCE_DELAY)) {
        button_giam = 1; last_time_PA10 = current_time;
    }
}

/**
  * @brief  Timer Period Elapsed Callback (1Hz Real-Time Clock)
  * @note   Only increments time if the system is explicitly in the RUNNING state.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        if (System_Run_State == SYS_RUNNING && Control_Fault == CONTROL_FAULT_NONE) {
            Run_Total_Seconds++;
            // Trigger an LCD refresh every second so the clock UI updates smoothly.
            if (Current_UI_State == UI_STATE_MAIN) {
                LCD_Needs_Update = true;
            }
        }
    }
}

static void Copy_To_LCD_Row(char row[17], const char *text) {
    size_t length = strlen(text);
    if (length > 16U) length = 16U;
    memcpy(row, text, length);
}

static uint32_t Interval_Duration_Seconds(uint8_t interval_index) {
    if (interval_index >= MAX_INTERVALS) return 0U;

    return ((uint32_t)Intervals[interval_index].Time_Hour * 3600U) +
           ((uint32_t)Intervals[interval_index].Time_Min * 60U) +
           (uint32_t)Intervals[interval_index].Time_Sec;
}

static void Sanitize_Settings(void) {
    if (Total_Intervals < 1U || Total_Intervals > MAX_INTERVALS) {
        Total_Intervals = 1U;
    }

    for (uint8_t i = 0U; i < MAX_INTERVALS; i++) {
        if (Intervals[i].Mode != MODE_MT && Intervals[i].Mode != MODE_TIOT) {
            Intervals[i].Mode = MODE_MT;
        }
        if (Intervals[i].Temp > PROCESS_MAX_TEMP_C) {
            Intervals[i].Temp = PROCESS_MAX_TEMP_C;
        }
        if (Intervals[i].Time_Min > 59U) Intervals[i].Time_Min = 59U;
        if (Intervals[i].Time_Sec > 59U) Intervals[i].Time_Sec = 59U;
    }
}

static void Commit_Pending_Edit(void) {
    if (Setting_P_Index < 1U || Setting_P_Index > Total_Intervals) return;

    if (Current_UI_State == UI_STATE_SET_TEMP) {
        if (Temp_Edit_Val > PROCESS_MAX_TEMP_C) Temp_Edit_Val = PROCESS_MAX_TEMP_C;
        Intervals[Setting_P_Index - 1U].Temp = Temp_Edit_Val;
    } else if (Current_UI_State == UI_STATE_SET_TIME) {
        if (Time_Edit_M > 59U) Time_Edit_M = 59U;
        if (Time_Edit_S > 59U) Time_Edit_S = 59U;
        Intervals[Setting_P_Index - 1U].Time_Hour = Time_Edit_H;
        Intervals[Setting_P_Index - 1U].Time_Min = Time_Edit_M;
        Intervals[Setting_P_Index - 1U].Time_Sec = Time_Edit_S;
    }
}

static void Stop_Heating_Control(void) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    if (PID_GetMode(&pid_heater) != PID_MANUAL) {
        PID_SetMode(&pid_heater, PID_MANUAL);
    }
    Output = 0.0f;
    active_output = 0.0f;
    pid_heater.outputSum = 0.0f;
    mt_control_phase = MT_PHASE_APPROACH;
    mt_predicted_temp_c = Input;
    mt_output_limit_ms = 0.0f;
    mt_precoast_output_ms = 0.0f;
    last_control_interval = 0U;
    last_control_mode = 0xFFU;
}

static void Trip_Control_Fault(ControlFault_t fault) {
    if (fault == CONTROL_FAULT_NONE) return;

    Control_Fault = fault;
    System_Run_State = SYS_IDLE;
    Stop_Heating_Control();
    LCD_Needs_Update = true;
}

static bool Start_Profile(void) {
    Sanitize_Settings();

    if (Control_Fault != CONTROL_FAULT_NONE || !sensor_has_valid_sample) {
        if (!sensor_has_valid_sample && Control_Fault == CONTROL_FAULT_NONE) {
            Control_Fault = CONTROL_FAULT_SENSOR_DATA;
        }
        System_Run_State = SYS_IDLE;
        Stop_Heating_Control();
        return false;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    Run_Total_Seconds = 0U;
    Current_Interval = 1U;
    Current_Interval_Start_Sec = 0U;
    Current_Interval_Start_Temp = Input;
    Target_Run_Seconds = Interval_Duration_Seconds(0U);
    System_Run_State = SYS_RUNNING;
    if (primask == 0U) __enable_irq();

    Setpoint = Input;
    last_setpoint = Setpoint;
    Output = 0.0f;
    active_output = 0.0f;
    pid_heater.outputSum = 0.0f;
    pid_heater.lastInput = Input;
    pid_heater.lastTime = HAL_GetTick() - pid_heater.SampleTime;
    Apply_PID_Tunings(MT_APPROACH_KP, 0.0f, MT_APPROACH_KD);
    ki_is_active = false;
    windowStartTime = HAL_GetTick();
    Reset_MT_Control(windowStartTime);

    /* Immediately skip any zero-duration segments without heating them. */
    Advance_Profile_If_Needed(0U);
    return (System_Run_State == SYS_RUNNING);
}

static void Advance_Profile_If_Needed(uint32_t current_run_seconds) {
    uint8_t guard = 0U;

    while (System_Run_State == SYS_RUNNING &&
           current_run_seconds >= Target_Run_Seconds &&
           guard < MAX_INTERVALS) {
        guard++;

        if (Current_Interval < Total_Intervals) {
            const float boundary_setpoint = (float)Intervals[Current_Interval - 1U].Temp;
            const uint32_t boundary_time = Target_Run_Seconds;

            Current_Interval++;
            Current_Interval_Start_Sec = boundary_time;
            Current_Interval_Start_Temp = boundary_setpoint;
            Target_Run_Seconds += Interval_Duration_Seconds(Current_Interval - 1U);
            LCD_Needs_Update = true;
        } else {
            System_Run_State = SYS_COMPLETED;
            Stop_Heating_Control();
            LCD_Needs_Update = true;
        }
    }
}

static float Calculate_Profile_Setpoint(uint32_t current_run_seconds) {
    const uint8_t index = Current_Interval - 1U;
    const float target_temp = (float)Intervals[index].Temp;

    if (Intervals[index].Mode == MODE_MT) {
        return target_temp;
    }

    const uint32_t duration = Interval_Duration_Seconds(index);
    if (duration == 0U) return target_temp;

    uint32_t elapsed = current_run_seconds - Current_Interval_Start_Sec;
    if (elapsed > duration) elapsed = duration;

    const float fraction = (float)elapsed / (float)duration;
    float ramp_setpoint = Current_Interval_Start_Temp +
                          ((target_temp - Current_Interval_Start_Temp) * fraction);

    if (ramp_setpoint < 0.0f) ramp_setpoint = 0.0f;
    if (ramp_setpoint > (float)PROCESS_MAX_TEMP_C) ramp_setpoint = (float)PROCESS_MAX_TEMP_C;
    return ramp_setpoint;
}

static void Reset_MT_Control(uint32_t now_ms) {
    mt_control_phase = MT_PHASE_APPROACH;
    mt_predicted_temp_c = Input;
    mt_output_limit_ms = 0.0f;
    mt_precoast_output_ms = 0.0f;

    temp_rate_c_per_min = 0.0f;
    temp_rate_last_temp = Input;
    temp_rate_last_time_ms = now_ms;
    temp_rate_initialized = sensor_has_valid_sample && isfinite(Input);

    last_control_interval = 0U;
    last_control_mode = 0xFFU;
}

static void Update_Temperature_Rate(uint32_t now_ms) {
    if (!sensor_has_valid_sample || !isfinite(Input)) return;

    if (!temp_rate_initialized) {
        temp_rate_c_per_min = 0.0f;
        temp_rate_last_temp = Input;
        temp_rate_last_time_ms = now_ms;
        temp_rate_initialized = true;
        return;
    }

    const uint32_t elapsed_ms = now_ms - temp_rate_last_time_ms;
    if (elapsed_ms < TEMP_RATE_UPDATE_MS) return;

    const float elapsed_min = (float)elapsed_ms / 60000.0f;
    if (elapsed_min <= 0.0f) return;

    float instant_rate = (Input - temp_rate_last_temp) / elapsed_min;
    if (!isfinite(instant_rate)) instant_rate = 0.0f;

    if (instant_rate > TEMP_RATE_MAX_ABS_C_PER_MIN) {
        instant_rate = TEMP_RATE_MAX_ABS_C_PER_MIN;
    } else if (instant_rate < -TEMP_RATE_MAX_ABS_C_PER_MIN) {
        instant_rate = -TEMP_RATE_MAX_ABS_C_PER_MIN;
    }

    temp_rate_c_per_min +=
        TEMP_RATE_FILTER_ALPHA * (instant_rate - temp_rate_c_per_min);
    temp_rate_last_temp = Input;
    temp_rate_last_time_ms = now_ms;
}

static void Apply_PID_Tunings(float kp, float ki, float kd) {
    const bool changed =
        (fabsf(PID_GetKp(&pid_heater) - kp) > 0.0001f) ||
        (fabsf(PID_GetKi(&pid_heater) - ki) > 0.0001f) ||
        (fabsf(PID_GetKd(&pid_heater) - kd) > 0.0001f);

    if (changed) {
        PID_SetTunings(&pid_heater, kp, ki, kd, PID_P_ON_E);
    }
    ki_is_active = (ki > 0.0f);
}

static float MT_Hold_Output_Cap(float target_temp) {
    /* Heat loss rises strongly with furnace temperature. This adaptive ceiling
     * avoids an unrealistically small hold duty at high Setpoints while still
     * limiting stored heat at low Setpoints.
     */
    float cap_ms = 180.0f + (0.45f * target_temp);
    if (cap_ms < 250.0f) cap_ms = 250.0f;
    if (cap_ms > 850.0f) cap_ms = 850.0f;
    return cap_ms;
}

static void Enter_MT_Phase(MTControlPhase_t new_phase, float target_temp) {
    if (new_phase == mt_control_phase) return;

    if (new_phase == MT_PHASE_COAST) {
        if (active_output > 0.0f) mt_precoast_output_ms = active_output;
        Output = 0.0f;
        active_output = 0.0f;
        pid_heater.outputSum = 0.0f;
    } else if (new_phase == MT_PHASE_APPROACH) {
        Output = 0.0f;
        active_output = 0.0f;
        pid_heater.outputSum = 0.0f;
    } else { /* MT_PHASE_HOLD */
        float initial_bias = mt_precoast_output_ms * MT_HOLD_BIAS_FRACTION;
        const float hold_cap = MT_Hold_Output_Cap(target_temp);
        if (initial_bias > hold_cap) initial_bias = hold_cap;
        if (initial_bias < 0.0f) initial_bias = 0.0f;
        pid_heater.outputSum = initial_bias;
        Output = initial_bias;
        active_output = initial_bias;
        pid_heater.lastInput = Input;
        pid_heater.lastTime = HAL_GetTick() - pid_heater.SampleTime;
    }

    mt_control_phase = new_phase;
}

static void Update_MT_Control_Phase(float target_temp, float input_temp) {
    const float error = target_temp - input_temp;
    const float rising_rate =
        (temp_rate_c_per_min > 0.0f) ? temp_rate_c_per_min : 0.0f;

    mt_predicted_temp_c = input_temp +
                          (rising_rate * MT_THERMAL_LOOKAHEAD_MIN);

    switch (mt_control_phase) {
        case MT_PHASE_APPROACH:
            if (input_temp >= (target_temp + MT_HARD_CUTOFF_C) ||
                raw_temp >= (target_temp + MT_HARD_CUTOFF_C)) {
                Enter_MT_Phase(MT_PHASE_COAST, target_temp);
            } else if (error <= MT_PREDICT_ACTIVE_BAND_C &&
                       rising_rate >= MT_MIN_RISING_RATE_C_PER_MIN &&
                       mt_predicted_temp_c >= (target_temp - MT_PREDICT_MARGIN_C)) {
                /* Stored heat is predicted to carry the furnace to Setpoint. */
                Enter_MT_Phase(MT_PHASE_COAST, target_temp);
            } else if (fabsf(error) <= MT_HOLD_ENTRY_ERROR_C &&
                       fabsf(temp_rate_c_per_min) <= MT_HOLD_ENTRY_RATE_C_PER_MIN) {
                Enter_MT_Phase(MT_PHASE_HOLD, target_temp);
            }
            break;

        case MT_PHASE_COAST:
            if (error > MT_REHEAT_ERROR_C && temp_rate_c_per_min <= 0.0f) {
                Enter_MT_Phase(MT_PHASE_APPROACH, target_temp);
            } else if ((error >= MT_COAST_RELEASE_ERROR_C &&
                        temp_rate_c_per_min <= MT_COAST_RELEASE_RATE_C_PER_MIN) ||
                       (fabsf(error) <= MT_HOLD_ENTRY_ERROR_C &&
                        fabsf(temp_rate_c_per_min) <= MT_HOLD_ENTRY_RATE_C_PER_MIN)) {
                Enter_MT_Phase(MT_PHASE_HOLD, target_temp);
            }
            break;

        case MT_PHASE_HOLD:
        default:
            if (input_temp >= (target_temp + MT_HARD_CUTOFF_C) ||
                raw_temp >= (target_temp + MT_HARD_CUTOFF_C) ||
                (temp_rate_c_per_min > MT_HOLD_ENTRY_RATE_C_PER_MIN &&
                 mt_predicted_temp_c >= (target_temp + MT_HARD_CUTOFF_C))) {
                Enter_MT_Phase(MT_PHASE_COAST, target_temp);
            } else if (error > MT_REHEAT_ERROR_C) {
                Enter_MT_Phase(MT_PHASE_APPROACH, target_temp);
            }
            break;
    }
}

static float Limit_MT_Output(float requested_output,
                             float target_temp,
                             float input_temp) {
    float limited_output = requested_output;
    const float error = target_temp - input_temp;

    if (mt_control_phase == MT_PHASE_COAST ||
        input_temp >= (target_temp + MT_HARD_CUTOFF_C) ||
        raw_temp >= (target_temp + MT_HARD_CUTOFF_C)) {
        Output = 0.0f;
        pid_heater.outputSum = 0.0f;
        mt_output_limit_ms = 0.0f;
        return 0.0f;
    }

    if (mt_control_phase == MT_PHASE_APPROACH) {
        if (error <= 0.0f) {
            Output = 0.0f;
            pid_heater.outputSum = 0.0f;
            mt_output_limit_ms = 0.0f;
            return 0.0f;
        }

        if (error > 40.0f) {
            mt_output_limit_ms = MT_MAX_OUTPUT_FAR_MS;
        } else if (error > 20.0f) {
            mt_output_limit_ms = MT_MAX_OUTPUT_40_20_MS;
        } else if (error > 10.0f) {
            mt_output_limit_ms = MT_MAX_OUTPUT_20_10_MS;
        } else if (error > 5.0f) {
            mt_output_limit_ms = MT_MAX_OUTPUT_10_5_MS;
        } else if (error > 3.0f) {
            mt_output_limit_ms = MT_MAX_OUTPUT_5_3_MS;
        } else if (error > 2.0f) {
            mt_output_limit_ms = MT_MAX_OUTPUT_3_2_MS;
        } else if (error > 1.0f) {
            mt_output_limit_ms = MT_MAX_OUTPUT_2_1_MS;
        } else {
            mt_output_limit_ms = MT_MAX_OUTPUT_NEAR_MS;
        }
    } else { /* MT_PHASE_HOLD */
        mt_output_limit_ms = MT_Hold_Output_Cap(target_temp);

        /* A heater cannot actively cool. Stop heating immediately above target. */
        if (error <= 0.0f) {
            limited_output = 0.0f;
        }

        /* If the furnace is below target and cooling, guarantee one useful
         * 50 Hz SSR pulse instead of losing a command in the deadband.
         */
        if (error > 0.25f && temp_rate_c_per_min < 0.0f &&
            limited_output > 0.0f && limited_output < MT_HOLD_MIN_PULSE_MS) {
            limited_output = MT_HOLD_MIN_PULSE_MS;
        }
    }

    if (limited_output > mt_output_limit_ms) {
        limited_output = mt_output_limit_ms;
    }
    if (limited_output < 0.0f) limited_output = 0.0f;

    /* Back-calculation by clamping the stored integral to the real actuator limit. */
    if (pid_heater.outputSum > mt_output_limit_ms) {
        pid_heater.outputSum = mt_output_limit_ms;
    }
    if (pid_heater.outputSum < 0.0f) pid_heater.outputSum = 0.0f;
    if (Output > limited_output) Output = limited_output;

    return limited_output;
}

static void Read_Temperature_Task(uint32_t now_ms) {
    if ((uint32_t)(now_ms - last_temp_read_time) < SENSOR_SAMPLE_MS) return;
    last_temp_read_time = now_ms;

    if (!max31856_ready) {
        Trip_Control_Fault(CONTROL_FAULT_SENSOR_DATA);
        return;
    }

    max31856_fault_bits = MAX31856_ReadFault(&max31856);
    if (max31856_fault_bits != 0U) {
        invalid_temp_count = 0U;
        valid_recovery_count = 0U;
        Trip_Control_Fault(CONTROL_FAULT_MAX31856);
        return;
    }

    const float measured = MAX31856_ReadThermocoupleTemperature(&max31856);
    const bool valid = isfinite(measured) &&
                       measured >= SENSOR_MIN_VALID_C &&
                       measured <= SENSOR_MAX_VALID_C;

    if (!valid) {
        valid_recovery_count = 0U;
        if (invalid_temp_count < 255U) invalid_temp_count++;
        if (invalid_temp_count >= SENSOR_INVALID_LIMIT) {
            Trip_Control_Fault(CONTROL_FAULT_SENSOR_DATA);
        }
        return;
    }

    invalid_temp_count = 0U;
    raw_temp = measured;
    if (!sensor_has_valid_sample || !isfinite(filtered_temp)) {
        filtered_temp = measured;
    } else {
        filtered_temp += TEMP_FILTER_ALPHA * (measured - filtered_temp);
    }

    Current_Temp = filtered_temp;
    Input = filtered_temp;
    sensor_has_valid_sample = true;
    Update_Temperature_Rate(now_ms);

    if (measured >= OVERTEMP_TRIP_C || filtered_temp >= OVERTEMP_TRIP_C) {
        valid_recovery_count = 0U;
        Trip_Control_Fault(CONTROL_FAULT_OVERTEMP);
    } else if (Control_Fault == CONTROL_FAULT_OVERTEMP) {
        if (filtered_temp <= OVERTEMP_RESET_C) {
            if (valid_recovery_count < 255U) valid_recovery_count++;
            if (valid_recovery_count >= SENSOR_RECOVERY_VALID_SAMPLES) {
                Control_Fault = CONTROL_FAULT_NONE;
                valid_recovery_count = 0U;
            }
        } else {
            valid_recovery_count = 0U;
        }
    } else if (Control_Fault == CONTROL_FAULT_MAX31856 ||
               Control_Fault == CONTROL_FAULT_SENSOR_DATA) {
        if (valid_recovery_count < 255U) valid_recovery_count++;
        if (valid_recovery_count >= SENSOR_RECOVERY_VALID_SAMPLES) {
            Control_Fault = CONTROL_FAULT_NONE;
            valid_recovery_count = 0U;
        }
    } else {
        valid_recovery_count = 0U;
    }

    if (Current_UI_State == UI_STATE_MAIN) LCD_Needs_Update = true;
}

static void Update_PID_And_SSR(uint32_t now_ms, uint32_t current_run_seconds) {
    if (System_Run_State != SYS_RUNNING ||
        Control_Fault != CONTROL_FAULT_NONE ||
        !sensor_has_valid_sample) {
        Stop_Heating_Control();
        return;
    }

    Setpoint = Calculate_Profile_Setpoint(current_run_seconds);

    const uint8_t interval_index = Current_Interval - 1U;
    const uint8_t interval_mode = Intervals[interval_index].Mode;
    const bool is_mt_mode = (interval_mode == MODE_MT);

    /* Reset the supervisory state only when entering a new segment/mode.
     * TIOT changes Setpoint every second, so Setpoint changes alone must not reset it.
     */
    if (last_control_interval != Current_Interval ||
        last_control_mode != interval_mode) {
        mt_control_phase = MT_PHASE_APPROACH;
        mt_predicted_temp_c = Input;
        mt_output_limit_ms = 0.0f;
        mt_precoast_output_ms = 0.0f;
        pid_heater.outputSum = 0.0f;
        Output = 0.0f;
        active_output = 0.0f;
        last_control_interval = Current_Interval;
        last_control_mode = interval_mode;

        if (!is_mt_mode) {
            /* Always enter TIOT with its own non-integrating profile first. */
            Apply_PID_Tunings(Kp, 0.0f, Kd);
        }
    }

    if (PID_GetMode(&pid_heater) != PID_AUTOMATIC) {
        PID_SetMode(&pid_heater, PID_AUTOMATIC);
    }

    const float error = Setpoint - Input;
    const float abs_error = fabsf(error);

    /* A decreasing trajectory cannot be followed actively by a heater. */
    if (Setpoint < (last_setpoint - 0.25f)) {
        pid_heater.outputSum = 0.0f;
    }

    if (is_mt_mode) {
        Update_MT_Control_Phase(Setpoint, Input);

        if (mt_control_phase == MT_PHASE_HOLD) {
            Apply_PID_Tunings(MT_HOLD_KP, MT_HOLD_KI, MT_HOLD_KD);
        } else {
            /* Approach and coast never integrate stored heat demand. */
            Apply_PID_Tunings(MT_APPROACH_KP, 0.0f, MT_APPROACH_KD);
            pid_heater.outputSum = 0.0f;
        }
    } else {
        /* Preserve the original TIOT behavior and PID gains. */
        mt_control_phase = MT_PHASE_APPROACH;
        mt_predicted_temp_c = Input;
        mt_output_limit_ms = (float)WINDOW_SIZE;

        if (abs_error > INTEGRAL_DISABLE_BAND_C) {
            Apply_PID_Tunings(Kp, 0.0f, Kd);
        } else if (abs_error < INTEGRAL_ENABLE_BAND_C) {
            Apply_PID_Tunings(Kp, Ki, Kd);
        }

        if (error < -HEATER_CUTOFF_ABOVE_SP_C) {
            pid_heater.outputSum = 0.0f;
        }
    }

    if (PID_Compute(&pid_heater)) {
        active_output = Output;
    }

    if (is_mt_mode) {
        active_output = Limit_MT_Output(active_output, Setpoint, Input);
    } else if (Input > (Setpoint + HEATER_CUTOFF_ABOVE_SP_C) ||
               raw_temp > (Setpoint + HEATER_CUTOFF_ABOVE_SP_C)) {
        active_output = 0.0f;
        Output = 0.0f;
        pid_heater.outputSum = 0.0f;
    }

    if (active_output < 0.0f) active_output = 0.0f;
    if (active_output > (float)WINDOW_SIZE) active_output = (float)WINDOW_SIZE;
    last_setpoint = Setpoint;

    /* Time-proportional SSR drive: active_output is ON-time in a 1000 ms window. */
    const uint32_t ms_in_window =
        (uint32_t)(now_ms - windowStartTime) % WINDOW_SIZE;

    if (active_output <= OUTPUT_DEADBAND_MS) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    } else if (active_output >= ((float)WINDOW_SIZE - OUTPUT_DEADBAND_MS)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,
                          (ms_in_window < (uint32_t)active_output)
                              ? GPIO_PIN_SET
                              : GPIO_PIN_RESET);
    }
}

/**
  * @brief  Main Event Loop for Button Processing
  * @note   Decoupled from ISR to keep interrupts brief. Handles all state machine transitions.
  */
void Process_Buttons(void) {
    uint8_t setting_event;
    uint8_t redirect_event;
    uint8_t select_event;
    uint8_t up_event;
    uint8_t down_event;

    /* Atomically snapshot and clear ISR-owned event flags. */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    setting_event = button_setting;   button_setting = 0U;
    redirect_event = button_redirect; button_redirect = 0U;
    select_event = button_select;     button_select = 0U;
    up_event = button_tang;           button_tang = 0U;
    down_event = button_giam;         button_giam = 0U;
    if (primask == 0U) __enable_irq();

    // ---------------------------------------------------------
    // 1. SETTING / RUN BUTTON (PA8)
    // ---------------------------------------------------------
    if (setting_event) {
        if (Current_UI_State == UI_STATE_MAIN) {
            Current_UI_State = UI_STATE_SET_INTERVAL;
            System_Run_State = SYS_IDLE;
            Stop_Heating_Control();
        } else {
            /* PA8 means Save & Exit even when currently inside an edit screen. */
            Commit_Pending_Edit();
            Sanitize_Settings();
            Save_Settings_To_Flash();
            Current_UI_State = UI_STATE_MAIN;
            (void)Start_Profile();
        }
        LCD_Needs_Update = true;
        return; /* Do not apply simultaneous events to the newly-entered UI state. */
    }

    // ---------------------------------------------------------
    // 2. REDIRECT BUTTON (PA9)
    // ---------------------------------------------------------
    if (redirect_event) {
        if (Current_UI_State == UI_STATE_SET_P) {
            Setting_P_Index++;
            if (Setting_P_Index > Total_Intervals) Setting_P_Index = 1U;
        } else if (Current_UI_State == UI_STATE_SET_TEMP) {
            Temp_Digit_Index = (Temp_Digit_Index + 1U) % 4U;
        } else if (Current_UI_State == UI_STATE_SET_TIME) {
            Time_Field_Index = (Time_Field_Index + 1U) % 3U;
        }
        LCD_Needs_Update = true;
    }

    // ---------------------------------------------------------
    // 3. SELECT / OK BUTTON (PB15)
    // ---------------------------------------------------------
    if (select_event) {
        switch (Current_UI_State) {
            case UI_STATE_SET_INTERVAL:
                Sanitize_Settings();
                Current_UI_State = UI_STATE_SET_P;
                Setting_P_Index = 1U;
                Menu_Cursor = 0U;
                break;

            case UI_STATE_SET_P:
                if (Menu_Cursor == 0U) {
                    Intervals[Setting_P_Index - 1U].Mode =
                        (Intervals[Setting_P_Index - 1U].Mode == MODE_MT) ? MODE_TIOT : MODE_MT;
                } else if (Menu_Cursor == 1U) {
                    Temp_Edit_Val = Intervals[Setting_P_Index - 1U].Temp;
                    Temp_Digit_Index = 0U;
                    Current_UI_State = UI_STATE_SET_TEMP;
                } else {
                    Time_Edit_H = Intervals[Setting_P_Index - 1U].Time_Hour;
                    Time_Edit_M = Intervals[Setting_P_Index - 1U].Time_Min;
                    Time_Edit_S = Intervals[Setting_P_Index - 1U].Time_Sec;
                    Time_Field_Index = 0U;
                    Current_UI_State = UI_STATE_SET_TIME;
                }
                break;

            case UI_STATE_SET_TEMP:
            case UI_STATE_SET_TIME:
                Commit_Pending_Edit();
                Current_UI_State = UI_STATE_SET_P;
                break;

            default:
                break;
        }
        LCD_Needs_Update = true;
    }

    // ---------------------------------------------------------
    // 4. UP (PA11) & DOWN (PA10)
    // If both arrive together, ignore the contradictory command.
    // ---------------------------------------------------------
    if ((up_event != 0U) != (down_event != 0U)) {
        const bool is_up = (up_event != 0U);

        switch (Current_UI_State) {
            case UI_STATE_SET_INTERVAL:
                if (is_up && Total_Intervals < MAX_INTERVALS) Total_Intervals++;
                else if (!is_up && Total_Intervals > 1U) Total_Intervals--;
                break;

            case UI_STATE_SET_P:
                if (is_up) Menu_Cursor = (Menu_Cursor > 0U) ? (Menu_Cursor - 1U) : 2U;
                else Menu_Cursor = (Menu_Cursor < 2U) ? (Menu_Cursor + 1U) : 0U;
                break;

            case UI_STATE_SET_TEMP: {
                uint8_t d[4];
                d[0] = (uint8_t)(Temp_Edit_Val / 1000U);
                d[1] = (uint8_t)((Temp_Edit_Val / 100U) % 10U);
                d[2] = (uint8_t)((Temp_Edit_Val / 10U) % 10U);
                d[3] = (uint8_t)(Temp_Edit_Val % 10U);

                if (is_up) d[Temp_Digit_Index] = (d[Temp_Digit_Index] == 9U) ? 0U : (d[Temp_Digit_Index] + 1U);
                else d[Temp_Digit_Index] = (d[Temp_Digit_Index] == 0U) ? 9U : (d[Temp_Digit_Index] - 1U);

                Temp_Edit_Val = (uint16_t)(d[0] * 1000U + d[1] * 100U + d[2] * 10U + d[3]);
                if (Temp_Edit_Val > PROCESS_MAX_TEMP_C) Temp_Edit_Val = PROCESS_MAX_TEMP_C;
                break;
            }

            case UI_STATE_SET_TIME:
                if (Time_Field_Index == 0U) {
                    if (is_up) Time_Edit_H = (Time_Edit_H == 99U) ? 0U : (Time_Edit_H + 1U);
                    else Time_Edit_H = (Time_Edit_H == 0U) ? 99U : (Time_Edit_H - 1U);
                } else if (Time_Field_Index == 1U) {
                    if (is_up) Time_Edit_M = (Time_Edit_M == 59U) ? 0U : (Time_Edit_M + 1U);
                    else Time_Edit_M = (Time_Edit_M == 0U) ? 59U : (Time_Edit_M - 1U);
                } else {
                    if (is_up) Time_Edit_S = (Time_Edit_S == 59U) ? 0U : (Time_Edit_S + 1U);
                    else Time_Edit_S = (Time_Edit_S == 0U) ? 59U : (Time_Edit_S - 1U);
                }
                break;

            default:
                break;
        }
        LCD_Needs_Update = true;
    }
}

/**
  * @brief  Advanced Display Renderer
  * @note   Implements String Double-Buffering. We assemble the entire row string in RAM,
  * compare it to what is currently on the screen, and only execute I2C writes
  * if the string actually changed. This completely eliminates UI flickering.
  */
void Update_LCD(void) {
    if (!LCD_Needs_Update) return;
    LCD_Needs_Update = false;

    char row1[17];
    char row2[17];
    char temp_buf[32];

    memset(row1, ' ', 16U); row1[16] = '\0';
    memset(row2, ' ', 16U); row2[16] = '\0';

    switch (Current_UI_State) {
        case UI_STATE_MAIN: {
            if (Control_Fault != CONTROL_FAULT_NONE) {
                if (Control_Fault == CONTROL_FAULT_OVERTEMP) {
                    snprintf(temp_buf, sizeof(temp_buf), "==OVER TEMP!==");
                } else {
                    snprintf(temp_buf, sizeof(temp_buf), "==SENSOR ERROR==");
                }
                Copy_To_LCD_Row(row1, temp_buf);
                Copy_To_LCD_Row(row2, "SSR: OFF");
            } else {
                const int display_temp = sensor_has_valid_sample ? (int)lroundf(Current_Temp) : 0;
                if (System_Run_State == SYS_IDLE) {
                    snprintf(temp_buf, sizeof(temp_buf), "Temp:%4d" "\xDF" "C P0/%u",
                             display_temp, (unsigned int)Total_Intervals);
                } else if (System_Run_State == SYS_COMPLETED) {
                    snprintf(temp_buf, sizeof(temp_buf), "Temp:%4d" "\xDF" "C DONE", display_temp);
                } else {
                    snprintf(temp_buf, sizeof(temp_buf), "Temp:%4d" "\xDF" "C P%u/%u",
                             display_temp, (unsigned int)Current_Interval,
                             (unsigned int)Total_Intervals);
                }
                Copy_To_LCD_Row(row1, temp_buf);

                const uint32_t elapsed = Run_Total_Seconds;
                const uint32_t hours = elapsed / 3600U;
                const uint32_t minutes = (elapsed / 60U) % 60U;
                const uint32_t seconds = elapsed % 60U;
                if (hours <= 99U) {
                    snprintf(temp_buf, sizeof(temp_buf), "Time: %02lu:%02lu:%02lu",
                             (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
                } else {
                    snprintf(temp_buf, sizeof(temp_buf), "Time:%03lu:%02lu:%02lu",
                             (unsigned long)hours, (unsigned long)minutes, (unsigned long)seconds);
                }
                Copy_To_LCD_Row(row2, temp_buf);
            }
            break;
        }

        case UI_STATE_SET_INTERVAL:
            Copy_To_LCD_Row(row1, "[SET INTERVAL]");
            snprintf(temp_buf, sizeof(temp_buf), "Quantity: %u", (unsigned int)Total_Intervals);
            Copy_To_LCD_Row(row2, temp_buf);
            break;

        case UI_STATE_SET_P:
            snprintf(temp_buf, sizeof(temp_buf), "==== SET P%u ====", (unsigned int)Setting_P_Index);
            Copy_To_LCD_Row(row1, temp_buf);
            if (Menu_Cursor == 0U) {
                snprintf(temp_buf, sizeof(temp_buf), ">Mode: %s",
                         (Intervals[Setting_P_Index - 1U].Mode == MODE_MT) ? "MT" : "TIOT");
            } else if (Menu_Cursor == 1U) {
                snprintf(temp_buf, sizeof(temp_buf), ">Set Temp");
            } else {
                snprintf(temp_buf, sizeof(temp_buf), ">Set Time");
            }
            Copy_To_LCD_Row(row2, temp_buf);
            break;

        case UI_STATE_SET_TEMP:
            snprintf(temp_buf, sizeof(temp_buf), "[SET TEMP P%u]", (unsigned int)Setting_P_Index);
            Copy_To_LCD_Row(row1, temp_buf);
            snprintf(temp_buf, sizeof(temp_buf), "Thres: %04u " "\xDF" "C", (unsigned int)Temp_Edit_Val);
            Copy_To_LCD_Row(row2, temp_buf);
            break;

        case UI_STATE_SET_TIME:
            snprintf(temp_buf, sizeof(temp_buf), "[SET TIME P%u]", (unsigned int)Setting_P_Index);
            Copy_To_LCD_Row(row1, temp_buf);
            snprintf(temp_buf, sizeof(temp_buf), "Time: %02u:%02u:%02u",
                     (unsigned int)Time_Edit_H, (unsigned int)Time_Edit_M,
                     (unsigned int)Time_Edit_S);
            Copy_To_LCD_Row(row2, temp_buf);
            break;

        default:
            break;
    }

    if (strcmp(row1, prev_lcd_row1) != 0) {
        LCDI2C_setCursor(0U, 0U);
        LCDI2C_write_String(row1);
        strcpy(prev_lcd_row1, row1);
    }
    if (strcmp(row2, prev_lcd_row2) != 0) {
        LCDI2C_setCursor(0U, 1U);
        LCDI2C_write_String(row2);
        strcpy(prev_lcd_row2, row2);
    }

    if (Current_UI_State == UI_STATE_SET_TEMP) {
        LCDI2C_setCursor((uint8_t)(7U + Temp_Digit_Index), 1U);
        LCDI2C_blink_on();
    } else if (Current_UI_State == UI_STATE_SET_TIME) {
        const uint8_t col = (Time_Field_Index == 0U) ? 6U :
                            ((Time_Field_Index == 1U) ? 9U : 12U);
        LCDI2C_setCursor(col, 1U);
        LCDI2C_blink_on();
    } else {
        LCDI2C_blink_off();
    }
}

/**
  * @brief  Fallback Initializer
  * @note   Used when Flash memory is empty (new MCU) or corrupted.
  */
void Init_Default_Intervals(void) {
    memset(Intervals, 0, sizeof(Intervals));
    for (uint8_t i = 0U; i < MAX_INTERVALS; i++) {
        Intervals[i].Mode = MODE_MT;
    }
    Total_Intervals = 1U;
}

static uint32_t Flash_Checksum(const Flash_Data_t *data) {
    const uint8_t *bytes = (const uint8_t *)data;
    const size_t count = offsetof(Flash_Data_t, Checksum);
    uint32_t hash = 2166136261UL; /* FNV-1a */

    for (size_t i = 0U; i < count; i++) {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}

/**
  * @brief  Write validated parameters to the last Flash page.
  */
void Save_Settings_To_Flash(void) {
    Sanitize_Settings();

    Flash_Data_t flash_data;
    memset(&flash_data, 0, sizeof(flash_data));
    flash_data.MagicWord = FLASH_MAGIC_WORD;
    flash_data.Version = FLASH_DATA_VERSION;
    flash_data.TotalIntervals = (uint32_t)Total_Intervals;
    memcpy(flash_data.Intervals, Intervals, sizeof(Intervals));
    flash_data.Checksum = Flash_Checksum(&flash_data);

    const uint32_t *data_ptr = (const uint32_t *)&flash_data;
    const uint16_t num_words = (uint16_t)((sizeof(Flash_Data_t) + 3U) / 4U);
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t page_error = 0U;
    HAL_StatusTypeDef status;

    flash_write_ok = false;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();

    status = HAL_FLASH_Unlock();
    if (status == HAL_OK) {
        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Banks = FLASH_BANK_1;
        erase.PageAddress = FLASH_STORAGE_ADDR;
        erase.NbPages = 1U;

        status = HAL_FLASHEx_Erase(&erase, &page_error);
        if (status == HAL_OK) {
            for (uint16_t i = 0U; i < num_words; i++) {
                status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                                           FLASH_STORAGE_ADDR + ((uint32_t)i * 4U),
                                           data_ptr[i]);
                if (status != HAL_OK) break;
            }
        }
        (void)HAL_FLASH_Lock();
    }

    if (primask == 0U) __enable_irq();

    if (status == HAL_OK) {
        const Flash_Data_t *stored = (const Flash_Data_t *)FLASH_STORAGE_ADDR;
        flash_write_ok = (stored->MagicWord == FLASH_MAGIC_WORD) &&
                         (stored->Version == FLASH_DATA_VERSION) &&
                         (stored->Checksum == Flash_Checksum(stored));
    }
}

/**
  * @brief  Load settings only when magic, version, size fields and checksum are valid.
  */
void Load_Settings_From_Flash(void) {
    const Flash_Data_t *flash_data = (const Flash_Data_t *)FLASH_STORAGE_ADDR;
    const bool header_valid = (flash_data->MagicWord == FLASH_MAGIC_WORD) &&
                              (flash_data->Version == FLASH_DATA_VERSION) &&
                              (flash_data->TotalIntervals >= 1U) &&
                              (flash_data->TotalIntervals <= MAX_INTERVALS);

    if (header_valid && flash_data->Checksum == Flash_Checksum(flash_data)) {
        Total_Intervals = (uint8_t)flash_data->TotalIntervals;
        memcpy(Intervals, flash_data->Intervals, sizeof(Intervals));
        Sanitize_Settings();
    } else {
        Init_Default_Intervals();
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // Load persistence data into RAM *before* starting any operation logic.
    Load_Settings_From_Flash();

  //====== Init LCD16x2 I2C ======//
    LCDI2C_init(&hi2c2, 0x27, 16, 2);
    LCDI2C_backlight();
    LCDI2C_clear();

  //====== Init timer 2 (1Hz) ======//
    HAL_TIM_Base_Start_IT(&htim2);

  //====== Init MAX31856 ======//
    max31856_ready = MAX31856_Init(&max31856, &hspi1, GPIOA, GPIO_PIN_15);
    if (max31856_ready) {
        MAX31856_SetThermocoupleType(&max31856, MAX31856_TCTYPE_S);
        MAX31856_SetNoiseFilter(&max31856, MAX31856_NOISE_FILTER_50HZ);
        MAX31856_SetConversionMode(&max31856, MAX31856_CONTINUOUS);
    } else {
        Control_Fault = CONTROL_FAULT_SENSOR_DATA;
    }

  //====== Init PID ======//
    Setpoint = 0.0f;
    PID_Init(&pid_heater, &Input, &Output, &Setpoint, Kp, Ki, Kd, PID_P_ON_E, PID_DIRECT);
    PID_SetMode(&pid_heater, PID_MANUAL); // By default, it's off until you press RUN
    PID_SetOutputLimits(&pid_heater, 0.0f, (float)WINDOW_SIZE);
    PID_SetSampleTime(&pid_heater, WINDOW_SIZE);
    windowStartTime = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      const uint32_t now_ms = HAL_GetTick();

      Read_Temperature_Task(now_ms);
      Process_Buttons();

      const uint32_t current_run_seconds = Run_Total_Seconds;
      Advance_Profile_If_Needed(current_run_seconds);
      Update_PID_And_SSR(now_ms, current_run_seconds);
      Update_LCD();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
