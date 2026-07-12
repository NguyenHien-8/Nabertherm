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
  * - Non-Volatile Storage: Uses Flash Page 63 to save parameters securely.
******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiquidCrystal_I2C.h"
#include "MAX31856.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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

/* * Data structure for a single heating interval (Segment).
 * Grouped into a struct for memory contiguity and easy array management.
 */
typedef struct {
    IntervalMode_t Mode;
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

// Flash Storage Architecture
// STM32F103C8T6 has 64KB Flash. Page 63 is the very last page (1KB size).
// Using the last page ensures we do not accidentally overwrite application code.
#define FLASH_STORAGE_ADDR 0x0800FC00
#define FLASH_MAGIC_WORD   0xAABBCCDD
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

/* * Flash Storage Wrapper Structure
 * This struct perfectly aligns our variables for easy writing/reading
 * to the internal flash using 32-bit Words.
 */
typedef struct {
    uint32_t MagicWord;      // Validation flag (0xAABBCCDD).
    uint32_t TotalIntervals; // Cast from uint8_t to uint32_t for alignment.
    Interval_TypeDef Intervals[MAX_INTERVALS]; // Payload.
} Flash_Data_t;

/*
 * Multi-Pin Independent Debounce Trackers
 * WHY: If we used a single global `last_exti_time`, pressing one button
 * would inadvertently lock out ALL other buttons for 200ms. By isolating
 * the timestamp per pin, simultaneous or rapid multi-button presses are handled correctly.
 */
//========== Multi-Pin Independent Debounce Variables ==========//
volatile uint32_t last_time_PA8 = 0;
volatile uint32_t last_time_PA9 = 0;
volatile uint32_t last_time_PA10 = 0;
volatile uint32_t last_time_PA11 = 0;
volatile uint32_t last_time_PB15 = 0;

/*
 * Button Event Flags
 * WHY: The EXTI ISR should be as short as possible. We only set flags here.
 * The heavy logic (state changing, flash writing) is deferred to Process_Buttons()
 * in the main loop.
 */
volatile uint8_t button_setting = 0;    // PA8 -> Setting/Run
volatile uint8_t button_redirect = 0;   // PA9 -> Redirect
volatile uint8_t button_select = 0;     // PB15 -> OK/Select
volatile uint8_t button_tang = 0;       // PA11 -> UP
volatile uint8_t button_giam = 0;       // PA10 -> DOWN

/* System Logic & Payload Variables */
Interval_TypeDef Intervals[MAX_INTERVALS];
uint8_t Total_Intervals = 1;

SystemState_t System_Run_State = SYS_IDLE;
UIState_t Current_UI_State = UI_STATE_MAIN;
uint8_t Current_Interval = 1;

// Global Run Clock. Never reset between interval transitions.
uint8_t Run_Hour = 0, Run_Min = 0, Run_Sec = 0;

/*
 * Target_Run_Seconds is the absolute cumulative target time.
 * WHY: Instead of resetting the clock to 0 at the start of a new interval
 * (which looks bad on the UI), we calculate the absolute total seconds needed
 * to finish the current interval and compare it against the running clock.
 */
uint32_t Target_Run_Seconds = 0;
float Current_Temp = 0.0f;

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
bool LCD_Needs_Update = true;
char prev_lcd_row1[17] = {0}; // Cache for Row 1
char prev_lcd_row2[17] = {0}; // Cache for Row 2
uint32_t last_temp_read_time = 0;

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
        if (System_Run_State == SYS_RUNNING) {
            Run_Sec++;
            if (Run_Sec >= 60) {
                Run_Sec = 0;
                Run_Min++;
                if (Run_Min >= 60) {
                    Run_Min = 0;
                    Run_Hour++;
                }
            }
            // Trigger an LCD refresh every second so the clock UI updates smoothly.
            if (Current_UI_State == UI_STATE_MAIN) {
                LCD_Needs_Update = true;
            }
        }
    }
}

/**
  * @brief  Main Event Loop for Button Processing
  * @note   Decoupled from ISR to keep interrupts brief. Handles all state machine transitions.
  */
void Process_Buttons(void) {

    // ---------------------------------------------------------
    // 1. SETTING / RUN BUTTON (PA8)
    // Acts as the main toggle between Operation and Configuration.
    // ---------------------------------------------------------
    if (button_setting) {
        button_setting = 0;
        if (Current_UI_State == UI_STATE_MAIN) {
            // Enter Settings Mode. Pause operation immediately for safety.
            Current_UI_State = UI_STATE_SET_INTERVAL;
            System_Run_State = SYS_IDLE;
        } else {
            // Exit Settings Mode.
            // Critical Step: Save to Non-Volatile Memory on exit.
            Save_Settings_To_Flash();

            Current_UI_State = UI_STATE_MAIN;

            // Hard reset the timeline to begin a fresh run from Interval 1.
            Run_Hour = 0; Run_Min = 0; Run_Sec = 0;
            Current_Interval = 1;
            System_Run_State = SYS_RUNNING;

            // Pre-calculate the first auto-step target threshold in absolute seconds.
            Target_Run_Seconds = Intervals[0].Time_Hour * 3600 +
                                 Intervals[0].Time_Min * 60 +
                                 Intervals[0].Time_Sec;
        }
        LCD_Needs_Update = true;
    }

    // ---------------------------------------------------------
    // 2. REDIRECT BUTTON (PA9)
    // Handles horizontal navigation (changing items at the same hierarchy level).
    // ---------------------------------------------------------
    if (button_redirect) {
        button_redirect = 0;
        if (Current_UI_State == UI_STATE_SET_P) {
            // Cycle through P1, P2, P3...
            Setting_P_Index++;
            if (Setting_P_Index > Total_Intervals) Setting_P_Index = 1;
        } else if (Current_UI_State == UI_STATE_SET_TEMP) {
            // Move blinking cursor across the 4 digits (Thousands -> Ones).
            Temp_Digit_Index = (Temp_Digit_Index + 1) % 4;
        } else if (Current_UI_State == UI_STATE_SET_TIME) {
            // Move blinking cursor: HH -> MM -> SS.
            Time_Field_Index = (Time_Field_Index + 1) % 3;
        }
        LCD_Needs_Update = true;
    }

    // ---------------------------------------------------------
    // 3. SELECT / OK BUTTON (PB15)
    // Handles vertical navigation (drilling deeper into menus) and saving edits.
    // ---------------------------------------------------------
    if (button_select) {
        button_select = 0;
        switch (Current_UI_State) {
            case UI_STATE_SET_INTERVAL:
                Current_UI_State = UI_STATE_SET_P;
                Setting_P_Index = 1;
                Menu_Cursor = 0;
                break;
            case UI_STATE_SET_P:
                if (Menu_Cursor == 0) {
                    // Direct toggle without needing a sub-menu.
                    Intervals[Setting_P_Index-1].Mode = (Intervals[Setting_P_Index-1].Mode == MODE_MT) ? MODE_TIOT : MODE_MT;
                } else if (Menu_Cursor == 1) {
                    // Load payload into edit buffer before entering Edit mode.
                    Temp_Edit_Val = Intervals[Setting_P_Index-1].Temp;
                    Temp_Digit_Index = 0;
                    Current_UI_State = UI_STATE_SET_TEMP;
                } else if (Menu_Cursor == 2) {
                    Time_Edit_H = Intervals[Setting_P_Index-1].Time_Hour;
                    Time_Edit_M = Intervals[Setting_P_Index-1].Time_Min;
                    Time_Edit_S = Intervals[Setting_P_Index-1].Time_Sec;
                    Time_Field_Index = 0;
                    Current_UI_State = UI_STATE_SET_TIME;
                }
                break;
            case UI_STATE_SET_TEMP:
                // Commit temp buffer back to payload and jump up one level.
                Intervals[Setting_P_Index-1].Temp = Temp_Edit_Val;
                Current_UI_State = UI_STATE_SET_P;
                break;
            case UI_STATE_SET_TIME:
                // Commit time buffers back to payload and jump up one level.
                Intervals[Setting_P_Index-1].Time_Hour = Time_Edit_H;
                Intervals[Setting_P_Index-1].Time_Min = Time_Edit_M;
                Intervals[Setting_P_Index-1].Time_Sec = Time_Edit_S;
                Current_UI_State = UI_STATE_SET_P;
                break;
            default: break;
        }
        LCD_Needs_Update = true;
    }

    // ---------------------------------------------------------
    // 4. UP (PA11) & DOWN (PA10) BUTTONS
    // Increments or decrements values and handles list scrolling.
    // ---------------------------------------------------------
    if (button_tang || button_giam) {
        bool is_up = button_tang;
        button_tang = 0; button_giam = 0;

        switch (Current_UI_State) {
            case UI_STATE_SET_INTERVAL:
                // Bound limits defined by architecture constraints.
                if (is_up && Total_Intervals < MAX_INTERVALS) Total_Intervals++;
                else if (!is_up && Total_Intervals > 1) Total_Intervals--;
                break;

            case UI_STATE_SET_P:
                // Scroll Menu: Mode (0) <-> Temp (1) <-> Time (2). Wraps around.
                if (is_up) Menu_Cursor = (Menu_Cursor > 0) ? Menu_Cursor - 1 : 2;
                else Menu_Cursor = (Menu_Cursor < 2) ? Menu_Cursor + 1 : 0;
                break;

            case UI_STATE_SET_TEMP: {
                // Break integer into array for single-digit manipulation.
                uint8_t d[4];
                d[0] = Temp_Edit_Val / 1000;
                d[1] = (Temp_Edit_Val / 100) % 10;
                d[2] = (Temp_Edit_Val / 10) % 10;
                d[3] = Temp_Edit_Val % 10;

                // Edit the specifically targeted digit. Wrap 0-9.
                if (is_up) d[Temp_Digit_Index] = (d[Temp_Digit_Index] == 9) ? 0 : d[Temp_Digit_Index] + 1;
                else d[Temp_Digit_Index] = (d[Temp_Digit_Index] == 0) ? 9 : d[Temp_Digit_Index] - 1;

                // Reconstruct integer.
                Temp_Edit_Val = d[0]*1000 + d[1]*100 + d[2]*10 + d[3];
                break;
            }

            case UI_STATE_SET_TIME: {
                if (Time_Field_Index == 0) {
                    if (is_up) Time_Edit_H = (Time_Edit_H == 99) ? 0 : Time_Edit_H + 1;
                    else Time_Edit_H = (Time_Edit_H == 0) ? 99 : Time_Edit_H - 1;
                } else if (Time_Field_Index == 1) {
                    if (is_up) Time_Edit_M = (Time_Edit_M == 59) ? 0 : Time_Edit_M + 1;
                    else Time_Edit_M = (Time_Edit_M == 0) ? 59 : Time_Edit_M - 1;
                } else if (Time_Field_Index == 2) {
                    if (is_up) Time_Edit_S = (Time_Edit_S == 59) ? 0 : Time_Edit_S + 1;
                    else Time_Edit_S = (Time_Edit_S == 0) ? 59 : Time_Edit_S - 1;
                }
                break;
            }
            default: break;
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
    char temp_buf[21];

    // Pre-pad with spaces. This inherently clears any old residual characters
    // at the end of the line without needing an explicit lcd_clear() command.
    memset(row1, ' ', 16); row1[16] = '\0';
    memset(row2, ' ', 16); row2[16] = '\0';

    switch (Current_UI_State) {
        case UI_STATE_MAIN:
            // String Concatenation Fix: Separating "\xDF" (Degree Symbol) from "C"
            // prevents the GCC compiler from parsing "C" as part of the hex sequence.
            if (System_Run_State == SYS_IDLE) {
            	sprintf(temp_buf, "Temp:%4d" "\xDF" "C P0/%d", (int)Current_Temp, Total_Intervals);
            } else if (System_Run_State == SYS_COMPLETED) {
            	sprintf(temp_buf, "Temp:%4d" "\xDF" "C DONE", (int)Current_Temp);
            } else {
            	sprintf(temp_buf, "Temp:%4d" "\xDF" "C P%d/%d", (int)Current_Temp, Current_Interval, Total_Intervals);
            }
            memcpy(row1, temp_buf, strlen(temp_buf));

            sprintf(temp_buf, "Time: %02d:%02d:%02d", Run_Hour, Run_Min, Run_Sec);
            memcpy(row2, temp_buf, strlen(temp_buf));
            break;

        case UI_STATE_SET_INTERVAL:
            sprintf(temp_buf, "[SET INTERVAL]");
            memcpy(row1, temp_buf, strlen(temp_buf));
            sprintf(temp_buf, "Quantity: %d", Total_Intervals);
            memcpy(row2, temp_buf, strlen(temp_buf));
            break;

        case UI_STATE_SET_P:
            sprintf(temp_buf, "==== SET P%d ====", Setting_P_Index);
            memcpy(row1, temp_buf, strlen(temp_buf));

            // Dynamic render based on Menu_Cursor position.
            if (Menu_Cursor == 0) {
                sprintf(temp_buf, ">Mode: %s", (Intervals[Setting_P_Index-1].Mode == MODE_MT) ? "MT" : "TIOT");
            } else if (Menu_Cursor == 1) {
                sprintf(temp_buf, ">Set Temp");
            } else if (Menu_Cursor == 2) {
                sprintf(temp_buf, ">Set Time");
            }
            memcpy(row2, temp_buf, strlen(temp_buf));
            break;

        case UI_STATE_SET_TEMP:
            sprintf(temp_buf, "[SET TEMP P%d]", Setting_P_Index);
            memcpy(row1, temp_buf, strlen(temp_buf));
            sprintf(temp_buf, "Thres: %04d " "\xDF" "C", Temp_Edit_Val);
            memcpy(row2, temp_buf, strlen(temp_buf));
            break;

        case UI_STATE_SET_TIME:
            sprintf(temp_buf, "[SET TIME P%d]", Setting_P_Index);
            memcpy(row1, temp_buf, strlen(temp_buf));
            sprintf(temp_buf, "Time: %02d:%02d:%02d", Time_Edit_H, Time_Edit_M, Time_Edit_S);
            memcpy(row2, temp_buf, strlen(temp_buf));
            break;
    }

    // Execute I2C write only if the buffered string differs from the cached string.
    if (strcmp(row1, prev_lcd_row1) != 0) {
        LCDI2C_setCursor(0, 0);
        LCDI2C_write_String(row1);
        strcpy(prev_lcd_row1, row1);
    }
    if (strcmp(row2, prev_lcd_row2) != 0) {
        LCDI2C_setCursor(0, 1);
        LCDI2C_write_String(row2);
        strcpy(prev_lcd_row2, row2);
    }

    // Hardware-level cursor blinking.
    // Calculates absolute column positions based on what field is being edited.
    if (Current_UI_State == UI_STATE_SET_TEMP) {
        LCDI2C_setCursor(7 + Temp_Digit_Index, 1); // "Thres: " is 7 chars
        LCDI2C_blink_on();
    } else if (Current_UI_State == UI_STATE_SET_TIME) {
        // "Time: hh:mm:ss" offsets: hr=7, min=10, sec=13
        uint8_t col = (Time_Field_Index == 0) ? 7 : ((Time_Field_Index == 1) ? 10 : 13);
        LCDI2C_setCursor(col, 1);
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
    for (int i = 0; i < MAX_INTERVALS; i++) {
        Intervals[i].Mode = MODE_MT;
        Intervals[i].Temp = 0;
        Intervals[i].Time_Hour = 0;
        Intervals[i].Time_Min = 0;
        Intervals[i].Time_Sec = 0;
    }
}

/**
  * @brief  Write Parameters to Non-Volatile Flash Memory
  * @note   Called strictly upon exiting the Setup Menu to prevent flash wear-out.
  * Max guaranteed erase cycles for STM32F1 is typically 10,000.
  */
void Save_Settings_To_Flash(void) {
    Flash_Data_t flash_data;
    flash_data.MagicWord = FLASH_MAGIC_WORD; // Stamp for validity check.
    flash_data.TotalIntervals = (uint32_t)Total_Intervals;
    memcpy(flash_data.Intervals, Intervals, sizeof(Intervals));

    // Pointer casting to allow word-by-word (32-bit) writing logic.
    uint32_t *data_ptr = (uint32_t *)&flash_data;
    uint16_t num_words = (sizeof(Flash_Data_t) + 3) / 4; // Ceiling division

    // CRITICAL SAFETY: Disable global interrupts.
    // If a Timer or EXTI fires while the Flash Controller is busy erasing/writing,
    // the CPU will fetch an instruction from a busy bus, causing a HardFault crash.
    __disable_irq();

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.PageAddress = FLASH_STORAGE_ADDR;
    EraseInitStruct.NbPages = 1;

    // Erase the page (Flash bits go from 0 to 1).
    // Writing can only flip 1s to 0s, so Erase is strictly required first.
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        for (uint16_t i = 0; i < num_words; i++) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + (i * 4), data_ptr[i]);
        }
    }

    HAL_FLASH_Lock();

    // Re-enable interrupts to resume normal background operations.
    __enable_irq();
}

/**
  * @brief  Read Parameters from Non-Volatile Flash Memory
  * @note   Called ONCE during boot sequence. Maps flash data straight to RAM.
  */
void Load_Settings_From_Flash(void) {
    // Treat the Flash address as a pointer to our struct type.
    Flash_Data_t *flash_data = (Flash_Data_t *)FLASH_STORAGE_ADDR;

    // If Magic Word is present, the memory has been explicitly written by our firmware before.
    if (flash_data->MagicWord == FLASH_MAGIC_WORD) {
        Total_Intervals = (uint8_t)flash_data->TotalIntervals;

        // Boundary constraint defense against potential bit-rot or bad pointers.
        if (Total_Intervals == 0 || Total_Intervals > MAX_INTERVALS) {
            Total_Intervals = 1;
        }

        // Deep copy from Flash into RAM execution variables.
        memcpy(Intervals, flash_data->Intervals, sizeof(Intervals));
    } else {
        // Factory default behavior.
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
    MAX31856_Init(&max31856, &hspi1, GPIOA, GPIO_PIN_15);
    MAX31856_SetThermocoupleType(&max31856, MAX31856_TCTYPE_K);
    MAX31856_SetNoiseFilter(&max31856, MAX31856_NOISE_FILTER_50HZ);
    MAX31856_SetConversionMode(&max31856, MAX31856_CONTINUOUS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // -------------------------------------------------------------------
      // 1. ASYNCHRONOUS POLLING: Temp Sensor Read
      // Uses the Systick (HAL_GetTick) to run every 500ms without blocking.
      // -------------------------------------------------------------------
      if (HAL_GetTick() - last_temp_read_time >= 500) {
          last_temp_read_time = HAL_GetTick();
          Current_Temp = MAX31856_ReadThermocoupleTemperature(&max31856);

          // Request a UI redraw only if we are actively viewing the temp.
          if (Current_UI_State == UI_STATE_MAIN) {
              LCD_Needs_Update = true;
          }
      }

      // ---------------------------------------------------------
      // 2. DISPATCHER: Input Handling
      // Executes logic triggered by the EXTI ISR flags.
      // ---------------------------------------------------------
      Process_Buttons();

      // ---------------------------------------------------------
      // 3. AUTO-STEP LOGIC: Cumulative Timeline
      // ---------------------------------------------------------
      if (System_Run_State == SYS_RUNNING) {
          // Convert elapsed time since start into absolute seconds.
          uint32_t current_run_seconds = Run_Hour * 3600 + Run_Min * 60 + Run_Sec;

          // If elapsed time has crossed the calculated threshold for the current interval...
          if (current_run_seconds >= Target_Run_Seconds) {
              if (Current_Interval < Total_Intervals) {
                  // Move to the next segment.
                  Current_Interval++;

                  // Calculate new threshold dynamically by adding the upcoming segment's duration.
                  // This maintains clock continuity.
                  Target_Run_Seconds += (Intervals[Current_Interval-1].Time_Hour * 3600 +
                                         Intervals[Current_Interval-1].Time_Min * 60 +
                                         Intervals[Current_Interval-1].Time_Sec);
                  LCD_Needs_Update = true;
              } else {
                  // No more segments left in the queue.
                  System_Run_State = SYS_COMPLETED;
                  LCD_Needs_Update = true;
              }
          }
      }

      // ----------------------------------------------------------------------------------
      // 4. DISPATCHER: UI Renderer
      // Repaints the LCD if and only if LCD_Needs_Update flag was raised by other modules.
      // ----------------------------------------------------------------------------------
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
