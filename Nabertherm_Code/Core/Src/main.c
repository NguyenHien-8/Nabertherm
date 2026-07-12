/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Real-time Clock & Temp Monitor
  * @author         : TRAN NGUYEN HIEN
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
// Định nghĩa các trạng thái của hệ thống vận hành
typedef enum {
    SYS_IDLE,
    SYS_RUNNING,
    SYS_COMPLETED
} SystemState_t;

// Định nghĩa các trạng thái của UI / Menu
typedef enum {
    UI_STATE_MAIN,
    UI_STATE_SET_INTERVAL,
    UI_STATE_SET_P,
    UI_STATE_SET_TEMP,
    UI_STATE_SET_TIME
} UIState_t;

// Định nghĩa Mode của Interval
typedef enum {
    MODE_MT,    // Maintain Temp
    MODE_TIOT   // Temp Increases Over Time
} IntervalMode_t;

// Cấu trúc dữ liệu của 1 Interval
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
#define DEBOUNCE_DELAY 200
#define MAX_INTERVALS 9

// ====================================================
// Flash Storage Architecture (Page 63 của 64KB Flash)
// ====================================================
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

// ====================================================
// Cấu trúc gom dữ liệu để lưu vào Flash
// ====================================================
typedef struct {
    uint32_t MagicWord;
    uint32_t TotalIntervals;
    Interval_TypeDef Intervals[MAX_INTERVALS];
} Flash_Data_t;

//========== Multi-Pin Independent Debounce Variables ==========//
volatile uint32_t last_time_PA8 = 0;
volatile uint32_t last_time_PA9 = 0;
volatile uint32_t last_time_PA10 = 0;
volatile uint32_t last_time_PA11 = 0;
volatile uint32_t last_time_PB15 = 0;

//======= Button State Flags =======//
volatile uint8_t button_setting = 0;    // PA8 -> Setting/Run
volatile uint8_t button_redirect = 0;   // PA9 -> Redirect
volatile uint8_t button_select = 0;     // PB15 -> OK/Select
volatile uint8_t button_tang = 0;       // PA11 -> UP
volatile uint8_t button_giam = 0;       // PA10 -> DOWN

//======= System Logic & Data Variables =======//
Interval_TypeDef Intervals[MAX_INTERVALS];
uint8_t Total_Intervals = 1;

SystemState_t System_Run_State = SYS_IDLE;
UIState_t Current_UI_State = UI_STATE_MAIN;
uint8_t Current_Interval = 1;

uint8_t Run_Hour = 0, Run_Min = 0, Run_Sec = 0;
uint32_t Target_Run_Seconds = 0;
float Current_Temp = 0.0f;

//======= Menu Navigation Variables =======//
uint8_t Setting_P_Index = 1;        // Đang cài đặt cho P mấy (1 -> Total_Intervals)
uint8_t Menu_Cursor = 0;            // 0: Mode, 1: Set Temp, 2: Set Time
uint8_t Temp_Digit_Index = 0;       // 0: Nghìn, 1: Trăm, 2: Chục, 3: Đơn vị
uint8_t Time_Field_Index = 0;       // 0: Hour, 1: Min, 2: Sec

// Temporary Editing Variables
uint16_t Temp_Edit_Val = 0;
uint8_t Time_Edit_H = 0, Time_Edit_M = 0, Time_Edit_S = 0;

//======= Display Optimization Variables =======//
bool LCD_Needs_Update = true;
char prev_lcd_row1[17] = {0};
char prev_lcd_row2[17] = {0};
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

//=== NGẮT NÚT NHẤN TỐI ƯU (Độc lập thời gian Debounce cho từng Pin) ===//
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

//=== NGẮT TIMER (1Hz) CẬP NHẬT THỜI GIAN THỰC ===//
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
            if (Current_UI_State == UI_STATE_MAIN) {
                LCD_Needs_Update = true; // Báo cờ cập nhật lại LCD
            }
        }
    }
}

//=== HÀM QUẢN LÝ VÀ PHÂN LUỒNG XỬ LÝ NÚT NHẤN ===//
void Process_Buttons(void) {
    // 1. NÚT SETTING/RUN (PA8)
    if (button_setting) {
        button_setting = 0;
        if (Current_UI_State == UI_STATE_MAIN) {
            // Đang chạy -> Chuyển sang Cài đặt (Tạm dừng)
            Current_UI_State = UI_STATE_SET_INTERVAL;
            System_Run_State = SYS_IDLE;
        } else {
            // Đang ở Setting -> Lưu vào Flash và Thoát ra chạy lại từ đầu
            Save_Settings_To_Flash(); // <==== GHI XUỐNG FLASH KHI THOÁT KHỎI MENU CÀI ĐẶT

            Current_UI_State = UI_STATE_MAIN;
            Run_Hour = 0; Run_Min = 0; Run_Sec = 0;
            Current_Interval = 1;
            System_Run_State = SYS_RUNNING;

            // Tính toán Target Time cho tiến trình P1 (Ra Giây)
            Target_Run_Seconds = Intervals[0].Time_Hour * 3600 +
                                 Intervals[0].Time_Min * 60 +
                                 Intervals[0].Time_Sec;
        }
        LCD_Needs_Update = true;
    }

    // 2. NÚT REDIRECT (PA9)
    if (button_redirect) {
        button_redirect = 0;
        if (Current_UI_State == UI_STATE_SET_P) {
            Setting_P_Index++;
            if (Setting_P_Index > Total_Intervals) Setting_P_Index = 1;
        } else if (Current_UI_State == UI_STATE_SET_TEMP) {
            Temp_Digit_Index = (Temp_Digit_Index + 1) % 4;
        } else if (Current_UI_State == UI_STATE_SET_TIME) {
            Time_Field_Index = (Time_Field_Index + 1) % 3;
        }
        LCD_Needs_Update = true;
    }

    // 3. NÚT SELECT/OK (PB15)
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
                    // Đảo Mode
                    Intervals[Setting_P_Index-1].Mode = (Intervals[Setting_P_Index-1].Mode == MODE_MT) ? MODE_TIOT : MODE_MT;
                } else if (Menu_Cursor == 1) {
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
                Intervals[Setting_P_Index-1].Temp = Temp_Edit_Val;
                Current_UI_State = UI_STATE_SET_P;
                break;
            case UI_STATE_SET_TIME:
                Intervals[Setting_P_Index-1].Time_Hour = Time_Edit_H;
                Intervals[Setting_P_Index-1].Time_Min = Time_Edit_M;
                Intervals[Setting_P_Index-1].Time_Sec = Time_Edit_S;
                Current_UI_State = UI_STATE_SET_P;
                break;
            default: break;
        }
        LCD_Needs_Update = true;
    }

    // 4. NÚT TĂNG (PA11) & GIẢM (PA10)
    if (button_tang || button_giam) {
        bool is_up = button_tang;
        button_tang = 0; button_giam = 0;

        switch (Current_UI_State) {
            case UI_STATE_SET_INTERVAL:
                if (is_up && Total_Intervals < MAX_INTERVALS) Total_Intervals++;
                else if (!is_up && Total_Intervals > 1) Total_Intervals--;
                break;

            case UI_STATE_SET_P:
                if (is_up) Menu_Cursor = (Menu_Cursor > 0) ? Menu_Cursor - 1 : 2;
                else Menu_Cursor = (Menu_Cursor < 2) ? Menu_Cursor + 1 : 0;
                break;

            case UI_STATE_SET_TEMP: {
                // Tách 4 chữ số để xoay vòng
                uint8_t d[4];
                d[0] = Temp_Edit_Val / 1000;
                d[1] = (Temp_Edit_Val / 100) % 10;
                d[2] = (Temp_Edit_Val / 10) % 10;
                d[3] = Temp_Edit_Val % 10;

                if (is_up) d[Temp_Digit_Index] = (d[Temp_Digit_Index] == 9) ? 0 : d[Temp_Digit_Index] + 1;
                else d[Temp_Digit_Index] = (d[Temp_Digit_Index] == 0) ? 9 : d[Temp_Digit_Index] - 1;

                Temp_Edit_Val = d[0]*1000 + d[1]*100 + d[2]*10 + d[3];
                break;
            }

            case UI_STATE_SET_TIME: {
                if (Time_Field_Index == 0) { // Giờ
                    if (is_up) Time_Edit_H = (Time_Edit_H == 99) ? 0 : Time_Edit_H + 1;
                    else Time_Edit_H = (Time_Edit_H == 0) ? 99 : Time_Edit_H - 1;
                } else if (Time_Field_Index == 1) { // Phút
                    if (is_up) Time_Edit_M = (Time_Edit_M == 59) ? 0 : Time_Edit_M + 1;
                    else Time_Edit_M = (Time_Edit_M == 0) ? 59 : Time_Edit_M - 1;
                } else if (Time_Field_Index == 2) { // Giây
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

//=== HÀM CẬP NHẬT LCD CHỐNG NHÁY MÀN HÌNH ===//
void Update_LCD(void) {
    if (!LCD_Needs_Update) return;
    LCD_Needs_Update = false;

    char row1[17];
    char row2[17];
    char temp_buf[21];

    // Xóa bộ đệm mảng (Mặc định lấp khoảng trắng để xóa dư ảnh)
    memset(row1, ' ', 16); row1[16] = '\0';
    memset(row2, ' ', 16); row2[16] = '\0';

    switch (Current_UI_State) {
        case UI_STATE_MAIN:
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

    // Chỉ update I2C những dòng có sự thay đổi
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

    // Quản lý vị trí con trỏ nhấp nháy bằng LCD Hardware function
    if (Current_UI_State == UI_STATE_SET_TEMP) {
        LCDI2C_setCursor(7 + Temp_Digit_Index, 1);
        LCDI2C_blink_on();
    } else if (Current_UI_State == UI_STATE_SET_TIME) {
        uint8_t col = (Time_Field_Index == 0) ? 7 : ((Time_Field_Index == 1) ? 10 : 13);
        LCDI2C_setCursor(col, 1);
        LCDI2C_blink_on();
    } else {
        LCDI2C_blink_off();
    }
}

//=== KHỞI TẠO DỮ LIỆU MẶC ĐỊNH ===//
void Init_Default_Intervals(void) {
    for (int i = 0; i < MAX_INTERVALS; i++) {
        Intervals[i].Mode = MODE_MT;
        Intervals[i].Temp = 0;
        Intervals[i].Time_Hour = 0;
        Intervals[i].Time_Min = 0;
        Intervals[i].Time_Sec = 0;
    }
}

//========================================================
// HÀM LƯU DỮ LIỆU VÀO FLASH (NON-VOLATILE MEMORY)
//========================================================
void Save_Settings_To_Flash(void) {
    Flash_Data_t flash_data;
    flash_data.MagicWord = FLASH_MAGIC_WORD;
    flash_data.TotalIntervals = (uint32_t)Total_Intervals;
    memcpy(flash_data.Intervals, Intervals, sizeof(Intervals));

    // Ép kiểu struct sang con trỏ uint32_t để ghi theo từng Word
    uint32_t *data_ptr = (uint32_t *)&flash_data;
    uint16_t num_words = (sizeof(Flash_Data_t) + 3) / 4;

    __disable_irq(); // Vô hiệu hóa ngắt (An toàn Flash)

    HAL_FLASH_Unlock();

    // Khởi tạo struct phục vụ việc Erase Page
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.Banks = FLASH_BANK_1;
    EraseInitStruct.PageAddress = FLASH_STORAGE_ADDR;
    EraseInitStruct.NbPages = 1;

    // Tiến hành Erase Page 63
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        // Lặp để ghi từng Word
        for (uint16_t i = 0; i < num_words; i++) {
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDR + (i * 4), data_ptr[i]);
        }
    }

    HAL_FLASH_Lock();

    __enable_irq(); // Bật lại ngắt
}

//========================================================
// HÀM TẢI DỮ LIỆU TỪ FLASH (NON-VOLATILE MEMORY)
//========================================================
void Load_Settings_From_Flash(void) {
    // Trỏ thẳng cấu trúc vào địa chỉ Flash để đọc
    Flash_Data_t *flash_data = (Flash_Data_t *)FLASH_STORAGE_ADDR;

    // Kiểm tra Magic Word để phân biệt Flash đã ghi hay Flash rỗng/Lỗi
    if (flash_data->MagicWord == FLASH_MAGIC_WORD) {
        Total_Intervals = (uint8_t)flash_data->TotalIntervals;

        // Ràng buộc giới hạn đề phòng rủi ro dữ liệu sai sót
        if (Total_Intervals == 0 || Total_Intervals > MAX_INTERVALS) {
            Total_Intervals = 1;
        }

        // Copy mảng Settings
        memcpy(Intervals, flash_data->Intervals, sizeof(Intervals));
    } else {
        // Không khớp -> Nạp giá trị mặc định cho MCU mới tinh
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

  //==== TẢI DỮ LIỆU TỪ FLASH VÀO RAM KHI KHỞI ĐỘNG ====//
  // Nếu có dữ liệu hợp lệ, tự động nạp. Nếu rỗng, nạp mặc định.
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
	  // 1. NON-BLOCKING: Đọc nhiệt độ từ cảm biến mỗi 500ms
      if (HAL_GetTick() - last_temp_read_time >= 500) {
          last_temp_read_time = HAL_GetTick();
          Current_Temp = MAX31856_ReadThermocoupleTemperature(&max31856);
          if (Current_UI_State == UI_STATE_MAIN) {
              LCD_Needs_Update = true; // Báo cập nhật nhiệt độ
          }
      }

      // 2. Xử lý Nút nhấn (Flags nhận được từ EXTI Callback)
      Process_Buttons();

      // 3. LOGIC AUTO-STEP (Cộng dồn mốc thời gian không reset)
      if (System_Run_State == SYS_RUNNING) {
          uint32_t current_run_seconds = Run_Hour * 3600 + Run_Min * 60 + Run_Sec;

          if (current_run_seconds >= Target_Run_Seconds) {
              if (Current_Interval < Total_Intervals) {
                  // Chuyển sang tiến trình P tiếp theo
                  Current_Interval++;
                  // Cộng dồn mốc đích (Target) với thời gian của tiến trình mới
                  Target_Run_Seconds += (Intervals[Current_Interval-1].Time_Hour * 3600 +
                                         Intervals[Current_Interval-1].Time_Min * 60 +
                                         Intervals[Current_Interval-1].Time_Sec);
                  LCD_Needs_Update = true;
              } else {
                  // Đã hoàn tất tất cả tiến trình
                  System_Run_State = SYS_COMPLETED;
                  LCD_Needs_Update = true;
              }
          }
      }

      // 4. Update LCD dựa theo Flag
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
