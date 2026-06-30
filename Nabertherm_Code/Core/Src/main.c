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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    NORMAL_MODE,
    SET_TEMP_MODE,
    SET_TIME_MODE
} SystemState;

typedef enum {
    SEL_HOUR,
    SEL_MIN,
    SEL_SEC
} TimeSelect;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY 200
// Chọn Page 63 (Page cuối của STM32F103C8T6 64KB) để lưu dữ liệu
#define FLASH_USER_START_ADDR   0x0800FC00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
//========== Biến Chống Dội ==========//
volatile uint32_t last_exti_time = 0;

//======= Trạng Thái Nút Nhấn =======//
volatile uint8_t button_setting = 0;    // PB0 -> Vào Setting HOẶC Chọn con trỏ hh/mm/ss
volatile uint8_t button_redirect = 0;   // PB1 -> Đổi Sub-mode
volatile uint8_t button_startstop = 0;  // PA7 -> Lưu và Thoát về Normal Mode
volatile uint8_t button_tang = 0;       // PB10 -> UP
volatile uint8_t button_giam = 0;       // PB11 -> DOWN

//======= Biến Quản Lý Hệ Thống =======//
volatile SystemState current_state = NORMAL_MODE;
volatile TimeSelect current_select = SEL_HOUR;

// 1. Bộ biến hiển thị và đếm thực tế (Bắt đầu từ 00:00:00)
volatile uint8_t run_hours = 0;
volatile uint8_t run_minutes = 0;
volatile uint8_t run_seconds = 0;

// 2. Bộ biến cài đặt mục tiêu
volatile uint8_t target_hours = 0;
volatile uint8_t target_minutes = 0;
volatile uint8_t target_seconds = 0;

// 3. Cờ trạng thái chạy (1 = Đang đếm tiến, 0 = Đã dừng)
volatile uint8_t is_running = 0;

volatile int16_t temp_threshold = 30; // Ngưỡng nhiệt độ mặc định
int16_t current_temp = 28;             // Giả lập nhiệt độ môi trường hiện tại
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void Handle_Buttons(void);
void Update_LCD(void);

// Khai báo thêm 2 hàm xử lý Flash
void Save_Settings_To_Flash(void);
void Load_Settings_From_Flash(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Save_Settings_To_Flash(void) {
    HAL_FLASH_Unlock(); // Mở khóa Flash để cho phép ghi

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;

    // Cấu hình xóa Page 63
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
    EraseInitStruct.NbPages = 1;

    // Thực hiện xóa và kiểm tra
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        // Xóa thành công thì tiến hành ghi từng giá trị (ép kiểu về uint16_t)
        // Mỗi lần ghi cách nhau 2 byte (HalfWord)
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR,     (uint16_t)temp_threshold);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR + 2, (uint16_t)target_hours);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR + 4, (uint16_t)target_minutes);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_USER_START_ADDR + 6, (uint16_t)target_seconds);
    }

    HAL_FLASH_Lock(); // Khóa Flash lại để bảo vệ
}

void Load_Settings_From_Flash(void) {
    // Đọc giá trị đầu tiên ra để kiểm tra
    uint16_t temp_check = *(__IO uint16_t*)FLASH_USER_START_ADDR;

    // 0xFFFF là trạng thái mặc định của bộ nhớ Flash trống (chưa ghi bao giờ)
    if (temp_check != 0xFFFF) {
        temp_threshold = temp_check;
        target_hours   = *(__IO uint16_t*)(FLASH_USER_START_ADDR + 2);
        target_minutes = *(__IO uint16_t*)(FLASH_USER_START_ADDR + 4);
        target_seconds = *(__IO uint16_t*)(FLASH_USER_START_ADDR + 6);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  uint32_t current_time = HAL_GetTick();

  if ((current_time - last_exti_time) > DEBOUNCE_DELAY) {
      if (GPIO_Pin == GPIO_PIN_0) {
          button_setting = 1;
      } else if (GPIO_Pin == GPIO_PIN_1) {
          button_redirect = 1;
      } else if (GPIO_Pin == GPIO_PIN_7) {
          button_startstop = 1;
      } else if (GPIO_Pin == GPIO_PIN_10) {
          button_tang = 1;
      } else if (GPIO_Pin == GPIO_PIN_11) {
          button_giam = 1;
      }
      last_exti_time = current_time;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
      // Chỉ đếm tiến khi ở chế độ Normal và cờ is_running được bật
      if (current_state == NORMAL_MODE && is_running == 1) {
          run_seconds++;
          if (run_seconds >= 60) {
              run_seconds = 0;
              run_minutes++;
              if (run_minutes >= 60) {
                  run_minutes = 0;
                  run_hours++;
                  if (run_hours >= 24) {
                      run_hours = 0;
                  }
              }
          }

          // Kiểm tra nếu thời gian chạy đã chạm mốc thời gian mục tiêu cài đặt
          if (run_hours == target_hours && run_minutes == target_minutes && run_seconds == target_seconds) {
              is_running = 0; // Đạt mục tiêu -> Dừng đếm

              // THÊM: Tắt Relay SSR khi hết thời gian
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          }
      }
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LCDI2C_init(&hi2c1, 0x27, 16, 2);
  // Gọi hàm Load dữ liệu từ Flash ngay khi vừa khởi động
  Load_Settings_From_Flash();
  HAL_TIM_Base_Start_IT(&htim2); // Khởi động ngắt Timer 2 (1Hz)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_lcd_tick = 0; // Thêm biến static/global để theo dõi thời gian cập nhật LCD
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Handle_Buttons();

    // Cập nhật hiển thị lên LCD
    if (HAL_GetTick() - last_lcd_tick >= 60) {
        last_lcd_tick = HAL_GetTick();
        Update_LCD();
    }
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#define HOLD_DELAY_MS 400
#define FAST_SPEED_MS 20

static uint32_t last_blink_tick = 0;
static uint8_t blink_toggle = 1;

void Handle_Buttons(void) {
  // 1. Nút PA7: Lưu & Thoát (nếu ở cài đặt) HOẶC Tạm dừng / Chạy tiếp (nếu ở Normal Mode)
  if (button_startstop) {
      button_startstop = 0;

      if (current_state != NORMAL_MODE) {
          // --- CODE CŨ: Xử lý khi thoát từ các chế độ SETTING về NORMAL_MODE ---
          LCDI2C_clear();
          current_state = NORMAL_MODE;

          // Lưu các giá trị vừa cấu hình vào Flash
          Save_Settings_To_Flash();

          // Khi thoát ra Normal Mode, reset thời gian chạy về 00:00:00 để bắt đầu chu kỳ mới
          run_hours = 0;
          run_minutes = 0;
          run_seconds = 0;

          // Nếu thời gian mục tiêu > 0 thì bắt đầu cho đếm
          if (target_hours > 0 || target_minutes > 0 || target_seconds > 0) {
              is_running = 1;
              // Bật Relay SSR để bắt đầu hoạt động
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
          } else {
              is_running = 0; // Nếu cài là 00:00:00 thì không chạy
              // Chắc chắn Relay SSR bị tắt nếu thời gian là 0
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          }
      }
      else {
          // --- ĐOẠN THÊM MỚI: Xử lý Tạm dừng / Chạy tiếp khi ĐANG Ở NORMAL_MODE ---
          if (is_running == 1) {
              // Nhấn lần 1: Nếu đang chạy -> Tạm dừng thời gian
              is_running = 0;

              // Ngắt relay SSR ngay lập tức
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
          }
          else {
              // Nhấn lần 2: Nếu đang dừng -> Tiếp tục chạy từ mốc thời gian cũ
              // Điều kiện: Thời gian mục tiêu phải hợp lệ (>0) và chưa bị timeout trước đó
              if ((target_hours > 0 || target_minutes > 0 || target_seconds > 0) &&
                  !(run_hours == target_hours && run_minutes == target_minutes && run_seconds == target_seconds)) {

                  is_running = 1;

                  // Đóng (bật) lại relay SSR để tiếp tục quá trình
                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
              }
          }
      }
      return;
  }

  // 2. Nút PB0: Vào Setting Mode HOẶC di chuyển con trỏ chọn phân đoạn
  if (button_setting) {
      button_setting = 0;
      if (current_state == NORMAL_MODE) {
          LCDI2C_clear();
          current_state = SET_TEMP_MODE;
      }
      else if (current_state == SET_TIME_MODE) {
          if (current_select == SEL_HOUR) current_select = SEL_MIN;
          else if (current_select == SEL_MIN) current_select = SEL_SEC;
          else current_select = SEL_HOUR;
      }
      return;
  }

  // Nếu đang ở Normal Mode, vô hiệu hóa các nút còn lại (PB1, PB10, PB11)
  if (current_state == NORMAL_MODE) {
      button_redirect = 0;
      button_tang = 0;
      button_giam = 0;
      return;
  }

  // 3. Nút PB1: Chuyển đổi Menu con (SET_TEMP_MODE <-> SET_TIME_MODE)
  if (button_redirect) {
      button_redirect = 0;
      LCDI2C_clear();
      if (current_state == SET_TEMP_MODE) {
          current_state = SET_TIME_MODE;
          current_select = SEL_HOUR;
      } else if (current_state == SET_TIME_MODE) {
          current_state = SET_TEMP_MODE;
      }
      return;
  }

  // =========================================================================
  // 4. THUẬT TOÁN NHẤN GIỮ (FAST FORWARD / REWIND)
  // =========================================================================
  static uint32_t hold_tick_up = 0;
  static uint32_t fast_tick_up = 0;
  static uint32_t hold_tick_down = 0;
  static uint32_t fast_tick_down = 0;

  uint8_t do_up = 0;
  uint8_t do_down = 0;
  uint16_t step_size = 1;

  // ---- Xử lý Nút Tăng (UP - PB10) ----
  if (button_tang) {
      button_tang = 0;
      do_up = 1;
      hold_tick_up = HAL_GetTick();
      fast_tick_up = HAL_GetTick();
  }
  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) {
      uint32_t press_duration = HAL_GetTick() - hold_tick_up;
      if (press_duration > HOLD_DELAY_MS) {
          if ((HAL_GetTick() - fast_tick_up) > FAST_SPEED_MS) {
              fast_tick_up = HAL_GetTick();
              do_up = 1;
              if (current_state == SET_TEMP_MODE) {
                  if (press_duration > 2500) step_size = 10;
                  else if (press_duration > 1200) step_size = 5;
              }
          }
      }
  }

  // ---- Xử lý Nút Giảm (DOWN - PB11) ----
  if (button_giam) {
      button_giam = 0;
      do_down = 1;
      hold_tick_down = HAL_GetTick();
      fast_tick_down = HAL_GetTick();
  }
  else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) {
      uint32_t press_duration = HAL_GetTick() - hold_tick_down;
      if (press_duration > HOLD_DELAY_MS) {
          if ((HAL_GetTick() - fast_tick_down) > FAST_SPEED_MS) {
              fast_tick_down = HAL_GetTick();
              do_down = 1;
              if (current_state == SET_TEMP_MODE) {
                  if (press_duration > 2500) step_size = 10;
                  else if (press_duration > 1200) step_size = 5;
              }
          }
      }
  }

  // =========================================================================
  // 5. THỰC THI THAY ĐỔI GIÁ TRỊ CÀI ĐẶT
  // =========================================================================
  if (do_up || do_down) {
      blink_toggle = 1;
      last_blink_tick = HAL_GetTick();

      if (current_state == SET_TEMP_MODE) {
          if (do_up) {
              temp_threshold += step_size;
              if (temp_threshold > 1000) temp_threshold = 1000;
          }
          if (do_down) {
              temp_threshold -= step_size;
              if (temp_threshold < 0) temp_threshold = 0;
          }
      }
      else if (current_state == SET_TIME_MODE) {
          // Lưu ý: Bây giờ ta thay đổi biến target_ thay vì biến đang chạy
          if (do_up) {
              if (current_select == SEL_HOUR) target_hours = (target_hours + 1) % 24;
              else if (current_select == SEL_MIN) target_minutes = (target_minutes + 1) % 60;
              else if (current_select == SEL_SEC) target_seconds = (target_seconds + 1) % 60;
          }
          if (do_down) {
              if (current_select == SEL_HOUR) target_hours = (target_hours == 0) ? 23 : target_hours - 1;
              else if (current_select == SEL_MIN) target_minutes = (target_minutes == 0) ? 59 : target_minutes - 1;
              else if (current_select == SEL_SEC) target_seconds = (target_seconds == 0) ? 59 : target_seconds - 1;
          }
      }
  }
}

void Update_LCD(void) {
  static uint8_t last_second = 255;
  char lcd_buf[24];

  if (HAL_GetTick() - last_blink_tick >= 300) {
      last_blink_tick = HAL_GetTick();
      blink_toggle = !blink_toggle;
  }

  switch (current_state) {
      case NORMAL_MODE:
          // In Normal Mode, hiển thị thời gian đang chạy thực tế (run_*)
          if (run_seconds != last_second) {
              last_second = run_seconds;

              snprintf(lcd_buf, sizeof(lcd_buf), "Temp: %2d\xDF" "C         ", current_temp);
              LCDI2C_setCursor(0, 0);
              LCDI2C_write_String(lcd_buf);

              snprintf(lcd_buf, sizeof(lcd_buf), "Time: %02d:%02d:%02d ", run_hours, run_minutes, run_seconds);
              LCDI2C_setCursor(0, 1);
              LCDI2C_write_String(lcd_buf);
          }
          break;

      case SET_TEMP_MODE:
          last_second = 255;

          LCDI2C_setCursor(0, 0);
          LCDI2C_write_String("[SET TEMP]      ");

          LCDI2C_setCursor(0, 1);
          if (blink_toggle) {
              snprintf(lcd_buf, sizeof(lcd_buf), "Thres: %3d\xDF" "C ", temp_threshold);
              LCDI2C_write_String(lcd_buf);
          } else {
              LCDI2C_write_String("Thres:    \xDF" "C ");
          }
          break;

      case SET_TIME_MODE:
          last_second = 255;

          LCDI2C_setCursor(0, 0);
          LCDI2C_write_String("[SET TIME]      ");

          char h_str[5], m_str[5], s_str[5];

          // In Set Time Mode, hiển thị biến mục tiêu cài đặt (target_*)
          if (current_select == SEL_HOUR && !blink_toggle) snprintf(h_str, sizeof(h_str), "  ");
          else snprintf(h_str, sizeof(h_str), "%02d", target_hours);

          if (current_select == SEL_MIN && !blink_toggle) snprintf(m_str, sizeof(m_str), "  ");
          else snprintf(m_str, sizeof(m_str), "%02d", target_minutes);

          if (current_select == SEL_SEC && !blink_toggle) snprintf(s_str, sizeof(s_str), "  ");
          else snprintf(s_str, sizeof(s_str), "%02d", target_seconds);

          snprintf(lcd_buf, sizeof(lcd_buf), "Time: %s:%s:%s  ", h_str, m_str, s_str);
          LCDI2C_setCursor(0, 1);
          LCDI2C_write_String(lcd_buf);
          break;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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
