/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
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
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
// Khai báo các object điều khiển
MAX31856_HandleTypeDef max31856;
PID_TypeDef pid_heater;

// Biến cho PID
float Input, Output, Setpoint;
float Kp = 20.0f, Ki = 0.02f, Kd = 300.0f;

// Biến cho Time-Proportional Control (Slow PWM)
#define WINDOW_SIZE 1000 // Chu kỳ băm xung là 1000ms (1 giây)
uint32_t windowStartTime = 0;
float active_output = 0; // Biến "chốt" (lock) Output cho 1 chu kỳ PWM

// Các biến định thời phi nghẽn
uint32_t lastSensorUpdate = 0;
uint32_t lastLcdUpdate = 0;
uint8_t sensor_fault = 0; // Biến lưu trạng thái lỗi cảm biến
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // 1. Init LCD 16x2
    LCDI2C_init(&hi2c2, 0x27, 16, 2);
    LCDI2C_backlight();
    LCDI2C_clear();

  // 2. Khởi tạo cảm biến MAX31856
    MAX31856_Init(&max31856, &hspi1, GPIOA, GPIO_PIN_15);
    MAX31856_SetThermocoupleType(&max31856, MAX31856_TCTYPE_S);
    MAX31856_SetNoiseFilter(&max31856, MAX31856_NOISE_FILTER_50HZ);
    MAX31856_SetConversionMode(&max31856, MAX31856_CONTINUOUS);

  // 3. Khởi tạo bộ PID
    Setpoint = 150.0f;
    PID_Init(&pid_heater, &Input, &Output, &Setpoint, Kp, Ki, Kd, PID_P_ON_E, PID_DIRECT);
    PID_SetMode(&pid_heater, PID_AUTOMATIC);
    PID_SetOutputLimits(&pid_heater, 0, WINDOW_SIZE);

    // Đồng bộ Chu kỳ tính toán PID đúng bằng chu kỳ băm xung SSR
    PID_SetSampleTime(&pid_heater, WINDOW_SIZE);
    windowStartTime = HAL_GetTick();

    // Biến dùng cho bộ lọc EMA (Exponential Moving Average)
     static float filtered_temp = -1.0f;
    // Trạng thái Anti-Windup để tránh gọi hàm SetTunings liên tục
     static bool ki_is_active = true;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t currentTick = HAL_GetTick();
      /* ==================================================================== */
      /* CHU KỲ 1: ĐỌC VÀ LỌC NHIỄU CẢM BIẾN (Chạy định kỳ 250ms)             */
      /* ==================================================================== */
      if (currentTick - lastSensorUpdate >= 250)
      {
          lastSensorUpdate = currentTick;

          // Kiểm tra lỗi cảm biến
          sensor_fault = MAX31856_ReadFault(&max31856);

          if (sensor_fault == 0)
          {
              float temp_read = MAX31856_ReadThermocoupleTemperature(&max31856);
              if (temp_read > -0.1f && temp_read < 500.0f) { // Lọc giá trị rác cực đoan
                  if (filtered_temp < 0.0f) filtered_temp = temp_read;
                  else filtered_temp = 0.8f * filtered_temp + 0.2f * temp_read; // Bộ lọc EMA
                  Input = filtered_temp;
              }
          } else {
              // Bảo vệ phần cứng: Tắt gia nhiệt ngay lập tức nếu cảm biến lỗi
              PID_SetMode(&pid_heater, PID_MANUAL);
              active_output = 0;
              Output = 0;
          }
      }

      /* ==================================================================== */
      /* CHU KỲ 2: TÍNH TOÁN PID VÀ CẬP NHẬT CHU KỲ PWM                       */
      /* ==================================================================== */
      if (sensor_fault == 0) {
          if (PID_GetMode(&pid_heater) != PID_AUTOMATIC) {
              PID_SetMode(&pid_heater, PID_AUTOMATIC);
          }

          // Anti-Windup: Chỉ bật khâu I (Tích phân) khi cách đích <= 5 độ C
          float error = Setpoint - Input;
          if (fabs(error) > 5.0f && ki_is_active) {
              PID_SetTunings(&pid_heater, Kp, 0.0f, Kd, PID_P_ON_E);
              ki_is_active = false;
          } else if (fabs(error) <= 5.0f && !ki_is_active) {
              PID_SetTunings(&pid_heater, Kp, Ki, Kd, PID_P_ON_E);
              ki_is_active = true;
          }

          // Cho phép thư viện PID tự quyết định thời điểm cập nhật dựa trên SampleTime (1000ms)
          if (PID_Compute(&pid_heater)) {
              // Hàm PID_Compute trả về true -> Có giá trị Output mới
              // ĐỒNG BỘ HOÁ: Reset lại bộ đếm PWM window ngay lúc này
              windowStartTime = currentTick;
              active_output = Output;
          }
      }

      /* ==================================================================== */
      /* CHU KỲ 3: ĐIỀU KHIỂN SSR PWM (Chạy liên tục không block)             */
      /* ==================================================================== */
      if (sensor_fault != 0) {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
      } else {
          uint32_t ms_in_window = currentTick - windowStartTime;

          // Xử lý Deadband an toàn cho SSR (Cắt bỏ dải < 20ms và giữ ON toàn thời gian nếu > 980ms)
          if (active_output <= 20) {
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
          } else if (active_output >= (WINDOW_SIZE - 20)) {
              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
          } else {
              // Băm xung bình thường
              if (ms_in_window < active_output) {
                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
              } else {
                  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
              }
          }
      }

      /* ==================================================================== */
      /* CHU KỲ 4: CẬP NHẬT LCD (Chạy định kỳ mỗi 500ms)                      */
      /* ==================================================================== */
      if (currentTick - lastLcdUpdate >= 500)
      {
          lastLcdUpdate = currentTick;
          char lcd_buf[18];

          if (sensor_fault != 0) {
              LCDI2C_setCursor(0, 0);
              LCDI2C_write_String("SENSOR ERROR!   ");
              LCDI2C_setCursor(0, 1);
              LCDI2C_write_String("SSR: OFF        ");
          } else {
              LCDI2C_setCursor(0, 0);
              snprintf(lcd_buf, sizeof(lcd_buf), "SP:%3.0f PV:%3.2f", Setpoint, Input);
              LCDI2C_write_String(lcd_buf);

              LCDI2C_setCursor(0, 1);
              // Ký hiệu [I] hiển thị để theo dõi xem bộ Anti-Windup đang bật hay tắt Tích phân
              snprintf(lcd_buf, sizeof(lcd_buf), "SSR: %4dms %c   ", (int)active_output, ki_is_active ? 'I' : ' ');
              LCDI2C_write_String(lcd_buf);
          }
      }
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
