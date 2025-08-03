/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include "SEGGER_SYSVIEW.h"
#include"stdio.h"
#include "core_cm4.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SLAVE_ADDRESS_LCD 0x4E
#define LCD_EN_PIN  0x04 // Enable pin
#define LCD_RS_PIN  0x01 // Register Select pin
#define LCD_RW_PIN  0x02 // Read/Write pin (not used in this write-only example)
#define LCD_BL_PIN  0x08 // Backlight pin
#define LCD_DATA_SHIFT  4 // Shift data bits by 4 to align with PCF8574 pins P4-P7
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void LcdTask(void *params);

//---------------LCD-------------------------------
void DelayUs(uint32_t us);
void lcd_send_4_bits(uint8_t nibble, uint8_t rs);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);

// HD44780 commands
void HD44780_Init(uint8_t rows);
void HD44780_Clear();
void HD44780_SetCursor(uint8_t col, uint8_t row);
void HD44780_PrintStr(const char *str);

void DelayUs(uint32_t us);
void lcd_send_4_bits(uint8_t data, uint8_t rs);
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
//----------------------------------------------
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_SYSVIEW_Conf();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    HAL_TIM_Base_Start(&htim2); // Start the timer for DelayUs function

    // Create a task to handle the LCD display
    BaseType_t task_created_LCD = xTaskCreate(LcdTask, "LcdTask", 256, NULL, 2, NULL);
       configASSERT(task_created_LCD == pdPASS);

    // Start the scheduler
    vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hi2c1.Init.OwnAddress1 = 156;
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
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------
void DelayUs(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void lcd_send_4_bits(uint8_t nibble, uint8_t rs)
{
    uint8_t i2c_data;

    // Shift nibble to upper 4 bits and add control bits
    i2c_data = ((nibble & 0x0F) << 4) | rs | LCD_BL_PIN | LCD_EN_PIN;
    if (HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, &i2c_data, 1, 100) != HAL_OK)
    {
        // Optionally debug: toggle LED here if you want
        return;
    }

    DelayUs(50);

    // Drop Enable low to latch data
    i2c_data = ((nibble & 0x0F) << 4) | rs | LCD_BL_PIN;
    HAL_I2C_Master_Transmit(&hi2c1, SLAVE_ADDRESS_LCD, &i2c_data, 1, 100);

    DelayUs(50);
}

void lcd_send_cmd(char cmd)
{
    lcd_send_4_bits(cmd >> 4, 0);      // High nibble
    lcd_send_4_bits(cmd & 0x0F, 0);    // Low nibble
    DelayUs(40);
}

void lcd_send_data(char data)
{
    lcd_send_4_bits(data >> 4, LCD_RS_PIN);    // High nibble with RS=1
    lcd_send_4_bits(data & 0x0F, LCD_RS_PIN);  // Low nibble with RS=1
    DelayUs(40);
}

void HD44780_Init(uint8_t rows)
{
    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_send_4_bits(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    lcd_send_4_bits(0x03, 0);
    DelayUs(100);
    lcd_send_4_bits(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_4_bits(0x02, 0);

    lcd_send_cmd(0x28);  // Function set: 4-bit, 2 lines, 5x8 dots
    lcd_send_cmd(0x0C);  // Display ON, Cursor OFF, Blink OFF
    lcd_send_cmd(0x01);  // Clear display
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_send_cmd(0x06);  // Entry mode set (increment cursor)
    vTaskDelay(pdMS_TO_TICKS(1));
}

void HD44780_Clear()
{
    lcd_send_cmd(0x01);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void HD44780_SetCursor(uint8_t col, uint8_t row)
{
    uint8_t address = 0;
    switch(row)
    {
        case 0: address = 0x80 + col; break;
        case 1: address = 0xC0 + col; break;
        default: address = 0x80 + col; break;
    }
    lcd_send_cmd(address);
}

void HD44780_PrintStr(const char *str)
{
    while(*str)
    {
        lcd_send_data(*str++);
    }
}

void LcdTask(void* params)
{
    taskENTER_CRITICAL();
    vTaskDelay(pdMS_TO_TICKS(100));
    HD44780_Init(2);
    HD44780_Clear();
    taskEXIT_CRITICAL();

    while(1)
    {
        HD44780_SetCursor(0, 0);
        HD44780_PrintStr("kay kartay re");
        vTaskDelay(pdMS_TO_TICKS(500));

        HD44780_SetCursor(0, 1);
        HD44780_PrintStr("sagle");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//----------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
