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
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    uint8_t humidity_int;
    uint8_t humidity_dec;
    uint8_t temperature_int;
    uint8_t temperature_dec;
} dht11_data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DHT11_GPIO_Port GPIOA
#define DHT11_Pin GPIO_PIN_2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint8_t data[5] = {0};
float humidity_val, temperature_val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static inline uint32_t timer_delta(uint32_t start, uint32_t end);
void delay_us(uint16_t us);
uint8_t wait_for_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state, uint32_t timeout_us);
void DHT11_SetPinAsOutput(void);
void DHT11_SetPinAsInput(void);
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
void Read_DHT11_Data(void);
void DHT11Task(void *argument); // Your FreeRTOS task
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline uint32_t 	timer_delta(uint32_t start, uint32_t end)
{
    // Handles 16-bit timer overflow (0 to 65535)
    return (end >= start) ? (end - start) : (65536 + end - start);
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset counter
    while (__HAL_TIM_GET_COUNTER(&htim2) < us); // Wait until counter reaches 'us'
}

// Waits for pin == state, returns 1 if ok, 0 on timeout (overflow-safe)
uint8_t wait_for_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x, uint8_t state, uint32_t timeout_us)
{
    uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin_x) != state)
    {
        uint32_t now = __HAL_TIM_GET_COUNTER(&htim2);
        if (timer_delta(start, now) > timeout_us)
        {
            return 0; // Timeout
        }
    }
    return 1; // State achieved
}

// Helper function to configure DHT11 pin as output
void DHT11_SetPinAsOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // Open-drain for DHT11
    GPIO_InitStruct.Pull = GPIO_PULLUP;         // Internal pull-up (or rely on external)
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High speed for quick transitions
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

// Helper function to configure DHT11 pin as input
void DHT11_SetPinAsInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP; // Keep pull-up enabled for input mode
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}


void DHT11_Start(void)
{
	printf("Starting DHT11...\r\n");
    // Set pin as output, open-drain, with pull-up
    DHT11_SetPinAsOutput();

    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET); // MCU pulls low
    HAL_Delay(18); // CRITICAL: 18 ms, min >18 ms required for DHT11 start pulse. Use HAL_Delay for milliseconds.

    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET); // MCU releases line (goes high due to pull-up)
    delay_us(30); // Host waits 20-40us before DHT11 responds

    // Switch to input with pull-up to listen for DHT11 response
    DHT11_SetPinAsInput();
}

// Wait for DHT11 response after start
uint8_t DHT11_CheckResponse(void)
{
    // DHT11 pulls low for ~80us (response low)
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET, 90)) { // Timeout slightly above 80us
        printf("DHT11 CheckResponse: No response (DHT did not pull LOW)\r\n");
        SEGGER_SYSVIEW_PrintfHost("DHT11 CheckResponse: No response (DHT did not pull LOW)\r\n");
        return 0;
    }

    // DHT11 pulls high for ~80us (response high)
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET, 90)) {   // Timeout slightly above 80us
        printf("DHT11 CheckResponse: No response (DHT did not pull HIGH)\r\n");
        SEGGER_SYSVIEW_PrintfHost("DHT11 CheckResponse: No response (DHT did not pull HIGH)\r\n");
        return 0;
    }
    return 1;
}

uint8_t DHT11_ReadBit(void)
{
    // Each bit starts with a ~50us LOW pulse from DHT11
    // Wait for the end of this 50us LOW pulse (i.e., pin goes HIGH)
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET, 70)) { // Timeout for 50us LOW + margin
        printf("DHT11 ReadBit: Timeout waiting for HIGH start\r\n");
        SEGGER_SYSVIEW_PrintfHost("DHT11 ReadBit: Timeout waiting for HIGH start\r\n");
        return 0xFF; // Indicate error
    }
    uint32_t tStart = __HAL_TIM_GET_COUNTER(&htim2);

    // After the LOW pulse, the DHT11 sends a HIGH pulse:
    // ~26-28us for '0'
    // ~70us for '1'
    // Wait for the end of the HIGH pulse (i.e., pin goes LOW again)
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET, 90)) { // Timeout for 70us HIGH + margin
        printf("DHT11 ReadBit: Timeout waiting for LOW end\r\n");
        SEGGER_SYSVIEW_PrintfHost("DHT11 ReadBit: Timeout waiting for LOW end\r\n");
        return 0xFF; // Indicate error
    }
    uint32_t tEnd = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t tDuration = timer_delta(tStart, tEnd);

    // Threshold for '0' or '1'
    // A '0' is 26-28us, a '1' is 70us. A good threshold is typically ~40-50us.
    return (tDuration > 45) ? 1 : 0; // Use a distinct threshold like 45us
}

uint8_t DHT11_ReadByte(void)
{
    uint8_t i;
    uint8_t val = 0;
    for (i = 0; i < 8; i++)
    {
        val <<= 1;
        uint8_t bit = DHT11_ReadBit();
        if (bit == 0xFF) return 0xFF; // Propagate error from DHT11_ReadBit
        val |= bit;
    }
    return val;
}

void Read_DHT11_Data(void)
{
    // Clear previous data
    for (int i = 0; i < 5; i++) data[i] = 0;

    DHT11_Start();
    if (DHT11_CheckResponse())
    {
        for (int i = 0; i < 5; i++)
        {
            data[i] = DHT11_ReadByte();
            if (data[i] == 0xFF)
            { // Check for read error (0xFF from DHT11_ReadByte indicates timeout)
                printf("DHT11 Fail: Error reading byte %d (timeout during bit read)\r\n", i);
                SEGGER_SYSVIEW_PrintfHost("DHT11 Fail: Error reading byte %d (timeout)\r\n", i);
                return; // Exit function on error
            }
        }

        uint8_t checksum = data[0] + data[1] + data[2] + data[3];
        // DHT11 typically returns integer values, decimal parts are often zero
        // data[0] = Humidity Integer
        // data[1] = Humidity Decimal (often 0)
        // data[2] = Temperature Integer
        // data[3] = Temperature Decimal (often 0)
        // data[4] = Checksum
        humidity_val = (float)data[0] + (float)data[1]/10.0f;
        temperature_val = (float)data[2] + (float)data[3]/10.0f;

        if (checksum == data[4])
        {
            printf("DHT11: RH=%.1f%% T=%.1fC\r\n", humidity_val, temperature_val);
            SEGGER_SYSVIEW_PrintfHost("DHT11: RH=%.1f%% T=%.1fC\r\n", humidity_val, temperature_val);
            checksum = 0;
        }
        else
        {
            printf("Checksum error: Calc: %02d / Recvd: %02d. Data: %02d %02d %02d %02d %02d\r\n",
                   checksum, data[4], data[0], data[1], data[2], data[3], data[4]);
            SEGGER_SYSVIEW_PrintfHost("Checksum error!\r\n");
            checksum = 0;
        }
    }
    else
    {
        printf("DHT11: No Response\n");
        SEGGER_SYSVIEW_PrintfHost("DHT11: No Response\n");
    }
    // Removed HAL_Delay here. All delays in FreeRTOS tasks should use vTaskDelay.
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
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
    SEGGER_SYSVIEW_Conf();
    setvbuf(stdout, NULL, _IONBF, 0);

    // DHT11 needs a boot-up delay
    HAL_Delay(1000);

    BaseType_t task_created_DHT11 = xTaskCreate(DHT11Task, "DHT11Task", 256, NULL, 3, NULL);
    configASSERT(task_created_DHT11 != pdFAIL);

    vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // This part of the code is typically not reached in a FreeRTOS application
  // unless the scheduler fails to start or is explicitly stopped.
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DHT11Task(void *argument)
{
	//vTaskDelay(pdMS_TO_TICKS(2000));
    while (1)
    {
        Read_DHT11_Data();
        vTaskDelay(pdMS_TO_TICKS(4000)); // DHT11: one read per 2 seconds
    }
}
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
