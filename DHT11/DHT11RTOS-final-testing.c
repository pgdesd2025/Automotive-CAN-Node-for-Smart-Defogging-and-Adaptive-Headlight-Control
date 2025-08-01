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
#define DHT11_GPIO_Port GPIOB
#define DHT11_Pin GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t data[5] = {0};
uint8_t checksum, byte;
float humidity, temperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void TaskReadDHT11( void * argument);
uint8_t wait_for_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state, uint32_t timeout_us);
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
void Read_DHT11_Data(void);
//---------------------------------------
void DWT_Init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
//void delay_ms(uint16_t ms);
//void delay_us(uint16_t us);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}

void delay_ms(uint16_t ms)
{
    while (ms--)
    {
        delay_us(1000);
    }
}*/
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
    DWT->CYCCNT = 0;                                 // Reset the counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable the counter
}

void delay_us(uint32_t us)
{
    uint32_t cycles_per_us = HAL_RCC_GetHCLKFreq() / 1000000;
    uint32_t start = DWT->CYCCNT;
    uint32_t delay_ticks = us * cycles_per_us;
    while ((DWT->CYCCNT - start) < delay_ticks);
}

void delay_ms(uint32_t ms)
{
    while(ms--) delay_us(1000);
}

uint16_t timer_delta(uint16_t start, uint16_t end)
{
    return (end >= start) ? (end - start) : (65536 + end - start);
}

uint8_t wait_for_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state, uint32_t timeout_us)
{
    uint16_t start = __HAL_TIM_GET_COUNTER(&htim4);
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != state)
    {
        if (timer_delta(start, __HAL_TIM_GET_COUNTER(&htim4)) > timeout_us)
        {
            return 0; // Timeout
        }
    }
    return 1;
}

void DHT11_SetPinOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;    // Open-drain output
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // Enable pull-up
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void DHT11_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;            // Pull-up enabled
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void DHT11_Start(void)
{
    DHT11_SetPinOutput();
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET); // Pull line low
    delay_ms(18);  // >18 ms low to initiate
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);   // Release line
    delay_us(30); // Wait 20-40us before sensor responds
    DHT11_SetPinInput();
}

uint8_t DHT11_CheckResponse(void)
{
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET, 80)) return 0;
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET, 80)) return 0;
    return 1;
}

uint8_t DHT11_ReadBit(void)
{
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET, 100)) return 0xFF;
    uint16_t tStart = __HAL_TIM_GET_COUNTER(&htim4);
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET, 100)) return 0xFF;
    uint16_t tEnd = __HAL_TIM_GET_COUNTER(&htim4);
    uint16_t duration = timer_delta(tStart, tEnd);
    // >40us HIGH pulse means '1' else '0'
    return (duration > 40) ? 1 : 0;
}

uint8_t DHT11_ReadByte(void)
{
    uint8_t i, val = 0;
    for (i = 0; i < 8; i++)
    {
        uint8_t bit = DHT11_ReadBit();
        if (bit == 0xFF) return 0xFF; // timeout/error
        val <<= 1;
        val |= bit;
    }
    return val;
}

void Read_DHT11_Data(void)
{

    DHT11_Start();
    if (DHT11_CheckResponse())
    {
        for (int i = 0; i < 5; i++)
        {
            data[i] = DHT11_ReadByte();
        }
       delay_us(20);
        checksum = data[0] + data[1] + data[2] + data[3];
        humidity = data[0] + data[1] / 10.0f;
        temperature = data[2] + data[3] / 10.0f;
        //SEGGER_SYSVIEW_PrintfHost("Temp %u.%u Humidity %u.%u\r\n", data[2], data[3], data[0], data[1]);
		//printf("Temp %f, Humidity %f\r\n",temperature, humidity);
        if (checksum == data[4])
        {
        	SEGGER_SYSVIEW_PrintfHost("Temp %u.%u Humidity %u.%u\r\n", data[2], data[3], data[0], data[1]);
            // data[0]: Humidity integer
            // data[1]: Humidity decimal
            // data[2]: Temperature integer
            // data[3]: Temperature decimal
            // data[4]: Checksum value
        }
        else
        {
        	checksum = 0; data[4] = 0;
        }
    }
    else
    {
        // try initiating the connection with dht11
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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  	SEGGER_SYSVIEW_Conf();
  	DWT_Init();
	HAL_TIM_Base_Start(&htim4);
	BaseType_t task_created_dht = xTaskCreate(TaskReadDHT11, "TaskReadDHT11", 256, NULL, 2, NULL); // Lower priority than ultrasonic
	   configASSERT(task_created_dht == pdPASS);

	//start the scheduler - shouldn't return unless there's a problem
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TaskReadDHT11( void * argument)
{
	while(1)
	{
		printf("Task running!\r\n");
		 __disable_irq();
	Read_DHT11_Data();
		__enable_irq();
		int temp_i = (int)temperature;
		int temp_f = (int)((temperature - temp_i) * 100.0f); // 2 decimal places
		if (temp_f < 0) temp_f = -temp_f; // Handle negatives

		int hum_i = (int)humidity;
		int hum_f = (int)((humidity - hum_i) * 100.0f); // 2 decimal places
		if (hum_f < 0) hum_f = -hum_f;

		// Print using SEGGER_SYSVIEW (no %f allowed)
		SEGGER_SYSVIEW_PrintfHost("Temp: %d.%02d C, Humidity: %d.%02d %%\r\n", temp_i, temp_f, hum_i, hum_f);
		printf("Temp %f, Humidity %f\r\n",temperature, humidity);
		data[0] = data[1] = data[2] = data[3] = data[4] = 0;
		printf("DHT11 read done\r\n");
		vTaskDelay(pdMS_TO_TICKS(2000));
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
