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
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint8_t data[5] = {0};
uint8_t checksum_val;    // Renamed to avoid conflict
uint8_t read_byte_val;   // Renamed to avoid conflict
float humidity_val;      // Renamed to avoid conflict
float temperature_val;   // Renamed to avoid conflict
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void DHT11Task(void *argument);
void delay_us(uint16_t us);
uint8_t wait_for_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state, uint32_t timeout_us);
void DHT11_SetPinAsOutput(void); // Helper to set pin as output push-pull
void DHT11_SetPinAsInput(void);  // Helper to set pin as input
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
void Read_DHT11_Data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief Provides a microsecond delay using TIM1.
 * @param us Number of microseconds to delay.
 */
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0); // Reset TIM1 counter
    while (__HAL_TIM_GET_COUNTER(&htim1) < us); // Wait until counter reaches 'us'
}

/**
 * @brief Waits for a GPIO pin to reach a specific state within a timeout.
 * @param GPIOx GPIO Port (e.g., GPIOA).
 * @param GPIO_Pin GPIO Pin (e.g., GPIO_PIN_2).
 * @param state The desired state (GPIO_PIN_RESET or GPIO_PIN_SET).
 * @param timeout_us Timeout in microseconds.
 * @return 1 if pin reached state within timeout, 0 otherwise.
 */
uint8_t wait_for_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state, uint32_t timeout_us)
{
    uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim1); // Get current timer value
    while (HAL_GPIO_ReadPin(GPIOx, GPIO_Pin) != state) {
        // Check for timeout
        if ((__HAL_TIM_GET_COUNTER(&htim1) - start_time) > timeout_us) {
            return 0; // Timeout occurred
        }
    }
    return 1; // Pin reached desired state
}

/**
 * @brief Configures DHT11 data pin as output push-pull.
 * Ensures the pin can actively drive high or low.
 */
void DHT11_SetPinAsOutput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Use Push-Pull for active driving
    GPIO_InitStruct.Pull = GPIO_NOPULL;         // Rely on external pull-up for high
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // High speed for faster transitions
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Configures DHT11 data pin as input.
 * Allows the MCU to read the state driven by DHT11 or the external pull-up.
 */
void DHT11_SetPinAsInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Input mode
    GPIO_InitStruct.Pull = GPIO_NOPULL;     // Rely on external pull-up
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief Initiates communication with the DHT11 sensor.
 * MCU sends start signal to DHT11.
 */
void DHT11_Start(void)
{
    DHT11_SetPinAsOutput(); // Set pin as output to send start signal
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET); // Pull low
    HAL_Delay(18); // Wait for at least 18ms (DHT11 datasheet requirement)
    HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET); // Pull high
    delay_us(30); // Wait for 20-40us (DHT11 datasheet requirement)
    DHT11_SetPinAsInput(); // Switch to input mode to listen for response
}

/**
 * @brief Checks for the DHT11 sensor's response signal.
 * DHT11 pulls low for 80us, then high for 80us.
 * @return 1 if response is valid, 0 otherwise.
 */
uint8_t DHT11_CheckResponse(void)
{
    // Wait for DHT11 to pull line LOW (80us response signal)
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET, 100)) return 0;

    // Wait for DHT11 to pull line HIGH (80us response signal)
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET, 100)) return 0;

    return 1; // Response received
}

/**
 * @brief Reads a single bit from the DHT11 data stream.
 * @return The read bit (0 or 1), or 0 on timeout.
 */
uint8_t DHT11_ReadBit(void)
{
    // Each bit starts with a 50us LOW pulse from DHT11
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET, 60)) return 0; // Wait for line to go HIGH (start of data pulse)

    uint32_t high_start_time = __HAL_TIM_GET_COUNTER(&htim1); // Record time when line goes HIGH
    // Wait for line to go LOW again (end of data pulse)
    if (!wait_for_pin_state(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_RESET, 100)) return 0; // Max 80us for HIGH pulse

    uint32_t high_duration = __HAL_TIM_GET_COUNTER(&htim1) - high_start_time; // Calculate HIGH pulse duration

    uint32_t pulse_length = (high_duration >= high_start_time) ? (high_duration - high_start_time) : (65535 - high_duration + high_start_time + 1);
    // Interpret bit based on HIGH pulse duration:
    // DHT11 datasheet: '0' is ~26-28us HIGH, '1' is ~70us HIGH.
    // A threshold of 40us is commonly used to distinguish.
    return (pulse_length > 35) ? 1 : 0;
}

/**
 * @brief Reads a single byte (8 bits) from the DHT11 data stream.
 * @return The read byte.
 */
uint8_t DHT11_ReadByte(void)
{
    uint8_t byte_val = 0;
    for (int i = 0; i < 8; i++)
    {
        byte_val <<= 1; // Shift left to make room for the new bit
        byte_val |= DHT11_ReadBit(); // Read the next bit and OR it into the byte
    }
    return byte_val;
}

/**
 * @brief Reads all 5 bytes of data from the DHT11 sensor and processes them.
 */
void Read_DHT11_Data(void)
{
    DHT11_Start(); // Initiate communication
    if (DHT11_CheckResponse()) // Check for DHT11's response
    {
        // Read 5 bytes of data
        for (int i = 0; i < 5; i++)
        {
            data[i] = DHT11_ReadByte();
        }

        // Calculate checksum
        checksum_val = data[0] + data[1] + data[2] + data[3];

        // Process data only if checksum is valid
        if (checksum_val == data[4])
        {
            humidity_val = (float)data[0] + ((float)data[1] / 10.0f);
            temperature_val = (float)data[2] + ((float)data[3] / 10.0f);

            // Print data to SEGGER SystemView and standard printf
            SEGGER_SYSVIEW_PrintfHost("DHT11: T=%.1fC, RH=%.1f%%\r\n", temperature_val, humidity_val);
            printf("DHT11: T=%.1fC, RH=%.1f%%\r\n", temperature_val, humidity_val);

            // Removed the problematic HAL_GPIO_TogglePin on PA2.
            // If you need an indicator, use a separate LED pin.
        }
        else
        {
            // Checksum mismatch: Data is corrupted.
            SEGGER_SYSVIEW_PrintfHost("DHT11 Read Failed: Checksum Mismatch!\r\n");
            printf("DHT11 Read Failed: Checksum Mismatch!\r\n");
        }
    }
    else
    {
        // No response from DHT11
        SEGGER_SYSVIEW_PrintfHost("DHT11 Read Failed: No Response!\r\n");
        printf("DHT11 Read Failed: No Response!\r\n");
    }
    // Clear buffer for next read (good practice)
    for (int i = 0; i < 5; i++)
    {
        data[i] = 0;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1); // Start TIM1 base timer for microsecond delays
  SEGGER_SYSVIEW_Conf();      // Configure SEGGER SystemView

  // Disable buffering for printf, useful for immediate output in debug consoles
  setvbuf(stdout, NULL, _IONBF, 0);

  BaseType_t task_created_DHT11 = xTaskCreate(DHT11Task, "DHT11Task", 256, NULL, 3, NULL);
  configASSERT(task_created_DHT11 != pdFAIL); // Use pdFAIL for robustness

  vTaskStartScheduler(); // Start the FreeRTOS scheduler
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 15;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DHT11Task(void *argument)
{
	while(1)
	{
		Read_DHT11_Data(); // Call the function to read and print DHT11 data
		vTaskDelay(pdMS_TO_TICKS(2000)); // Delay for 2 seconds before next read
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
