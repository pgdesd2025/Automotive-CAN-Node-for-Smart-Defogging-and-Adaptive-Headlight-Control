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
#include "main.h" // HAL includes

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h" // FreeRTOS core
#include "task.h"     // Task management
#include "timers.h"   // Software timers (if needed, not used directly here)
#include "queue.h"    // Queue management
#include "semphr.h"   // Semaphores (if needed)
#include "event_groups.h" // Event Groups (if needed)
#include "SEGGER_SYSVIEW.h" // SEGGER SystemView for tracing
// #include "Disc_F407.h" // This header is typically for board-specific initializations like LEDs, not strictly needed for DHT11 logic itself
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Structure to hold DHT11 sensor data (integer parts only for DHT11)
typedef struct {
    uint8_t temperature;
    uint8_t humidity;
    // You could add decimal parts if using DHT22 or want full DHT11 data read:
    // uint8_t temperature_dec;
    // uint8_t humidity_dec;
} DHT_Data_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STACK_SIZE_TASK 128*4 // Stack size for each task (adjust if needed)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1; // Timer handle for TIM1

/* USER CODE BEGIN PV */
static QueueHandle_t dht11_queue = NULL; // Queue for transferring DHT11 data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
// FreeRTOS Task Prototypes
void SendTask(void *arguments);
void RecvTask(void *arguments);

// DHT11 Communication Helper Function Prototypes
void delay_us(uint16_t us);
uint8_t wait_for_pin_state(GPIO_PinState state, uint32_t timeout_us);
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
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
  MX_TIM1_Init(); // Initialize TIM1

  /* USER CODE BEGIN 2 */
  SEGGER_SYSVIEW_Conf(); // Configure SEGGER SystemView for tracing

  // Start TIM1 for microsecond delays
  HAL_TIM_Base_Start(&htim1);

  // Create FreeRTOS Tasks
  // RecvTask has higher priority (tskIDLE_PRIORITY + 3) to process data quickly
  // SendTask has lower priority (tskIDLE_PRIORITY + 2) to perform sensor polling
  assert_param(xTaskCreate(RecvTask, "RecvTask", STACK_SIZE_TASK, NULL, tskIDLE_PRIORITY + 3, NULL) == pdPASS);
  assert_param(xTaskCreate(SendTask, "SendTask", STACK_SIZE_TASK, NULL, tskIDLE_PRIORITY + 2, NULL) == pdPASS);

  // Create Queue for transferring DHT11 data
  dht11_queue = xQueueCreate(8, sizeof(DHT_Data_t)); // Queue can hold 8 DHT_Data_t items
  assert_param(dht11_queue != NULL); // Assert if queue creation fails

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // This loop is typically not reached if the scheduler starts successfully
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // Use HSE (External Crystal)
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;     // 8MHz HSE / 8 = 1MHz
  RCC_OscInitStruct.PLL.PLLN = 336;   // 1MHz * 336 = 336MHz (VCO output)
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // 336MHz / 2 = 168MHz (SYSCLK)
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // SYSCLK from PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;       // HCLK = 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;       // APB1 = 42MHz (timers on APB1 will be 84MHz)
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;       // APB2 = 84MHz (TIM1 on APB2 will be 84MHz)

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) // Adjust flash latency for 168MHz
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
  // Prescaler: (Timer_Clock_Freq / Desired_Freq) - 1
  // Timer_Clock_Freq = APB2 Timer Clock = 84 MHz
  // Desired_Freq = 1 MHz (for 1 us tick)
  // Prescaler = (84,000,000 / 1,000,000) - 1 = 84 - 1 = 83
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535; // Max value for a 16-bit timer
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0; // Not used for base timer
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET; // Not using trigger, but setting a default
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
  __HAL_RCC_GPIOA_CLK_ENABLE(); // Enable clock for GPIOA (PA0)
  __HAL_RCC_GPIOD_CLK_ENABLE(); // Enable clock for GPIOD (if used for LEDs/other)

  /*Configure GPIO pin Output Level for PA0 (DHT11_DATA_PIN) */
  // Initial state is LOW. This will be reconfigured by DHT11_Start()
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Initial configuration from IOC
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // You might have other pins configured here, e.g., for LEDs
  // HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // Example for LED on PD12
  // GPIO_InitStruct.Pin = GPIO_PIN_12;
  // GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  // GPIO_InitStruct.Pull = GPIO_NOPULL;
  // GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  // HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief FreeRTOS Task to read DHT11 data and send it to a queue.
  * @param arguments Not used.
  * @retval None
  */
void SendTask(void *arguments) {
    uint8_t humidity_int, humidity_dec, temp_int, temp_dec, checksum;
    DHT_Data_t sensor_data;

    while (1) {
        SEGGER_SYSVIEW_PrintfHost("Inside Sending Task: Attempting DHT11 read...\n");
        // In main() after SystemClock_Config()
        // Or in a task if you need it later
        SEGGER_SYSVIEW_PrintfHost("APB2 Timer Clock: %lu Hz\n", HAL_RCC_GetPCLK2Freq());
        DHT11_Start(); // Initiate communication with DHT11

        // Check DHT11 response signal
        if (DHT11_CheckResponse()) {
            // Read 5 bytes of data from DHT11
            humidity_int = DHT11_ReadByte();    // Humidity Integer Part
            humidity_dec = DHT11_ReadByte();    // Humidity Decimal Part (usually 0 for DHT11)
            temp_int = DHT11_ReadByte();        // Temperature Integer Part
            temp_dec = DHT11_ReadByte();        // Temperature Decimal Part (usually 0 for DHT11)
            checksum = DHT11_ReadByte();        // Checksum

            // Verify checksum for data integrity
            if (checksum == (humidity_int + humidity_dec + temp_int + temp_dec)) {
                sensor_data.humidity = humidity_int;
                sensor_data.temperature = temp_int;

                // Send data to the queue with a timeout
                if (xQueueSend(dht11_queue, &sensor_data, pdMS_TO_TICKS(100)) == pdPASS) {
                    SEGGER_SYSVIEW_PrintfHost("DHT11 Data Sent: Temp=%d, Hum=%d\n", sensor_data.temperature, sensor_data.humidity);
                } else {
                    SEGGER_SYSVIEW_PrintfHost("DHT11 Data Send Failed (Queue Full/Timeout)\n");
                }
            } else {
                SEGGER_SYSVIEW_PrintfHost("DHT11 Checksum Error! (Calculated: %d, Received: %d)\n",
                                          (humidity_int + humidity_dec + temp_int + temp_dec), checksum);
            }
        } else {
            SEGGER_SYSVIEW_PrintfHost("DHT11 No Response or Invalid Response!\n");
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Sample every 2 seconds
    }
}

/**
  * @brief FreeRTOS Task to receive DHT11 data from queue and print it.
  * @param arguments Not used.
  * @retval None
  */
void RecvTask(void *arguments) {
    DHT_Data_t sensor_data;

    while (1) {
        SEGGER_SYSVIEW_PrintfHost("Inside Receiving Task: Waiting for data...\n");

        // Receive data from the queue, block indefinitely until data is available
        if (xQueueReceive(dht11_queue, &sensor_data, portMAX_DELAY) == pdPASS) {
            SEGGER_SYSVIEW_PrintfHost("Received Data: Temperature: %d C, Humidity: %d %%\n",
                                      sensor_data.temperature, sensor_data.humidity);
        } else {
            SEGGER_SYSVIEW_PrintfHost("RecvTask: Error receiving data from queue.\n");
        }
    }
}

/**
  * @brief Provides a precise microsecond delay using TIM1.
  * @param us Number of microseconds to delay.
  * @retval None
  */
void delay_us(uint16_t us) {
    // Reset the counter if it's nearing overflow or for precise start
    __HAL_TIM_SET_COUNTER(&htim1, 0); // Start counting from 0
    while(__HAL_TIM_GET_COUNTER(&htim1) < us); // Busy-wait until target microseconds
}

/**
  * @brief Waits for the DHT11 data pin to reach a specific state within a timeout.
  * @param state The desired GPIO_PinState (GPIO_PIN_RESET or GPIO_PIN_SET).
  * @param timeout_us Maximum time to wait in microseconds.
  * @retval 1 if pin reaches state, 0 on timeout.
  */
uint8_t wait_for_pin_state(GPIO_PinState state, uint32_t timeout_us) {
    uint32_t start_time = __HAL_TIM_GET_COUNTER(&htim1);
    // Reset counter for precise timeout measurement from current point
    __HAL_TIM_SET_COUNTER(&htim1, 0);

    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) != state) {
        if (__HAL_TIM_GET_COUNTER(&htim1) > timeout_us) {
            return 0; // Timeout occurred
        }
    }
    return 1; // Pin reached desired state
}

/**
  * @brief Initiates communication with the DHT11 sensor.
  * @param None
  * @retval None
  */
void DHT11_Start(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. Configure PA0 as Output Open-Drain with Pull-up
    // This allows the MCU to pull the line low, and when released,
    // the external pull-up (or DHT11's internal pull-up) pulls it high.
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // Open-drain mode
    GPIO_InitStruct.Pull = GPIO_PULLUP;         // Enable internal pull-up
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // Can be high for faster transitions
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 2. Master (MCU) sends start signal: Pull LOW for at least 18ms
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); // Pull data line low
    vTaskDelay(pdMS_TO_TICKS(18)); // FreeRTOS delay for ~18ms

    // 3. Master releases line (pulls HIGH) and waits 20-40us
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); // Release data line (line goes high due to pull-up)
    delay_us(30); // Wait for 30us (DHT11 expects 20-40us high)

    // 4. Switch PA0 to Input mode with Pull-up to read DHT11's response
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Switch to input
    GPIO_InitStruct.Pull = GPIO_PULLUP;     // Keep pull-up enabled for stable high state
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
  * @brief Checks for the DHT11's response signal.
  * @param None
  * @retval 1 if response is valid, 0 otherwise.
  */
uint8_t DHT11_CheckResponse(void) {
    // After master releases line (30us high), DHT11 pulls low for 80us, then high for 80us.
    // We expect the line to go LOW first.
    if (!wait_for_pin_state(GPIO_PIN_RESET, 90)) { // Wait for 80us LOW signal (add margin)
        return 0; // Timeout waiting for DHT11 to pull low
    }
    if (!wait_for_pin_state(GPIO_PIN_SET, 90)) {   // Wait for 80us HIGH signal (add margin)
        return 0; // Timeout waiting for DHT11 to pull high
    }
    return 1; // Response received
}

/**
  * @brief Reads a single bit from the DHT11 sensor.
  * @param None
  * @retval The bit value (0 or 1), or 0 if timeout occurs (needs robust error handling if applicable).
  */
uint8_t DHT11_ReadBit(void) {
    // Each bit starts with a 50us HIGH pulse from DHT11
    if (!wait_for_pin_state(GPIO_PIN_SET, 60)) { // Wait for 50us HIGH start (add margin)
        // SEGGER_SYSVIEW_PrintfHost("DHT11_ReadBit: Timeout waiting for HIGH start\n");
        return 0; // Error or timeout, handle as 0 or propagate error
    }

    uint32_t t_high_start = __HAL_TIM_GET_COUNTER(&htim1); // Mark time when pin went high

    // DHT11 then pulls LOW for:
    // ~26-28us for a '0' bit (total high pulse ~76-78us from start of 50us high)
    // ~70us for a '1' bit (total high pulse ~120us from start of 50us high)
    if (!wait_for_pin_state(GPIO_PIN_RESET, 90)) { // Wait for pin to go LOW (end of bit data pulse)
        // SEGGER_SYSVIEW_PrintfHost("DHT11_ReadBit: Timeout waiting for LOW after HIGH pulse\n");
        return 0; // Error or timeout
    }

    uint32_t t_high_duration = __HAL_TIM_GET_COUNTER(&htim1) - t_high_start; // Measure the 'data' high pulse duration

    // Compare duration to distinguish 0 vs 1
    // A '0' has a shorter high pulse after the 50us, a '1' has a longer one.
    // Threshold is typically around 40us
    return (t_high_duration > 40) ? 1 : 0;
}

/**
  * @brief Reads a full byte (8 bits) from the DHT11 sensor.
  * @param None
  * @retval The read byte.
  */
uint8_t DHT11_ReadByte(void) {
    uint8_t i, byte = 0;
    for (i = 0; i < 8; i++) {
        byte <<= 1; // Shift existing bits to the left
        byte |= DHT11_ReadBit(); // Read next bit and set the LSB
    }
    return byte;
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
      // Blink an LED to indicate error
      // HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12); // Example for LED on PD12
      // HAL_Delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
