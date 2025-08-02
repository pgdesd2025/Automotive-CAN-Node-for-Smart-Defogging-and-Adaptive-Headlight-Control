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

#define ULTRASONIC_TRIG_PIN       GPIO_PIN_4
#define ULTRASONIC_TRIG_PORT      GPIOB
#define ADC_BUFFER_SIZE           100 // Or 1000 for 1000 samples
#define ADC_RESOLUTION_BITS       12
#define ADC_MAX_VALUE             ((1 << ADC_RESOLUTION_BITS) - 1) // 4095 for 12-bit
#define ADC_INTERNAL_VREF         5.0 // Internal ADC reference voltage (Vref+) in Volts
#define VOLTAGE_DIVIDER_RATIO     1.0 // ASSUMING your LDR module outputs 0-3.3V directly

// Adjust these values based on your specific STM32's HCLK frequency
// For 16MHz HCLK, 1us = 16 cycles
#define HCLK_FREQ_MHZ             (HAL_RCC_GetHCLKFreq() / 1000000)
#define DHT11_GPIO_Port GPIOB
#define DHT11_Pin GPIO_PIN_0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
//----------------------UV1----------------------
volatile uint32_t ic_timestamp_rising = 0;
volatile uint32_t ic_timestamp_falling = 0;
volatile uint8_t capture_rising_edge_done = 0; // State machine for echo capture
volatile uint32_t measured_distance_cm = 0;

SemaphoreHandle_t xUltrasonicEchoSemaphore = NULL;

//----------------------ADC----------------------
uint32_t adc_dma_buffer[ADC_BUFFER_SIZE];
volatile uint8_t adc_half_transfer_complete = 0;
volatile uint8_t adc_full_transfer_complete = 0;

//----------------------SPI----------------------
uint8_t txbuff[2];			//address,value
uint8_t rx_x,rx_y,rx_z;		//to store the values of xyz
volatile uint8_t xFL, yFL, zFL; //flags for xyz

//----------------------DHT11----------------------
uint8_t data[5] = {0};
uint8_t checksum, byte;
float humidity, temperature;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
void delay_us(uint32_t us);
void UltrasonicSensorTask(void *argument);
void AdcProcessingTask(void *argument);
void SPIProcessingTask(void *argument);
void TaskReadDHT11( void * argument);
static void MX_NVIC_Init(void);
void delay_ms(uint32_t ms);
uint8_t wait_for_pin_state(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state, uint32_t timeout_us);
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_ReadBit(void);
uint8_t DHT11_ReadByte(void);
void Read_DHT11_Data(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//SemaphoreHandle_t mutexPtr = NULL;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  MX_NVIC_Init();
    DWT_Init(); // Initialize DWT for microsecond delays
    SEGGER_SYSVIEW_Conf(); // Configure SystemView
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    // Create binary semaphore for ultrasonic echo capture event
    xUltrasonicEchoSemaphore = xSemaphoreCreateBinary();
    configASSERT(xUltrasonicEchoSemaphore != NULL);

    HAL_TIM_IC_Start(&htim3, 2);
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_IC_Start(&htim4, 4);

    // Start ADC conversion in DMA mode
    // The ADC will continuously convert and transfer data to adc_dma_buffer.
    // Since DMA is in Circular mode, it will wrap around when the buffer is full.
    if (HAL_ADC_Start_DMA(&hadc1, adc_dma_buffer, ADC_BUFFER_SIZE) != HAL_OK)
    {
      Error_Handler(); // Handle error if DMA start fails
    }

    // Create FreeRTOS tasks

    BaseType_t task_created_ultrasonic = xTaskCreate(UltrasonicSensorTask, "UltrasonicSensorTask", 256, NULL, 2, NULL);
    configASSERT(task_created_ultrasonic == pdPASS);

    // ADC tasks
    BaseType_t task_created_adc = xTaskCreate(AdcProcessingTask, "AdcProcessingTask", 256, NULL, 2, NULL); // Lower priority than ultrasonic
    configASSERT(task_created_adc == pdPASS);

    //i2c for accelerometer
    BaseType_t task_created_spi = xTaskCreate(SPIProcessingTask, "SPITask", 256, NULL, 2, NULL); // Lower priority than ultrasonic
     configASSERT(task_created_spi == pdPASS);

     BaseType_t task_created_dht = xTaskCreate(TaskReadDHT11, "TaskReadDHT11", 256, NULL, 2, NULL); // Lower priority than ultrasonic
     	   configASSERT(task_created_dht == pdPASS);

     //mutexPtr = xSemaphoreCreateMutex();
     	//  	assert_param(mutexPtr != NULL);
    //----------------------SPI----------------------
     //PULL CS PIN LOW
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
      txbuff[0]=0x20;  //address of CTRL REG
      txbuff[1]=0x37;      //value

      HAL_SPI_Transmit(&hspi1, txbuff, 2, 50);

      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET); // CS PULLED HIGH
  		//----------------------SPI----------------------
    // Start the FreeRTOS scheduler
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

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
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief Initializes the DWT for microsecond delays.
 * Required for delay_us function.
 */
//-----------------------------------DELAY-------------------------------------------
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

/**
 * @brief FreeRTOS Task for Ultrasonic Sensor Measurement.
 * @param argument Not used.
 */
//-------------------------------------ULTRASONIC---------------------------------------------
void UltrasonicSensorTask(void *argument)
{
	printf("Task running ULTRASONIC!\r\n");

    // Give the semaphore once at the beginning to ensure the first xSemaphoreTake doesn't block forever
    // if the ISR is somehow triggered before the task is ready. Or, more robustly,
    // ensure the initial state is clean. For this, we'll just ensure the ISR is disabled.
    // xSemaphoreGive(xUltrasonicEchoSemaphore); // Not needed if task manages start/stop

    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // Measure every 500ms

    xLastWakeTime = xTaskGetTickCount(); // Initialize xLastWakeTime

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xFrequency); // Delay until the next cycle

        // Reset timer counter before starting a new measurement
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        ic_timestamp_rising = 0;
		ic_timestamp_falling = 0;
		capture_rising_edge_done = 0;
		measured_distance_cm = 0;
        // Ensure input capture polarity is set to RISING for the start of echo pulse
       // __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
        // Enable the input capture interrupt for TIM3 Channel 2



        // Generate trigger pulse
        HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
        delay_us(2);
        HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_SET);
        delay_us(10);
        HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT, ULTRASONIC_TRIG_PIN, GPIO_PIN_RESET);
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
        // Wait for echo capture semaphore with a timeout (e.g., 100ms for max range ~17m)
        if (xSemaphoreTake(xUltrasonicEchoSemaphore, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // Data received and processed in ISR, now print/use it
        	vTaskDelay(pdMS_TO_TICKS(10000));
            SEGGER_SYSVIEW_PrintfHost("Distance: %lu cm\r\n", measured_distance_cm);
            printf("Distance: %lu cm\r\n", measured_distance_cm); // Requires UART/SWV printf setup
        }
        else
        {
            // Timeout occurred, no valid echo received
            measured_distance_cm = 0; // Indicate no valid reading
            SEGGER_SYSVIEW_PrintfHost("Timeout or no Echo\r\n");
            printf("Timeout or no Echo\r\n"); // Requires UART/SWV printf setup
            // Ensure the interrupt is disabled if it wasn't already (e.g., if no falling edge)
            //HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2);
        }
    }
}

/**
 * @brief TIM3 Interrupt Handler.
 * This function should be called from the TIM3_IRQHandler in stm32f4xx_it.c
 * (or directly implemented if you override the default HAL handler).
 */
//-------------------------------------------LDR----------------------------------------------
void TIM3_IRQHandler(void)
{
    // Ensure this is the input capture interrupt for Channel 2
    if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC2) != RESET)
    {
        if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC2) != RESET)
        {
            // Clear the interrupt flag
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);

            if (capture_rising_edge_done == 0)
            {
                // First edge: Rising edge of echo pulse
                ic_timestamp_rising = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
                capture_rising_edge_done = 1;
                // Switch to capture falling edge
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);
            }
            else
            {
                // Second edge: Falling edge of echo pulse
                ic_timestamp_falling = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

                // Calculate pulse duration (in microseconds, as timer tick is 1us)
                uint32_t pulse_duration;
                if (ic_timestamp_falling >= ic_timestamp_rising)
                {
                    pulse_duration = ic_timestamp_falling - ic_timestamp_rising;
                }
                else
                {
                    // Handle timer overflow if it happened between rising and falling edges
                    // This is rare for short ultrasonic pulses but good practice
                    pulse_duration = (htim3.Init.Period + 1 - ic_timestamp_rising) + ic_timestamp_falling;
                }

                // Calculate distance in cm: (time in us * speed of sound in cm/us) / 2
                // Speed of sound approx 0.0343 cm/us at 20°C
                if (pulse_duration < 150 || pulse_duration > 35000)
                {
                    measured_distance_cm = 0; // ignore invalid measurements (too close or too far/no-echo)
                }
                else
                {
                    measured_distance_cm = (uint32_t)((double)pulse_duration * 0.0343 / 2.0);
                }

                // Reset state for next measurement
                capture_rising_edge_done = 0; measured_distance_cm = 0;
                ic_timestamp_rising = 0; ic_timestamp_falling = 0;
                // Switch back to capture rising edge for the next cycle
                __HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
                // Disable the input capture interrupt until the next trigger
                HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_2); // Use HAL_TIM_IC_Stop_IT for clean disable

                // Give semaphore to unblock the FreeRTOS task
                BaseType_t higher_priority_task_woken = pdFALSE;
                xSemaphoreGiveFromISR(xUltrasonicEchoSemaphore, &higher_priority_task_woken);
                portYIELD_FROM_ISR(higher_priority_task_woken);
            }
        }
    }

}


/**
 * @brief FreeRTOS Task for ADC Data Processing.
 * @param argument Not used.
 */
void AdcProcessingTask(void *argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // Check for new ADC data every 100ms

    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
    	printf("Task running ADC !\r\n");
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Process half of the buffer (first half) if ready
        if (adc_half_transfer_complete)
        {
            adc_half_transfer_complete = 0; // Clear the flag

            double sum_voltage = 0.0f;
            for (int i = 0; i < ADC_BUFFER_SIZE / 2; i++)
            {
                // Convert raw ADC value to voltage at the pin (0-3.3V)
                float voltage_at_pin = (float)adc_dma_buffer[i] * (ADC_INTERNAL_VREF / ADC_MAX_VALUE);
                // Scale back to original input voltage (0-5V) using voltage divider ratio
                float original_input_voltage = voltage_at_pin / VOLTAGE_DIVIDER_RATIO;
                sum_voltage += original_input_voltage;
            }
            float average_voltage = sum_voltage / (ADC_BUFFER_SIZE / 2);

            // --- CORRECTED PRINTING FOR SEGGER_SYSVIEW_PrintfHost ---
            // Manually separate integer and fractional parts for printing
            int integer_part = (int)average_voltage;
            // Get 3 decimal places: multiply by 1000, cast to int, then take modulo 1000
            // Ensure handling of negative numbers if applicable (though voltage won't be negative here)
            int fractional_part = (int)((average_voltage - integer_part) * 1000.0f);
            if (fractional_part < 0) fractional_part = -fractional_part; // Handle potential negative fractional part for robustness
            vTaskDelay(pdMS_TO_TICKS(10000));
            SEGGER_SYSVIEW_PrintfHost("ADC Half-Buffer Avg: %d.%03d V\r\n", integer_part, fractional_part);
            printf("ADC Half-Buffer Avg: %.3fV (0-5V scaled)\r\n", average_voltage); // This printf should work if configured for float
        }

        // Process full buffer (second half) if ready
        if (adc_full_transfer_complete)
        {
            adc_full_transfer_complete = 0; // Clear the flag

            double sum_voltage = 0.0f;
            for (int i = ADC_BUFFER_SIZE / 2; i < ADC_BUFFER_SIZE; i++)
            {
                float voltage_at_pin = (float)adc_dma_buffer[i] * (ADC_INTERNAL_VREF / ADC_MAX_VALUE);
                float original_input_voltage = voltage_at_pin / VOLTAGE_DIVIDER_RATIO;
                sum_voltage += original_input_voltage;
            }
            float average_voltage = sum_voltage / (ADC_BUFFER_SIZE / 2);
            // Removed vTaskDelay(pdMS_TO_TICKS(2000)); from here, as it would block ADC processing.
            // If you need a delay, consider placing it outside the processing blocks or in a separate task.

            // --- CORRECTED PRINTING FOR SEGGER_SYSVIEW_PrintfHost ---
            // Manually separate integer and fractional parts for printing
            int integer_part = (int)average_voltage;
            // Get 3 decimal places: multiply by 1000, cast to int, then take modulo 1000
            int fractional_part = (int)((average_voltage - integer_part) * 1000.0f);
            if (fractional_part < 0) fractional_part = -fractional_part; // Handle potential negative fractional part for robustness
            vTaskDelay(pdMS_TO_TICKS(10000));
            SEGGER_SYSVIEW_PrintfHost("ADC Full-Buffer Avg: %d.%03d V\r\n", integer_part, fractional_part);
            printf("ADC Full-Buffer Avg: %.3fV (0-5V scaled)\r\n", average_voltage); // This printf should work if configured for float
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1) // Check if the callback is for ADC1
  {
    adc_full_transfer_complete = 1; // Set flag when the entire buffer is filled
  }
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1) // Check if the callback is for ADC1
  {
    adc_half_transfer_complete = 1; // Set flag when half of the buffer is filled
  }
}

static void MX_NVIC_Init(void)
{
  /* TIM3_IRQn interrupt configuration */
  // Priority set for FreeRTOS compatibility:
  // Lower numerical value means higher priority.
  // Must be >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
  // (or configMAX_SYSCALL_INTERRUPT_PRIORITY for older FreeRTOS versions)
  // For STM32F4, typically 5 is a safe priority for peripheral interrupts
  // that call FreeRTOS API functions.
  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  /* ADC global interrupt configuration */
  // This is for ADC error/regular callbacks, if enabled.
  // Set priority compatible with FreeRTOS.
  HAL_NVIC_SetPriority(ADC_IRQn, 5, 0); // Assuming ADC_IRQn is the correct IRQ for your ADC
  HAL_NVIC_EnableIRQ(ADC_IRQn);

  /* DMA2_Stream0_IRQn interrupt configuration */
  // This is for DMA transfer complete/half-complete callbacks.
  // Set priority compatible with FreeRTOS.
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0); // Assuming DMA2 Stream0 is used for ADC
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}
//-------------------------------------------------SPI--------------------------------------
void SPIProcessingTask(void *argument)
{
	 while (1)
	  {
		 printf("Task running SPI !\r\n");
		 //2.READ THE VALUE OF X AXIS
		 	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
		 	  	  txbuff[0] = 0x29 | 0x80;  //address to be read
		 	  	  HAL_SPI_Transmit(&hspi1, txbuff,1, 50); 		//transmit the address to be read
		 	  	  HAL_SPI_Receive(&hspi1, &rx_x, 1, 50);		//Receive tore in rx_x
		 	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);

		 	  	vTaskDelay(pdMS_TO_TICKS(2));

		 	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
		 	  	  txbuff[0] = 0x2B | 0x80;  //address to be read
		 	  	  HAL_SPI_Transmit(&hspi1, txbuff,1, 50); 		//transmit the address to be read
		 	  	  HAL_SPI_Receive(&hspi1, &rx_y, 1, 50);		//Receive tore in rx_x
		 	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);

		 	  	vTaskDelay(pdMS_TO_TICKS(2));

		 	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, RESET);
		 	  	  txbuff[0] = 0x2D | 0x80;  				//address to be read
		 	  	  HAL_SPI_Transmit(&hspi1, txbuff,1, 50); 	//transmit the address to be read
		 	  	  HAL_SPI_Receive(&hspi1, &rx_z, 1, 50);	//Receive  and store in rx_x
		 	  	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, SET);

		 	  	  vTaskDelay(pdMS_TO_TICKS(10000));

		 	  	  SEGGER_SYSVIEW_PrintfHost("x: %d, y: %d, z: %d\n", rx_x,rx_y,rx_z);
			      printf("x: %d, y: %d, z: %d\n", rx_x,rx_y,rx_z);

		 	  	  if(rx_x>125 && rx_y<125 && rx_z<125)
		 	  	  {
		 	  		  xFL = 1;
		 	  	  }
		 	  	  else if(rx_x<125 && rx_y>125 && rx_z<125)
		 	  	  {
		 	  		  yFL = 1;
		 	  	  }
		 	  	  else if(rx_x<125 && rx_y<125 && rx_z>125)
		 	  	  {
		 	  		  zFL = 1;
		 	  	  }
		 	  	  else
		 	  	  {
		 	  		  xFL = yFL = zFL = 0;
		 	  	  }

		 	  	  if(xFL || yFL)
		 	  	  {
		 	  		  SEGGER_SYSVIEW_PrintfHost("x: %d, y: %d, z: %d\n", rx_x,rx_y,rx_z);
		 	  		  printf("x: %d, y: %d, z: %d\n", rx_x,rx_y,rx_z);
		 	  	  }

		   }
}
//----------------------------------------------DHT-------------------------------------
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
    GPIO_InitStruct.Pull = GPIO_NOPULL;            // Enable pull-up
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_DeInit(DHT11_GPIO_Port, GPIO_PIN_0);
    HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);
}

void DHT11_SetPinInput(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_DeInit(DHT11_GPIO_Port, GPIO_PIN_0);// Pull-up enabled
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
void TaskReadDHT11( void * argument)
{
	while(1)
	{
		printf("Task running DHT !\r\n");
		//if (xSemaphoreTake(mutexPtr, 20) == pdTRUE)
		//{
		taskENTER_CRITICAL();
		    Read_DHT11_Data();
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
			vTaskDelay(pdMS_TO_TICKS(10000));
		taskEXIT_CRITICAL();
		   // xSemaphoreGive(mutexPtr);
		//}
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
