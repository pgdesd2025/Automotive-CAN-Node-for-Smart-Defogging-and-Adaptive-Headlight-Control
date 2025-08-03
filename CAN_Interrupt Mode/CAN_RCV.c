/* USER CODE BEGIN Header */
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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//*Structure to hold a complete CAN message for passing via FreeRTOS queue
typedef struct {
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
} CAN_Message_t;

// A union to convert a 32-bit float to a 4-byte array
typedef union {
    float float_value;
    uint8_t byte_array[4];
} FloatConverter;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_ID_INT_RX    0x125 // Standard ID for integer messages
#define CAN_ID_STRING_RX 0x126 // Standard ID for string messages
#define CAN_ID_FLOAT_RX  0x127 // New CAN ID for floating-point messages
#define CAN_MESSAGE_QUEUE_LENGTH 10 // Size of the CAN message queue
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
// FreeRTOS Queue handle for CAN messages
QueueHandle_t xCanRxQueue;

CAN_Message_t receivedMessage;
uint8_t received_integer;
//Buffers for storing received string data
char received_string_buffer[100];
uint8_t string_buffer_index = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void CAN_filterConfig(void);
void vCanRxTask(void *pvParameters);
void vToggleLEDs(void);
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
  MX_CAN1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SEGGER_SYSVIEW_Conf();  // To start the SEGGER //

   // Configure CAN filter to accept messages from both IDs
   CAN_filterConfig();

   // Create the FreeRTOS message queue for CAN messages
   xCanRxQueue = xQueueCreate(CAN_MESSAGE_QUEUE_LENGTH, sizeof(CAN_Message_t));

   if (xCanRxQueue == NULL) {
       // Handle error if queue creation failed
       Error_Handler();
   }

   // Start the CAN peripheral
   if (HAL_CAN_Start(&hcan1) != HAL_OK)
   {
     Error_Handler();
   }

   HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);

   // Activate the CAN RX FIFO 0 message pending interrupt
   if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
   {
       Error_Handler();
   }

   // CRITICAL: Start the TIM2 interrupt. This is necessary for the callback to be triggered.
   if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
   {
       Error_Handler();
   }

   // Create the CAN Rx task
   xTaskCreate(vCanRxTask, "CAN_Rx_Task", configMINIMAL_STACK_SIZE + 200, NULL, tskIDLE_PRIORITY + 2, NULL);

   // Start the FreeRTOS scheduler
   vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while(1)
     {
     }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* USER CODE END CAN1_Init 2 */

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
  htim2.Init.Prescaler = 16;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Configure PB8 and PB9 for CAN1
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief Configures the CAN filter to accept specific IDs.
 * This is a simple ID List mode filter that accepts messages with
 * CAN_ID_INT_RX and CAN_ID_STRING_RX.
 * The filter has been updated to also accept CAN_ID_FLOAT_RX.
 */
void CAN_filterConfig(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

  // The original code used FilterIdHigh and FilterMaskIdHigh for two different IDs.
  // We'll follow this pattern and add the new float ID to FilterIdLow.
  sFilterConfig.FilterIdHigh = CAN_ID_INT_RX << 5;
  sFilterConfig.FilterIdLow = CAN_ID_FLOAT_RX << 5; // New float ID filter
  sFilterConfig.FilterMaskIdHigh = CAN_ID_STRING_RX << 5;
  sFilterConfig.FilterMaskIdLow = 0; // This entry is unused in this configuration

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief This is the Interrupt Service Routine (ISR) callback for CAN Rx.
 * It is triggered when a new message is received in FIFO 0.
 * The function reads the message and sends it to the FreeRTOS queue.
 * This is a time-sensitive function, so it should be as short as possible.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Message_t rxMessage;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Read the received message from the CAN FIFO
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxMessage.RxHeader, rxMessage.RxData) == HAL_OK)
    {
        // Send the received message to the FreeRTOS task via the queue
        // This is an ISR-safe function, so we must use 'FromISR' versions.
        if (xQueueSendFromISR(xCanRxQueue, &rxMessage, &xHigherPriorityTaskWoken) != pdPASS)
        {
            // The queue is full.
        }
    }

    // Check if the ISR woke a higher priority task. If so, a context switch
    // will be performed on exit from the ISR.
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 * @brief Task to process received CAN messages.
 * This task waits on the message queue and handles the received data.
 */
void vCanRxTask(void *pvParameters)
{
	uint32_t intpart = 0;
    CAN_Message_t receivedMessage;
    FloatConverter converter;

    for (;;)
    {
        // Wait indefinitely for a new message to arrive in the queue.
        if (xQueueReceive(xCanRxQueue, &receivedMessage, portMAX_DELAY) == pdPASS)
        {
            // A message was received, toggle the LEDs to indicate reception.
            vToggleLEDs();

            // Check the message ID to process the data
            if (receivedMessage.RxHeader.StdId == CAN_ID_INT_RX)
            {
                // Received an integer message
                // Note: The transmitter sends single bytes, so we can access RxData[0]
                uint8_t received_integer = receivedMessage.RxData[0];
                SEGGER_SYSVIEW_PrintfHost("Received Integer: %d", received_integer);
            }
           /* else if (receivedMessage.RxHeader.StdId == CAN_ID_STRING_RX)
            {
                // Received a single character message
                char received_char = receivedMessage.RxData[0];
                SEGGER_SYSVIEW_PrintfHost("Received Character: '%c'", received_char);
            }
            */
            else if (receivedMessage.RxHeader.StdId == CAN_ID_FLOAT_RX)
            {
                // Received a floating-point message
                // Copy the received bytes to the union to reinterpret as a float
                for (int i = 0; i < 4; i++) {
                    converter.byte_array[i] = receivedMessage.RxData[i];
                }
                intpart = (uint32_t) converter.float_value;
                SEGGER_SYSVIEW_PrintfHost("Received Float: %d.%d", intpart/10, intpart%10);
            }
        }
    }
}

/**
 * @brief Toggles all LEDs together to indicate a successful reception.
 */
void vToggleLEDs(void)
{
    static GPIO_PinState pin_state = GPIO_PIN_RESET;
    uint16_t all_pins = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

    // Toggle all four LEDs
    pin_state = (pin_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOD, all_pins, pin_state);
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
	// This section is for a custom timer.
	    // In this case, we use TIM2 to create a simple heartbeat LED.
	    if (htim->Instance == TIM2) {
	        // USER CODE BEGIN TIM2 Callback
	        static GPIO_PinState pin_state = GPIO_PIN_RESET;
	        pin_state = (pin_state == GPIO_PIN_RESET) ? GPIO_PIN_SET : GPIO_PIN_RESET;
	        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, pin_state);
	        // USER CODE END TIM2 Callback
	    }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	 SEGGER_SYSVIEW_PrintfHost("HAL Error occurred!");
	  __disable_irq();
	  while (1)
	  {
	    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); // Turn on an LED for error
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
	SEGGER_SYSVIEW_PrintfHost("Assertion Failed:file %s on line %d\r\n", file, line);
	while(1);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
