/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
  uint8_t command;
  uint8_t Frequency[3];
  uint8_t Threshold[3];
}myUartQueueData_t;

typedef struct{
  uint8_t id;
	uint8_t temp;
  uint8_t humi;
  uint8_t distance;
}myQueueData_t;

uint8_t Rx_data;

#define TransmitBuff_SIZE	35
#define ReceiveBuff_SIZE	10
#define MainBuff_SIZE		8
#define Queue_SIZE			3
uint8_t RxBuffer[ReceiveBuff_SIZE];
uint8_t TxBuffer[TransmitBuff_SIZE];
uint8_t MainBuffer[MainBuff_SIZE];
uint8_t len;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

osThreadId DHTTaskHandleHandle;
/* USER CODE BEGIN PV */
osThreadId HCSR04TaskHandle;            // task HCSR
osThreadId LCDTaskHandle;               // task LCD
osThreadId UARTTaskHandle;              // task xử lý dữ liệu nhận được từ UART
osThreadId ButtonTaskHandle;            // task nút nhấn
osThreadId Uart_ThresholdTaskHandle;    // task xử lý ngưỡng và uart

osMessageQId myQueueDataHandle;                 // Gửi dữ liệu đến LCDTask
osMessageQId myQueueUart_ThresholdHandle;       // Gửi dữ liệu đến Uart_ThresholdTask
osMessageQId myQueueUart_ThresholdDataHandle;   // Gửi ngưỡng, chu kì đến Uart_ThresholdTask
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
void DHTTask(void const * argument);

/* USER CODE BEGIN PFP */
void HCSR04Task(void const * argument);
void LCDTask(void const * argument);
void UARTTask(void const * argument);
void ButtonTask(void const * argument);
void Uart_ThresholdTask(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of DHTTaskHandle */
  osThreadDef(DHTTaskHandle, DHTTask, osPriorityNormal, 0, 128);
  DHTTaskHandleHandle = osThreadCreate(osThread(DHTTaskHandle), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  osMailQDef(myQueueData, 14, myQueueData_t);
  myQueueDataHandle = osMailCreate(osMailQ(myQueueData), NULL);

  osMailQDef(myQueueUart_Threshold, 14, myQueueData_t);
  myQueueUart_ThresholdHandle = osMailCreate(osMailQ(myQueueUart_Threshold), NULL);

  osMailQDef(myQueueUartHCSR, 14, myUartQueueData_t);
  myQueueUart_ThresholdDataHandle = osMailCreate(osMailQ(myQueueUartHCSR), NULL);

  
  osThreadDef(HCSR04TaskHandle, HCSR04Task, osPriorityBelowNormal, 0, 128*4);
  HCSR04TaskHandle  = osThreadCreate(osThread(HCSR04TaskHandle), NULL);

  osThreadDef(LCDTaskHandle, LCDTask, osPriorityAboveNormal, 0, 128*4);
  LCDTaskHandle  = osThreadCreate(osThread(LCDTaskHandle), NULL);

  osThreadDef(UARTTaskHandle, UARTTask, osPriorityAboveNormal, 0, 128*4);
  UARTTaskHandle  = osThreadCreate(osThread(UARTTaskHandle), NULL);

  osThreadDef(ButtonTaskHandle, ButtonTask, osPriorityAboveNormal, 0, 128*4);
  ButtonTaskHandle  = osThreadCreate(osThread(ButtonTaskHandle), NULL);

  osThreadDef(Uart_ThresholdTaskHandle, Uart_ThresholdTask, osPriorityAboveNormal, 0, 128*4);
  Uart_ThresholdTaskHandle  = osThreadCreate(osThread(Uart_ThresholdTaskHandle), NULL);

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */



// Xử lý ngắt nút nhấn
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriority;
  xHigherPriority = xTaskResumeFromISR(ButtonTask);
  portEND_SWITCHING_ISR(xHigherPriority);
}
// Xử lý ngắt  UART, nhảy vào hàm 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  char ISR_data[14] = "Hello DevIoT";
  HAL_UART_Receive_IT(&huart1, &Rx_data, 1);
  if(Rx_data == 'k')
  {
    printf("send data from ISR\n");
    BaseType_t xHigher = xTaskResumeFromISR(UARTTask);
    portEND_SWITCHING_ISR(xHigher);
  }
}

// Task đọc và gửi nhiệt độ

// Task đọc và gửi khoảng cách
void HCSR04Task(void const * argument)
{
  int8_t Distance = 0;

  myQueueData_t *msg = osMailAlloc(myQueueDataHandle, osWaitForever);
  msg->id = 2;    // Gửi từ HCSR04Task tới LCDTask
  myQueueData_t *msg1 = osMailAlloc(myQueueUart_ThresholdDataHandle, osWaitForever);
  msg1->id = 2;   // Gửi từ HCSR04Task tới Uart_ThresholdTask
  
  while(1)
  {
  // Đo khoảng cách lưu vào biết Distance
   msg->distance = Distance;      // biến khoảng cách
   osMailPut(myQueueDataHandle, msg);

   msg1->temp = Distance++;      
   osMailPut(myQueueUart_ThresholdDataHandle, msg1);

   }
}
// Task hiển thị LCD
void LCDTask(void const * argument)
{
  osEvent Recv_Task_data; // nhận dữ liệu từ Queue01
  while(1)
  {
    uint8_t temp, humi, dist;
    // Nhận dữ liệu từ DHT và HCSR
    Recv_Task_data = osMailGet(myQueueDataHandle, 0);
    if (Recv_Task_data.status == osEventMail)
    {
    myQueueData_t *data = Recv_Task_data.value.p; // con trỏ đến cấu trúc lưu nhiệt độ, độ ẩm, khoảng cách
    if(data->id == 1) // Nhận nhiệt độ độ ẩm
    {
      temp = data->temp;
      humi = data->humi;
    }
    else              // Nhận Khoảng cách
    {
      dist = data->distance;
    }
// nhận xong phải giải phóng
    osMailFree(myQueueDataHandle, data);
// Xử lý hiển thị

    
    }
  }
}
// Task xử lý khi nhân được ngắt UART
// Nhận dữ liệu và cập nhật ngưỡng vào Queue2 để các task khác xử lý
// Thêm 1 task ngưỡng, dùng suspend và resume
void UARTTask(void const * argument)
{
  myUartQueueData_t *msg = osMailAlloc(myQueueUart_ThresholdHandle, osWaitForever);
  while(1)
  {
    vTaskSuspend(NULL);
// Đưa các biến Command; Frequency; Threshold vào Queue2 để các task khác xử lý

// gửi đến DHTTask, thay 1 bằng dữ liệu nhân được từ UART
    
    msg->Frequency[0] = 1;         // chu kì nhiệt độ DHTTask
    msg->Frequency[1] = 1;         // chu kì độ ẩm DHTTask
    msg->Frequency[2] = 1;         // chu kì HCSR04ask
    msg->Threshold[0] = 1;         // Ngưỡng nhiệt
    msg->Threshold[1] = 1;         // Ngưỡng độ ẩm
    msg->Threshold[2] = 1;         // Ngưỡng khoảng cách
    osMailPut(myQueueUart_ThresholdHandle, msg);

// Xử lý command    

  }
}
//Task xử lý khi nhấn nút
void ButtonTask(void const * argument)
{
  while(1)
  {
    vTaskSuspend(NULL);
    printf("hello from AboveNormal Task\n");
  }
}

//Task xử lý ngưỡng
void Uart_ThresholdTask(void const * argument)
{
  uint8_t Frequency_Temp = 100;
  uint8_t Frequency_Humi = 100;
  uint8_t Frequency_Dist = 100;
	
  uint8_t Threshold_Temp = 100;
  uint8_t Threshold_Humi = 100;
  uint8_t Threshold_Dist = 100;
	
  uint8_t temp = 100;
  uint8_t humi = 100;
  uint8_t dist = 100;
  
  osEvent Recv_Task_data_uart;  // nhận dữ liệu từ myQueueUart_ThresholdHandle
  osEvent Recv_Task_data; // nhận dữ liệu từ DHT và HCSR
  while(1)
  {

    // Nhận dữ liệu từ myQueueUart_ThresholdHandle
    Recv_Task_data_uart = osMailGet(myQueueUart_ThresholdHandle, 0);
    if (Recv_Task_data_uart.status == osEventMail)
    {
        myUartQueueData_t *data = Recv_Task_data_uart.value.p; // con trỏ đến cấu trúc lưu nhiệt độ, độ ẩm
        Frequency_Temp = data->Frequency[0];              // chu kì nhiệt độ DHT
        Frequency_Humi = data->Frequency[1];              // chu kì độ ẩm DHT
        Frequency_Dist = data->Frequency[2];              // chu kì khoảng cách
        Threshold_Temp = data->Threshold[0];              // Ngưỡng nhiệt
        Threshold_Humi = data->Threshold[1];              // Ngưỡng độ ẩm
        Threshold_Dist = data->Threshold[2];              // Ngưỡng khoảng cách
        osMailFree(myQueueUart_ThresholdHandle, data);
    }

    // Nhận dữ liệu từ myQueueUart_ThresholdHandle
    // Nhận dữ liệu từ DHT và HCSR
    Recv_Task_data = osMailGet(myQueueUart_ThresholdDataHandle, 0);
    if (Recv_Task_data.status == osEventMail)
    {
        myQueueData_t *data = Recv_Task_data.value.p; // con trỏ đến cấu trúc lưu nhiệt độ, độ ẩm, khoảng cách
        if(data->id == 1) // Nhận nhiệt độ độ ẩm
        {
        temp = data->temp;
        humi = data->humi;
        }
        else              // Nhận Khoảng cách
        {
        dist = data->distance;
        }
        // nhận xong phải giải phóng
        osMailFree(myQueueUart_ThresholdDataHandle, data);
    }

// Xử lý ngưỡng, Bật Led

// Xử lý Uart
    if((xTaskGetTickCount() % Frequency_Dist) == 0 ) // gửi sau chu kì Frequency_Dist
    {
      memset(TxBuffer, 0, TransmitBuff_SIZE);
	    sprintf((char*)TxBuffer, "\nTemperature: %doC, Humidity: %d%%, Distance: %d cm, Time: %d\n", temp, humi, dist, (int)xTaskGetTickCount());
	    for(size_t i = 0; i < sizeof(TxBuffer); i++){
		  if(TxBuffer[i] == '\0'){
			 len = i + 1;
	   	}
    }
    }
}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_DHTTask */
/**
  * @brief  Function implementing the DHTTaskHandle thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_DHTTask */
void DHTTask(void const * argument)
{
  uint8_t temp = 0;
  uint8_t humi = 0;

  myQueueData_t *msg = osMailAlloc(myQueueDataHandle, osWaitForever);
  msg->id = 1;    // Gửi từ DHTTask tới LCDTask
  myQueueData_t *msg1 = osMailAlloc(myQueueUart_ThresholdDataHandle, osWaitForever);
  msg1->id = 1;   // Gửi từ DHTTask tới Uart_ThresholdTask

  while(1)
  {  
// Đưa nhiệt độ, độ ẩm vào myQueueDataHandle để hiển thị LCD
// temp: nhiệt độ đo được bởi DHT
   msg->temp = temp;      // biến nhiệt độ đo được
   msg->humi = humi;
   osMailPut(myQueueDataHandle, msg);
// gửi sang Task Threshold để xử lý
   msg1->temp = temp;      // biến nhiệt độ đo được
   msg1->humi = humi;
   osMailPut(myQueueUart_ThresholdDataHandle, msg1);



   }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
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
