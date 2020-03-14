/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_hd44780_i2c.h"
#include "bmp180.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "meteo_home.h"
#include "RF24.h"
#include "transmitter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const uint64_t pipe_addresses[6] = {
	0xF0F0F0F0D2LL,
	0xF0F0F0F0E1LL,
	0xF0F0F0F0E2LL,
	0xF0F0F0F0E3LL,
	0xF0F0F0F0E4LL,
	0xF0F0F0F0E5LL
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

SPI_HandleTypeDef hspi1;

osThreadId defaultTaskHandle;
osThreadId lcd1602Handle;
osThreadId bmp180SensorHandle;
osThreadId myRadioHandle;
osMessageQId sensorTempHandle;
osMessageQId msgClockTickHandle;
osMessageQId msgRadioTransmitHandle;
osTimerId realTimeClockHandle;
/* USER CODE BEGIN PV */


osPoolDef(bmp_pool, 16, BMP180_DATA);                    // Define memory pool
osPoolId  bmp_pool;
osMessageQDef(bmp180MailTempHandler, 16, BMP180_DATA);              // Define message queue
osMessageQId bmp180MailTempHandler;
osMessageQId  MsgBox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void const * argument);
void StartLcd1602(void const * argument);
void StartBmp180Sensor(void const * argument);
void StartRadio(void const * argument);
void CallbackTimerClock(void const * argument);

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of realTimeClock */
  osTimerDef(realTimeClock, CallbackTimerClock);
  realTimeClockHandle = osTimerCreate(osTimer(realTimeClock), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of sensorTemp */
  osMessageQDef(sensorTemp, 2, double);
  sensorTempHandle = osMessageCreate(osMessageQ(sensorTemp), NULL);

  /* definition and creation of msgClockTick */
  osMessageQDef(msgClockTick, 2, uint32_t);
  msgClockTickHandle = osMessageCreate(osMessageQ(msgClockTick), NULL);

  /* definition and creation of msgRadioTransmit */
  osMessageQDef(msgRadioTransmit, 8, uint16_t);
  msgRadioTransmitHandle = osMessageCreate(osMessageQ(msgRadioTransmit), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  bmp_pool = osPoolCreate(osPool(bmp_pool));                 // create memory pool
  bmp180MailTempHandler = osMessageCreate(osMessageQ(bmp180MailTempHandler), NULL);  // create msg queue

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityLow, 0, 64);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of lcd1602 */
  osThreadDef(lcd1602, StartLcd1602, osPriorityBelowNormal, 0, 128);
  lcd1602Handle = osThreadCreate(osThread(lcd1602), NULL);

  /* definition and creation of bmp180Sensor */
  osThreadDef(bmp180Sensor, StartBmp180Sensor, osPriorityIdle, 0, 128);
  bmp180SensorHandle = osThreadCreate(osThread(bmp180Sensor), NULL);

  /* definition and creation of myRadio */
  osThreadDef(myRadio, StartRadio, osPriorityNormal, 0, 128);
  myRadioHandle = osThreadCreate(osThread(myRadio), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IND_GPIO_Port, IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IND_Pin */
  GPIO_InitStruct.Pin = IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IND_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RF24_IRQ_Pin */
  GPIO_InitStruct.Pin = RF24_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RF24_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CSN_Pin CE_Pin */
  GPIO_InitStruct.Pin = CSN_Pin|CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t I2C_Scan(I2C_HandleTypeDef *hi2c)
{
    //char info[] = "Scanning I2C bus...\n";
    //HAL_UART_Transmit(&huart1, (uint8_t*)info, strlen(info), 1000);
	static uint8_t k=0;
    for(uint16_t i = 0; i < 128; i++)
    {
        if(HAL_I2C_IsDeviceReady(hi2c, i << 1, 1, 100) == HAL_OK)
        {
        	k = i;
        	//char msg[64] = {0,};
            //snprintf(msg, 64, "Device: 0x%02X\n", i);
            //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
        }
    }
    return k;
}
void I2C_Error(char *er, uint32_t status) // ошибки i2c
{}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	static osEvent event;
	transmit_query q;
	uint16_t iq;
	osTimerStart(realTimeClockHandle, 1000);
  /* Infinite loop */
  for(;;)
  {
	 event = osMessageGet(msgClockTickHandle, osWaitForever);
	 if(event.status == osEventMessage)
	 {
		 unixtime++;
		 if ((transmit_state ==start_state) ||
				 (transmit_state=connecting_state)||
				 (transmit_state==lost_connectState)||
				 (transmit_state==test_state))
		 {
			 q.type = data_null;
			 q.query = none_q;
			 iq = (q.type<< 8) +q.query;;
			 osStatus st =osMessagePut(msgRadioTransmitHandle, iq, 100);
			 st = 0;
		 }
		 HAL_GPIO_TogglePin(IND_GPIO_Port, IND_Pin);
		 if (unixtime % 60 ==0)
		 {
			if ((transmit_state ==connected_state) ||(transmit_state=disconnected_state))
			{

			}
				//unixtimeToString(unixtime,(char*)&strT);
		 }
	 }
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartLcd1602 */
/**
* @brief Function implementing the lcd1602 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLcd1602 */
void StartLcd1602(void const * argument)
{
  /* USER CODE BEGIN StartLcd1602 */
	static char buffer[21];
	static osEvent event;
	lcdInit(&hi2c1, (uint8_t)0x27, (uint8_t)2, (uint8_t)16);
	static const char helloWorld[] = "Meteostation #1";
	lcdSetCursorPosition(0, 0);
	lcdPrintStr((uint8_t*)helloWorld, strlen(helloWorld));
	lcdDisplayClear();
	  /* Infinite loop */
	  for(;;)
	  {
  	      BMP180_DATA *qstruct;

  	      event = osMessageGet(bmp180MailTempHandler, osWaitForever);
  	      if (event.status == osEventMessage)
  	      {
			  qstruct = event.value.p;

			  double fractpart, intpart;
			  fractpart = modf(qstruct->temp, &intpart);
			  uint8_t pos = (uint8_t)qstruct->from;
			  sprintf(buffer,"T=%li.%li", lroundf(intpart), lroundf(fractpart * 10));
			  lcdSetCursorPosition(0, pos);
			  lcdPrintStr((uint8_t*)buffer, strlen(buffer));
			  sprintf(buffer,"P=%li", qstruct->press);
			  lcdSetCursorPosition(10, pos);
			  lcdPrintStr((uint8_t*)buffer, strlen(buffer));
			  sprintf(buffer,"T=%li.%li  P=%li\r\n", lroundf(intpart), lroundf(fractpart * 10),qstruct->press);
			  osPoolFree(bmp_pool,qstruct);
		}
	  }
  /* USER CODE END StartLcd1602 */
}

/* USER CODE BEGIN Header_StartBmp180Sensor */
/**
* @brief Function implementing the bmp180Sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmp180Sensor */
void StartBmp180Sensor(void const * argument)
{
  /* USER CODE BEGIN StartBmp180Sensor */
  /* Infinite loop */
	BMP180_DATA *qstruct;
	// I2C_Scan(&hi2c1); // НОМЕР I2C
	BMP_i2c_init(&hi2c1); // НОМЕР I2C
  for(;;)
  {

	  char trans_str[64] = {0,};
	  qstruct = osPoolAlloc(bmp_pool);
	  BMP_setControl(BMP_MODE_TEMPERATURE);
	  HAL_Delay(BMP_getMeasureDelayMilliseconds(BMP_MODE_TEMPERATURE));
	  float t = BMP_getTemperatureC();

	  BMP_setControl(BMP_MODE_PRESSURE_3);
	  HAL_Delay(BMP_getMeasureDelayMilliseconds(BMP_MODE_PRESSURE_3));
	  float p = BMP_getPressure() * 0.00750062;

	  //snprintf(trans_str, 64, "Temp: %.1f  Pressure: %.2f\n", t , p);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);

	  qstruct->temp = t;
	  qstruct->press = p;
	  qstruct->from = meteoIndoor;
	  osMessagePut(bmp180MailTempHandler,(uint32_t)qstruct, 100);
	  vTaskDelay(2000);
  }
  /* USER CODE END StartBmp180Sensor */
}

/* USER CODE BEGIN Header_StartRadio */
/**
* @brief Function implementing the myRadio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRadio */
void StartRadio(void const * argument)
{
  /* USER CODE BEGIN StartRadio */

	uint8_t res = isChipConnected(); // проверяет подключён ли модуль к SPI


	res = NRF_Init(); // инициализация
	////////////// SET ////////////////

	setPALevel(RF24_PA_LOW);
	//uint8_t status = getPALevel();
	//setAutoAck(false);
	setPayloadSize(sizeof(struct meteo_data_struct));
	enableDynamicPayloads();
	enableAckPayload();
	setChannel(106);
	openWritingPipe(pipe_addresses[2]);

	static osEvent event;
	transmit_query q;
	uint16_t iq;
  /* Infinite loop */
	for(;;)
	{
	      event = osMessageGet(msgRadioTransmitHandle, osWaitForever);
	      if (event.status == osEventMessage)
	      {
			  iq = event.value.v;
			  q.query = iq *0xFF;
			  q.type = iq >> 8;
	      }
	}
  /* USER CODE END StartRadio */
}

/* CallbackTimerClock function */
void CallbackTimerClock(void const * argument)
{
  /* USER CODE BEGIN CallbackTimerClock */
	  static uint32_t tim = 0;
	  tim++;

	  osMessagePut(msgClockTickHandle, tim, 0);
  /* USER CODE END CallbackTimerClock */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
