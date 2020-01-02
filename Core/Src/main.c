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
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "lwip/api.h"
#include "task.h"

#include <stdio.h>
#include <string.h>
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
SPI_HandleTypeDef hspi1;

osThreadId_t defaultTaskHandle;
osThreadId_t networkTaskHandle;
osThreadId_t accTaskHandle;
/* USER CODE BEGIN PV */

// Generic
signed char* overflowTaskName;

// Accelerometer
AccData accData;
uint8_t accDataRdyFlag=0;

// Network
extern struct netif gnetif;
struct netconn* conn;
IpAddr serverIp = { 192, 168, 1, 3 };
ip_addr_t serverAddr;
u16_t serverPort = 3000;
err_t serverConnStatus = -1;
/*char requestData[] = "POST /location HTTP/1.0\r\n\
Host: 192.168.1.3:3000\r\n\
Content-Type: application/json\r\n\
Content-Length: 25\r\n\
\r\n\
{\"lat\":12.12,\"lon\":13.13}\r\n\
\r\n";
size_t requestDataSize = sizeof(requestData);*/
int startSendingLocation = 0;
char jsonString[50];
char requestString[150];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartNetworkTask(void *argument);
void StartAccTask(void *argument);

/* USER CODE BEGIN PFP */

void createHttpLocationRequest(char* outputString, IpAddr ip, u16_t port, GpsLocation location);
void parseDouble(char* output, double value, uint8_t precision);

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

  IP_ADDR4(&serverAddr, serverIp.addr0, serverIp.addr1, serverIp.addr2, serverIp.addr3);

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  // Accelerometer initialization
  /*LIS3DSH_InitTypeDef myAccConfigDef;
  myAccConfigDef.dataRate = LIS3DSH_DATARATE_25;
  myAccConfigDef.fullScale = LIS3DSH_FULLSCALE_4;
  myAccConfigDef.antiAliasingBW = LIS3DSH_FILTER_BW_50;
  myAccConfigDef.enableAxes = LIS3DSH_XYZ_ENABLE;
  myAccConfigDef.interruptEnable = true;
  LIS3DSH_Init(&hspi1, &myAccConfigDef);*/

  /* USER CODE END 2 */

  osKernelInitialize();

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
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 512
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of networkTask */
  const osThreadAttr_t networkTask_attributes = {
    .name = "networkTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 1024
  };
  networkTaskHandle = osThreadNew(StartNetworkTask, NULL, &networkTask_attributes);

  /* definition and creation of accTask */
  const osThreadAttr_t accTask_attributes = {
    .name = "accTask",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 256
  };
  accTaskHandle = osThreadNew(StartAccTask, NULL, &accTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  /*const osThreadAttr_t accTask_attributes = {
      .name = "accTask",
      .priority = (osPriority_t) osPriorityNormal,
      .stack_size = 128
    };
  accTaskHandle = osThreadNew(StartAccTask, NULL, &accTask_attributes);*/

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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	accDataRdyFlag = 1;
	HAL_GPIO_TogglePin(GPIOD, LED_ORANGE);
	//HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}*/

void parseDouble(char* output, double value, uint8_t precision) {
	int integerPart = (int)value;
	float floatingPointPart = (value - integerPart) * pow(10, precision);
	sprintf(output, "%d.%d", (int)integerPart, (int)floatingPointPart);
}

void createHttpLocationRequest(char* outputString, IpAddr ip, u16_t port, GpsLocation location) {
	char ipString[15];
	sprintf(ipString, "%d.%d.%d.%d", ip.addr0, ip.addr1, ip.addr2, ip.addr3);

	char jsonFormat[] = "{\"lat\":%s,\"lon\":%s}";
	char latString[10];
	parseDouble(latString, location.lat, 4);
	char lonString[10];
	parseDouble(lonString, location.lon, 4);
	sprintf(jsonString, jsonFormat, latString, lonString);

	char requestFormat[] = "POST /location HTTP/1.0\r\n\
Host: %s:%d\r\n\
Content-Type: application/json\r\n\
Content-Length: %d\r\n\
\r\n\
%s\r\n\
\r\n";

	sprintf(outputString, requestFormat, ipString, port, strlen(jsonString), jsonString);
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	overflowTaskName = pcTaskName;
	HAL_GPIO_WritePin(GPIOD, LED_RED, 1);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for(;;)
  {
	  /* Async init code */


	  /* LED indicators */

	  // Check if DHCP got an addr from router
	  if (gnetif.ip_addr.addr != 0) {
		  startSendingLocation = 1;
		  HAL_GPIO_WritePin(GPIOD, LED_GREEN, 1);
	  }

	  osDelay(50);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartNetworkTask */
/**
* @brief Function implementing the networkTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNetworkTask */
void StartNetworkTask(void *argument)
{
  /* USER CODE BEGIN StartNetworkTask */
  /* Infinite loop */
  for(;;)
  {
	  if (startSendingLocation) {
		conn = netconn_new(NETCONN_TCP);
		serverConnStatus = netconn_connect(conn, &serverAddr, serverPort);
		if (serverConnStatus == ERR_OK) {
		  GpsLocation location = { 12.12, 13.13 };
		  createHttpLocationRequest(requestString, serverIp, serverPort, location);
		  netconn_write(conn, requestString, strlen(requestString), NETCONN_NOFLAG);
		}

		netconn_delete(conn);

		HAL_GPIO_TogglePin(GPIOD, LED_BLUE);
	  }

	  osDelay(1000);
  }
  /* USER CODE END StartNetworkTask */
}

/* USER CODE BEGIN Header_StartAccTask */
/**
* @brief Function implementing the accTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccTask */
void StartAccTask(void *argument)
{
  /* USER CODE BEGIN StartAccTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(50);
  }
  /* USER CODE END StartAccTask */
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
