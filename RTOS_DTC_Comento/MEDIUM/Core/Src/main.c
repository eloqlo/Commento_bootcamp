/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>
#include "EEPROM.h"
#include "PMIC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint16_t DTC_SAE;		// DTC SAE표준 (제동이므로 샤시 C)
	uint8_t DTC_FTB;		// DTC Failure Type byte
	uint8_t DTC_Status;		// DTC Status byte

	uint8_t active;
	char Description[50];
} DTC_Table_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// EEPROM CMD
#define EEPROM_CMD_WREN 0x06
#define EEPROM_CMD_WRITE 0x02
#define EEPROM_CMD_READ 0x03

// EEPROM DTC Addresses
#define EEPROM_DTC_ADDR_UV_A 0x0000
#define EEPROM_DTC_ADDR_OV_A ((uint16_t)(0x0000 + 55*1))
#define EEPROM_DTC_ADDR_OC_A ((uint16_t)(0x0000 + 55*2))

#define EEPROM_DTC_ADDR_UV_B ((uint16_t)(0x0000 + 55*3))
#define EEPROM_DTC_ADDR_OV_B ((uint16_t)(0x0000 + 55*4))
#define EEPROM_DTC_ADDR_OC_B ((uint16_t)(0x0000 + 55*5))

#define EEPROM_DTC_ADDR_UV_C ((uint16_t)(0x0000 + 55*6))
#define EEPROM_DTC_ADDR_OV_C ((uint16_t)(0x0000 + 55*7))
#define EEPROM_DTC_ADDR_OC_C ((uint16_t)(0x0000 + 55*8))

#define EEPROM_DTC_ADDR_UV_D ((uint16_t)(0x0000 + 55*9))
#define EEPROM_DTC_ADDR_OV_D ((uint16_t)(0x0000 + 55*10))
#define EEPROM_DTC_ADDR_OC_D ((uint16_t)(0x0000 + 55*11))

#define EEPROM_DTC_ADDR_TEMP ((uint16_t)(0x0000 + 55*12))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart4;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2CTask */
osThreadId_t I2CTaskHandle;
const osThreadAttr_t I2CTask_attributes = {
  .name = "I2CTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SPITask */
osThreadId_t SPITaskHandle;
const osThreadAttr_t SPITask_attributes = {
  .name = "SPITask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CANTask */
osThreadId_t CANTaskHandle;
const osThreadAttr_t CANTask_attributes = {
  .name = "CANTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for UARTTask */
osThreadId_t UARTTaskHandle;
const osThreadAttr_t UARTTask_attributes = {
  .name = "UARTTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CanQueue */
osMessageQueueId_t CanQueueHandle;
const osMessageQueueAttr_t CanQueue_attributes = {
  .name = "CanQueue"
};
/* Definitions for CommMutexHandle */
osMutexId_t CommMutexHandleHandle;
const osMutexAttr_t CommMutexHandle_attributes = {
  .name = "CommMutexHandle"
};


/* USER CODE BEGIN PV */
DTC_Table_t DTC_Table_UV_A = { 0x0000, "Brake BuckA Under Voltage Fault", 0};
DTC_Table_t DTC_Table_OV_A = { 0x0001, "Brake BuckA Over Voltage Fault", 0};
DTC_Table_t DTC_Table_OC_A = { 0x0002, "Brake BuckA Over Current Fault", 0};

DTC_Table_t DTC_Table_UV_B = { 0x0003, "Brake BuckB Under Voltage Fault", 0};
DTC_Table_t DTC_Table_OV_B = { 0x0004, "Brake BuckB Over Voltage Fault", 0};
DTC_Table_t DTC_Table_OC_B = { 0x0005, "Brake BuckB Over Current Fault", 0};

DTC_Table_t DTC_Table_UV_C = { 0x0006, "Brake BuckC Under Voltage Fault", 0};
DTC_Table_t DTC_Table_OV_C = { 0x0007, "Brake BuckC Over Voltage Fault", 0};
DTC_Table_t DTC_Table_OC_C = { 0x0008, "Brake BuckC Over Current Fault", 0};

DTC_Table_t DTC_Table_UV_D = { 0x0006, "Brake BuckC Under Voltage Fault", 0};
DTC_Table_t DTC_Table_OV_D = { 0x0007, "Brake BuckC Over Voltage Fault", 0};
DTC_Table_t DTC_Table_OC_D = { 0x0008, "Brake BuckC Over Current Fault", 0};
DTC_Table_t DTC_Table_TEMP = { 0x0009, "Brake High Temperature Fault", 0};


uint8_t pmic_read_fault_flag = 1;
uint8_t eeprom_read_dtc_flag = 0;
uint8_t can_transmit_dtc_flag = 0;
uint8_t uart_transmit_flag = 0;

uint8_t CAN_Counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART4_Init(void);
void StartDefaultTask(void *argument);
void StartI2CTask(void *argument);
void StartSPITask(void *argument);
void StartCANTask(void *argument);
void StartUARTTask(void *argument);
uint8_t IsDTCActive(void);
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
  MX_ADC1_Init();
  MX_CAN1_Init();		// HAL_CAN_INIT() 수행.
  MX_I2C1_Init();
//  MX_I2C2_Init();
  MX_SPI1_Init();
//  MX_SPI2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  PMIC_Init();			// PMIC 전원 켜기 + PMIC 벅 켜기 + 벅 레귤레이터 출력전압 컨트롤

  EEPROM_ReadDTC_DMA(DTC_Table_UV_A, EEPROM_DTC_ADDR_UV_A);		// 부팅 이전에 저장된 DTC 정보 복구
  EEPROM_ReadDTC_DMA(DTC_Table_UV_B, EEPROM_DTC_ADDR_UV_B);
  EEPROM_ReadDTC_DMA(DTC_Table_UV_C, EEPROM_DTC_ADDR_UV_C);
  EEPROM_ReadDTC_DMA(DTC_Table_UV_D, EEPROM_DTC_ADDR_UV_D);
  EEPROM_ReadDTC_DMA(DTC_Table_OV_A, EEPROM_DTC_ADDR_OV_A);
  EEPROM_ReadDTC_DMA(DTC_Table_OV_B, EEPROM_DTC_ADDR_OV_B);
  EEPROM_ReadDTC_DMA(DTC_Table_OV_C, EEPROM_DTC_ADDR_OV_C);
  EEPROM_ReadDTC_DMA(DTC_Table_OV_D, EEPROM_DTC_ADDR_OV_D);
  EEPROM_ReadDTC_DMA(DTC_Table_OC_A, EEPROM_DTC_ADDR_OC_A);
  EEPROM_ReadDTC_DMA(DTC_Table_OC_B, EEPROM_DTC_ADDR_OC_B);
  EEPROM_ReadDTC_DMA(DTC_Table_OC_C, EEPROM_DTC_ADDR_OC_C);
  EEPROM_ReadDTC_DMA(DTC_Table_OC_D, EEPROM_DTC_ADDR_OC_D);
  EEPROM_ReadDTC_DMA(DTC_Table_TEMP, EEPROM_DTC_ADDR_TEMP);

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);	// Tx Mailbox가 비어서 새로운 전송 가능할 때 IRQ !

  can_transmit_dtc_flag = 0;
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of CommMutexHandle */
  CommMutexHandleHandle = osMutexNew(&CommMutexHandle_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of CanQueue */
  CanQueueHandle = osMessageQueueNew (8, 8, &CanQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /*
   * osThreadNew() 를 통해, 새로운 TASK를 생성한다.
   * osThreadNew() 의 매개변수는 [실행할 TASK loop], [?], [(이름/스택크기/우선순위) 구조체 주소] 이다.
   *  */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of I2CTask */
  I2CTaskHandle = osThreadNew(StartI2CTask, NULL, &I2CTask_attributes);

  /* creation of SPITask */
  SPITaskHandle = osThreadNew(StartSPITask, NULL, &SPITask_attributes);

  /* creation of CANTask */
  CANTaskHandle = osThreadNew(StartCANTask, NULL, &CANTask_attributes);

  /* creation of UARTTask */
  UARTTaskHandle = osThreadNew(StartUARTTask, NULL, &UARTTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();		// 스케쥴러 시작.

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE END WHILE */
	  /* USER CODE BEGIN 3 */

	  // [1] PMIC 레지스터를 읽어오고, 이에 맞춰 DTC를 업데이트 해준다.
	  if (pmic_read_fault_flag == 1) {
		  pmic_read_fault_flag = 0;

		  // OC, OV, UV, TEMP 레지스터의 Fault Data 를 읽어온다.
		  PMIC_ReadUVOV();
		  PMIC_ReadOC();
		  PMIC_ReadTEMP();

		  // Fault 값이 바뀌었으면 DTC를 업데이트한다. 이후 EEPROM에 DTC를 저장.
		  PMIC_DTC_Update();
	  }

	  // [2] EEPROM에 저장된 DTC를 DMA로 다시 읽어온다.
	  if (eeprom_read_dtc_flag == 1){
		  eeprom_read_dtc_flag = 0;
		  EEPROM_ReadDTC_DMA(DTC_Table_UV_A, EEPROM_DTC_ADDR_UV_A);
		  EEPROM_ReadDTC_DMA(DTC_Table_UV_B, EEPROM_DTC_ADDR_UV_B);
		  EEPROM_ReadDTC_DMA(DTC_Table_UV_C, EEPROM_DTC_ADDR_UV_C);
		  EEPROM_ReadDTC_DMA(DTC_Table_UV_D, EEPROM_DTC_ADDR_UV_D);
		  EEPROM_ReadDTC_DMA(DTC_Table_OV_A, EEPROM_DTC_ADDR_OV_A);
		  EEPROM_ReadDTC_DMA(DTC_Table_OV_B, EEPROM_DTC_ADDR_OV_B);
		  EEPROM_ReadDTC_DMA(DTC_Table_OV_C, EEPROM_DTC_ADDR_OV_C);
		  EEPROM_ReadDTC_DMA(DTC_Table_OV_D, EEPROM_DTC_ADDR_OV_D);
		  EEPROM_ReadDTC_DMA(DTC_Table_OC_A, EEPROM_DTC_ADDR_OC_A);
		  EEPROM_ReadDTC_DMA(DTC_Table_OC_B, EEPROM_DTC_ADDR_OC_B);
		  EEPROM_ReadDTC_DMA(DTC_Table_OC_C, EEPROM_DTC_ADDR_OC_C);
		  EEPROM_ReadDTC_DMA(DTC_Table_OC_D, EEPROM_DTC_ADDR_OC_D);
		  EEPROM_ReadDTC_DMA(DTC_Table_TEMP, EEPROM_DTC_ADDR_TEMP);
	  }

	  // [3] CAN interrupt DTC 송출
	  	  if (can_transmit_dtc_flag == 1){
	  		  can_transmit_dtc_flag = 0;
	  		  CAN_Send_DTC_Table(DTC_Table_UV_A);
	  		  CAN_Send_DTC_Table(DTC_Table_UV_B);
	  		  CAN_Send_DTC_Table(DTC_Table_UV_C);
	  		  CAN_Send_DTC_Table(DTC_Table_UV_D);
	  		  CAN_Send_DTC_Table(DTC_Table_OV_A);
	  		  CAN_Send_DTC_Table(DTC_Table_OV_B);
	  		  CAN_Send_DTC_Table(DTC_Table_OV_C);
	  		  CAN_Send_DTC_Table(DTC_Table_OV_D);
	  		  CAN_Send_DTC_Table(DTC_Table_OC_A);
	  		  CAN_Send_DTC_Table(DTC_Table_OC_B);
	  		  CAN_Send_DTC_Table(DTC_Table_OC_C);
	  		  CAN_Send_DTC_Table(DTC_Table_OC_D);
	  		  CAN_Send_DTC_Table(DTC_Table_TEMP);
	  	  }
	  	  if (CAN_Counter == 13){
	  		  uart_transmit_flag = 1;
	  		  CAN_Counter = 0;
	  	  }

	  // [4] UART 매 주기 시스템 동작 이상 없음 송출
	  if (uart_transmit_flag == 1){
		  uart_transmit_flag = 0;
		  char uart_comment[50];
		  if (IsDTCActive()) {
			  snprintf(uart_comment, sizeof(uart_comment), "Fault Detected in Brake System");
		  }
		  else {
			  snprintf(uart_comment, sizeof(uart_comment), "No Fault Detected in Brake System");
		  }
		  HAL_UART_Transmit(&huart4, uart_comment, 50, HAL_MAX_DELAY);
	  }
	  /* USER CODE END 3 */
  }
}

/* NEW 25.08.24 - CAN으로 DTC data를 송출 */
void CAN_Send_DTC_Table(DTC_Table_t DTC_Table){
	CAN_TxHeaderTypeDef canTxHeader;
	uint32_t TxMailbox;
	uint8_t TxData[8] = {0};

	canTxHeader.StdId = 0x7E8;				// 응답 ID
	canTxHeader.IDE = CAN_ID_STD;			// 11bit ID
	canTxHeader.RTR = CAN_RTR_DATA;			// data frame
	canTxHeader.DLC = 8;					// Data 길이: 8bytes

	// Send DTC
	TxData[0] = 0x06;	// UDS 데이터 길이
	TxData[1] = 0x22;	// RDBI + Positive Response
	TxData[2] = (DTC_Table.DTC_SAE >> 8) & 0xFF;	// DTC SAE High
	TxData[3] = DTC_Table.DTC_SAE & 0xFF;	// DTC SAE Low
	TxData[4] = DTC_Table.DTC_FTB;
	TxData[5] = DTC_Table.DTC_Status;

	HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, TxData, &TxMailbox);
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Counter++;
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Counter++;
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Counter++;
}

uint8_t IsDTCActive(void){
	return DTC_Table_UV_A.active | DTC_Table_UV_B.active | DTC_Table_UV_C.active | DTC_Table_UV_D.active |
			  DTC_Table_OV_A.active | DTC_Table_OV_B.active | DTC_Table_OV_C.active | DTC_Table_OV_D.active |
			  DTC_Table_OC_A.active | DTC_Table_OC_B.active | DTC_Table_OC_C.active | DTC_Table_OC_D.active |
			  DTC_Table_TEMP.active;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// --- CAN 수신 인터럽트 콜백 ---



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
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartI2CTask */
/**
* @brief Function implementing the I2CTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartI2CTask */
void StartI2CTask(void *argument)
{
	/* USER CODE BEGIN StartI2CTask */
	/* Infinite loop */
	uint8_t faultReg;
	for(;;)
	{
		osMutexAcquire(CommMutexHandleHandle, osWaitForever);
		HAL_I2C_Mem_Read(&hi2c1, PMIC_DEV_ADDR, PMIC_FAULT_STATUS1_REG, I2C_MEMADD_SIZE_8BIT, &faultReg, 1, HAL_MAX_DELAY);
		if (faultReg & 0x01) {
			if (DTC_Table.active == 0) {
				DTC_Table.active = 1;
//				EEPROM_WriteDTC_DMA(DTC_Table, mem_addr);
			}
		}
		osMutexRelease(CommMutexHandleHandle);
		osDelay(500);
	}
	/* USER CODE END StartI2CTask */
}

/* USER CODE BEGIN Header_StartSPITask */
/**
* @brief Function implementing the SPITask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSPITask */
void StartSPITask(void *argument)
{
  /* USER CODE BEGIN StartSPITask */
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcqutire(CommMutexHandleHandle, osWaitForever);
//	  EEPROM_WriteDTC_DMA(DTC_Table, mem_addr)();
	  osMutexRelease(CommMutexHandleHandle);
	  osDelay(5000);
  }
  /* USER CODE END StartSPITask */
}

/* USER CODE BEGIN Header_StartCANTask */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCANTask */
void StartCANTask(void *argument)
{
  /* USER CODE BEGIN StartCANTask */
  /* Infinite loop */
  uint8_t rxBuf[8];
  for(;;)
  {
    if (osMessageQueueGet(CanQueueHandle, rxBuf, NULL, osWaitForever) == osOK) {
    	osMutexAcquire(CommMutexHandleHandle, osWaitForever);
    	Process_CAN_Respopnse(rxBuf);
    	osMutexRelease(CommMutexHandleHandle);
    }
    osDelay(100);
  }
  /* USER CODE END StartCANTask */
}

/* USER CODE BEGIN Header_StartUARTTask */
/**
* @brief Function implementing the UARTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUARTTask */
void StartUARTTask(void *argument)
{
  /* USER CODE BEGIN StartUARTTask */
  /* Infinite loop */
	const char msg[] = "ECU System Running\r\n";
	for(;;) {
		osMutexAcquire(CommMutexHandleHandle, osWaitForever);
		HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		osMutexRelease(CommMutexHandleHandle);
		osDelay(1000);
  /* USER CODE END StartUARTTask */
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


CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO, &RxHeader, RxData);
	osMessageQueuePut(CanQueueHandle, RxData, 0, 0);
}

void Process_CAN_Response(uint8_t *data, DTC_Table_t DTC_Table, uint16_t mem_addr){
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint8_t TxData[8] = {0};

	TxHeader.StdId = 0x7E8;		// 응답 ID
	TxHeader.IDE = CAN_ID_STD;	// msg ID
	TxHeader.RTR = CAN_RTR_DATA;	// 메시지 Frame
	TxHeader.DLC = 8;

	// OBD2 0x43 : Read DTCs
	if (data[1] == 0x43) {
		if (DTC_Table.active) {
			TxData[0] = 0x03;
			TxData[1] = 0x43;
			TxData[2] = (DTC_Table.DTC_Code >> 8) & 0xFF;
			TxData[3] = DTC_Table.DTC_Code & 0xFF;
		} else {
			TxData[0] = 0x01;
			TxData[1] = 0x43;
			TxData[2] = 0x00;
		}
	}
	// OBD2 0x04 : Clear DTCs
	else if (data[1] == 0x04) {
		DTC_Table.active = 0;
		EEPROM_WriteDTC_DMA(DTC_Table, mem_addr);
		TxData[0] = 0x01;
		TxData[1] = 0x44;	// 응답
	}
	// UDS 0x19 : Read DTCs
	else if (data[1] == 0x19) {
		if (DTC_Table.active){
			TxData[0] = 0x03;
			TxData[1] = 0x59;
			TxData[2] = 0x02;
			TxData[3] = (DTC_Table.DTC_Code >> 8) & 0xFF;
			TxData[4] = DTC_Table.DTC_Code & 0xFF;
		} else {
			TxData[0] = 0x01;
			TxData[1] = 0x59;
			TxData[2] = 0x00;
		}
	}
	else if (data[1] == 0x14) {
		DTC_Table.active = 0;
		EEPROM_WriteDTC_DMA(DTC_Table, mem_addr);
		TxData[0] = 0x02;
		TxData[1] = 0x54;	// 응답
	}
	else{
		/* for misra code */
	}

	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
