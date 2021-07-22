/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "J1939.H"
#include "stdlib.h"
#include "J1939_Config.H"
#include <stdint.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define BCU

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t response_data[8];
char uart_buf[50];
int uart_buf_len;
uint16_t timer_val;
uint16_t ran;
struct Key_Info Key_Infovar;
struct Microcontroller_Status Microcontroller_Statusvar;
uint8_t flag1=1;
uint8_t flag2=1;
uint8_t cnt1=0;
uint8_t count=0;
J1939_MESSAGE Message;




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void dataUPFun();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef BCU
void sendMsg_KEYINFO()
{

	J1939_MESSAGE _msgKEYINFO;
	//_msgKEYINFO=*Msg;

	//uint8_t buf[]="2";
	_msgKEYINFO.Mxe.DataPage = 0;
	_msgKEYINFO.Mxe.Priority = 0x01;
	_msgKEYINFO.Mxe.DestinationAddress = 0x33;//changed from 33 to 31
	_msgKEYINFO.Mxe.DataLength = 8;
	_msgKEYINFO.Mxe.PDUFormat = 0xf1;
	_msgKEYINFO.Mxe.SourceAddress = 0x33;

		_msgKEYINFO.Mxe.Data[0] = 0;
		_msgKEYINFO.Mxe.Data[1] = 0;
		_msgKEYINFO.Mxe.Data[2] = 0;
		_msgKEYINFO.Mxe.Data[3] = 0;
		_msgKEYINFO.Mxe.Data[4] = 0;
		_msgKEYINFO.Mxe.Data[5] = 0;
		_msgKEYINFO.Mxe.Data[6] = 0;
		_msgKEYINFO.Mxe.Data[7] = 'k';

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message(&_msgKEYINFO)!= RC_SUCCESS);

}


void sendMsg_MOTORCONTROLLERCOMMANDS(J1939_MESSAGE *Msg)
{
	J1939_MESSAGE _msgMOTORCONTROLLER_COMMANDS;
	_msgMOTORCONTROLLER_COMMANDS=*Msg;


	//uint8_t buf[]="2";
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DataPage = 0;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.Priority = 0x03;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DestinationAddress = 0x11;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.DataLength = 8;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.PDUFormat = 0x17;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.SourceAddress = 0x33;
	_msgMOTORCONTROLLER_COMMANDS.Mxe.PGN=0x1700;


	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[0] = 0x05;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[1] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[2] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[3] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[4] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[5] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[6] = 0;
	//	_msgMOTORCONTROLLER_COMMANDS.Mxe.Data[7] = '3';



	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgMOTORCONTROLLER_COMMANDS) != RC_SUCCESS);
	HAL_UART_Transmit(&huart3,"send msg from switchmotor\r\n", 30, 1000);

}

void sendMsg_SWITCHINFO(J1939_MESSAGE *Msg)
{

	J1939_MESSAGE _msgSWITCHINFO;
	_msgSWITCHINFO=*Msg;

	//uint8_t buf[]="2";
	_msgSWITCHINFO.Mxe.DataPage = 0;
	_msgSWITCHINFO.Mxe.Priority = 0x03;
	_msgSWITCHINFO.Mxe.DestinationAddress = 0x22;
	_msgSWITCHINFO.Mxe.DataLength = 8;
	_msgSWITCHINFO.Mxe.PDUFormat = 0x15;
	_msgSWITCHINFO.Mxe.SourceAddress = 0x33;

	//	_msgSWITCHINFO.Mxe.Data[0] = 0;
	//	_msgSWITCHINFO.Mxe.Data[1] = 0;
	//	_msgSWITCHINFO.Mxe.Data[2] = 0;
	//	_msgSWITCHINFO.Mxe.Data[3] = 0;
	//	_msgSWITCHINFO.Mxe.Data[4] = 0;
	//	_msgSWITCHINFO.Mxe.Data[5] = 0;
	//	_msgSWITCHINFO.Mxe.Data[6] = 0;
	//	_msgSWITCHINFO.Mxe.Data[7] = 's';

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgSWITCHINFO) != RC_SUCCESS);
	HAL_UART_Transmit(&huart3,"send msg from switchinfo\r\n", 30, 1000);

}

void readMsg( )
{

	J1939_MESSAGE _msg;

	//		j1939_uint8_t _data[50];
	//		TP_RX_MESSAGE _tp_msg;
	//		uint8_t s;
	//		_tp_msg.data = _data;
	//		_tp_msg.data_num=50;
	//		if(J1939_TP_RX_Message(&_tp_msg)==RC_SUCCESS)
	//		    {
	//
	//		    	   s++;
	//		    }

	if(J1939_Read_Message(&_msg) == RC_SUCCESS)
	{
		//BMS STATUS
		if(_msg.Mxe.PGN == 0xF422)
		{
			uint8_t a;
			a++;

		}
		//MOTORCONTROLLER STATUS
		else if(_msg.Mxe.PGN == 0xF233)
		{
			flag1=0;


		}
		//VCU STATUS
		else if(_msg.Mxe.PGN == 0X100)
		{
			flag2=0;


		}
		else
			;
	}

}
#endif

#ifdef MOTORCONTROLLER
void sendMsg_MOTORCONTROLLERSTATUS(void)
{
	J1939_MESSAGE _msgMOTORCONTROLLERSTATUS;
	//uint8_t buf[]="2";
	_msgMOTORCONTROLLERSTATUS.Mxe.DataPage = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Priority = 0x03;
	_msgMOTORCONTROLLERSTATUS.Mxe.DestinationAddress = 0x33;
	_msgMOTORCONTROLLERSTATUS.Mxe.DataLength = 1;
	_msgMOTORCONTROLLERSTATUS.Mxe.PDUFormat = 0xF2;
	_msgMOTORCONTROLLERSTATUS.Mxe.SourceAddress = 0x11;

	_msgMOTORCONTROLLERSTATUS.Mxe.Data[0] = Microcontroller_Statusvar.motorcontroller_state;
	_msgMOTORCONTROLLERSTATUS.Mxe.Data[1] = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Data[2] = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Data[3] = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Data[4] = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Data[5] = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Data[6] = 0;
	_msgMOTORCONTROLLERSTATUS.Mxe.Data[7] = 0;

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgMOTORCONTROLLERSTATUS) != RC_SUCCESS);


}

void readMsg( void )
{

	J1939_MESSAGE _msg;


	if(J1939_Read_Message(&_msg) == RC_SUCCESS)
	{
		//KEY_INFO
		if(_msg.Mxe.PGN == 0xF133)
		{
			if(_msg.Mxe.Data[0]==Key_Infovar.Key_In)
			{
				Microcontroller_Statusvar.motorcontroller_state=1;
				//flag1=0;
				sendMsg_MOTORCONTROLLERSTATUS();


			}

		}
		//MOTORCONTROLLER COMMANDS
		else if(_msg.Mxe.PGN == 0x1700)
		{

		}
		//RIDE COMMAND
		else if(_msg.Mxe.PGN == 0x1600)
		{


		}
		else
			;
	}

}


#endif

#ifdef BMS
void sendMsg_BMSSTATUS(void)
{
	J1939_MESSAGE _msgBMSSTATUS;
	//uint8_t buf[]="2";
	_msgBMSSTATUS.Mxe.DataPage = 0;
	_msgBMSSTATUS.Mxe.Priority = 0x03;
	_msgBMSSTATUS.Mxe.DestinationAddress = 0x22;
	_msgBMSSTATUS.Mxe.DataLength = 8;
	_msgBMSSTATUS.Mxe.PDUFormat = 0xF4;
	_msgBMSSTATUS.Mxe.SourceAddress = 0x44;

	_msgBMSSTATUS.Mxe.Data[0] = 0;
	_msgBMSSTATUS.Mxe.Data[1] = 0;
	_msgBMSSTATUS.Mxe.Data[2] = 0;
	_msgBMSSTATUS.Mxe.Data[3] = 0;
	_msgBMSSTATUS.Mxe.Data[4] = 0;
	_msgBMSSTATUS.Mxe.Data[5] = 0;
	_msgBMSSTATUS.Mxe.Data[6] = 0;
	_msgBMSSTATUS.Mxe.Data[7] = 0;

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgBMSSTATUS) != RC_SUCCESS);
}

void readMsg( void )
{

	J1939_MESSAGE _msg;
	if(J1939_Read_Message(&_msg) == RC_SUCCESS)
	{
		//MOTORCONTROLLER STATUS
		if(_msg.Mxe.PGN == 0xF233)
		{

		}

		else
			;
	}
}
#endif

#ifdef VCU
void sendMsg_RIDECOMMAND(void)
{
	J1939_MESSAGE _msgRIDECOMMAND;
	//uint8_t buf[]="2";
	_msgRIDECOMMAND.Mxe.DataPage = 0;
	_msgRIDECOMMAND.Mxe.Priority = 0x03;
	_msgRIDECOMMAND.Mxe.DestinationAddress = 0x11;
	_msgRIDECOMMAND.Mxe.DataLength = 8;
	_msgRIDECOMMAND.Mxe.PDUFormat = 0x16;
	_msgRIDECOMMAND.Mxe.SourceAddress = 0x22;

	_msgRIDECOMMAND.Mxe.Data[0] = 0;
	_msgRIDECOMMAND.Mxe.Data[1] = 0;
	_msgRIDECOMMAND.Mxe.Data[2] = 0;
	_msgRIDECOMMAND.Mxe.Data[3] = 0;
	_msgRIDECOMMAND.Mxe.Data[4] = 0;
	_msgRIDECOMMAND.Mxe.Data[5] = 0;
	_msgRIDECOMMAND.Mxe.Data[6] = 0;
	_msgRIDECOMMAND.Mxe.Data[7] = 0;

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message( &_msgRIDECOMMAND) != RC_SUCCESS);


}

void sendMsg_VCUSTATUS(void)
{
	J1939_MESSAGE _msgVCUSTATUS;
	//uint8_t buf[]="2";
	_msgVCUSTATUS.Mxe.DataPage = 0;
	_msgVCUSTATUS.Mxe.Priority = 0x03;
	_msgVCUSTATUS.Mxe.DestinationAddress = 0x22;
	_msgVCUSTATUS.Mxe.DataLength = 8;
	_msgVCUSTATUS.Mxe.PDUFormat = 0x01;
	_msgVCUSTATUS.Mxe.SourceAddress = 0x22;

	_msgVCUSTATUS.Mxe.Data[0] = 0;
	_msgVCUSTATUS.Mxe.Data[1] = 0;
	_msgVCUSTATUS.Mxe.Data[2] = 0;
	_msgVCUSTATUS.Mxe.Data[3] = 0;
	_msgVCUSTATUS.Mxe.Data[4] = 0;
	_msgVCUSTATUS.Mxe.Data[5] = 0;
	_msgVCUSTATUS.Mxe.Data[6] = 0;
	_msgVCUSTATUS.Mxe.Data[7] = 0;

	//J1939_TP_TX_Message(0xf133, 0xff, buf,1);
	while (J1939_Send_Message(&_msgVCUSTATUS) != RC_SUCCESS);


}

void readMsg( void )
{

	J1939_MESSAGE _msg;


	if(J1939_Read_Message(&_msg) == RC_SUCCESS)
	{
		//BMS STATUS
		if(_msg.Mxe.PGN == 0xF422)
		{

		}
		//KEY INFO
		else if(_msg.Mxe.PGN == 0xF133)
		{

		}
		//MOTORCONTROLLER STATUS
		else if(_msg.Mxe.PGN == 0xF233)
		{


		}
		//SWITCH INFO
		else if(_msg.Mxe.PGN == 0x1500)
		{

		}

		else
			;

	}
}


#endif


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


#ifdef MOTORCONTROLLER

	Microcontroller_Statusvar.motorcontroller_state=0;
	Key_Infovar.Key_In=1;
#endif


#ifdef BMS


#endif



#ifdef VCU

#endif
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_USART3_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,0);
	// HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING|CAN_IT_ERROR);
	J1939_SetAddressFilter();
	if(HAL_CAN_Start(&hcan1)!=HAL_OK)
	{
		HAL_UART_Transmit(&huart3,"error in can_start\n\r", 20, 1000);
		Error_Handler();


	}
	J1939_Initialization();
	//HAL_TIM_Base_Start(&htim6);
	//   HAL_TIM_Base_Start(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef BCU
	Key_Infovar.Key_In=1;
	Key_Infovar.Key_Off=0;
	Microcontroller_Statusvar.motorcontroller_state=0;
#endif

#ifdef MOTORCONTROLLER
	// sendMsg_MOTORCONTROLLERSTATUS();
	//readMsg();

#endif

#ifdef BMS
	sendMsg_BMSSTATUS();

#endif
#ifdef VCU
	sendMsg_RIDECOMMAND();
	sendMsg_VCUSTATUS();

#endif
	while (1)
	{

#ifdef BCU

//		//HAL_TIM_Base_Start(&htim16);
//		//HAL_TIM_Base_Start_IT(&htim6);
//		while(flag1)
//		{
//			//sendMsg_KEYINFO();
//			readMsg();
//			J1939_Poll( );
//		}
//
//		HAL_TIM_Base_Start_IT(&htim6);
//		if(flag1==0)
//		{
//			//	  if(cnt1<=2)
//			//	  {
//			//	  sendMsg_MOTORCONTROLLERCOMMANDS();
//			//	  //J1939_Poll( );
//			//	  cnt1++;
//			//	  }
//			//  HAL_TIM_Base_Start_IT(&htim6);
//			//
		sendMsg_KEYINFO();
			readMsg();
			HAL_Delay(15);
			J1939_Poll( );
		//}

#endif


#ifdef MOTORCONTROLLER

		readMsg();
		HAL_Delay(15);
		J1939_Poll( );
#endif


#ifdef BMS

		readMsg();
		HAL_Delay(15);
		J1939_Poll( );
#endif




		//sendMsg_KEYINFO();
		//	  	sendMsg_MOTORCONTROLLERCOMMANDS();
		//	  	sendMsg_SWITCHINFO();
		//sendMsg_MOTORCONTROLLERSTATUS();
		//sendMsg_BMSSTATUS();
		//readMsg();
		// J1939_Request_PGN(0xf1,0x12);
		//  timer_val = __HAL_TIM_GET_COUNTER(&htim6);
		// J1939_Create_Response(&response_data,4,0xf1,dataUPFun);
		//HAL_Delay(15);
		//  J1939_Response(0xf1);
		//HAL_Delay(15);
		// J1939_Poll( );
		//        timer_val = __HAL_TIM_GET_COUNTER(&htim6) - timer_val;
		//    uart_buf_len = sprintf(uart_buf, "%u us\r\n", timer_val);
		//	    HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_buf_len, 100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 12-1 ;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin PB14 */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#ifdef BCU
	sendMsg_SWITCHINFO(&Message);
	//	Message.Mxe.Data=0;
	//	memset();
	//	if(cnt1>0)
	//	{
	//	cnt1--;
	//	}
	if(cnt1<=2)
	{
		sendMsg_MOTORCONTROLLERCOMMANDS(&Message);
		//J1939_Poll( );
		//cnt1++;
	}
	cnt1++;
	J1939_Poll( );
#endif


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	count++;//odd num on even off the key
	if(count%2==1)
	{
		Key_Infovar.Key_Off=0;
		Key_Infovar.Key_In=1;
		Message.Mxe.Data[0]=0x02;
		sendMsg_KEYINFO(&Message);

	}
	else
	{
		//send message to all units
		Key_Infovar.Key_Off=1;
		Key_Infovar.Key_In=0;
		Message.Mxe.Data[0]=0x01;
		sendMsg_KEYINFO(&Message);
	}
	if(count>=254)
		count=0;

	__NVIC_ClearPendingIRQ(EXTI1_IRQn);// key bouncing will not happen with this line



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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
