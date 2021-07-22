#include "fun.h"

#include "J1939.H"

#include "J1939_Config.H"


void can_fil_config()
{
		fil.FilterIdHigh=0x0000;
	  	fil.FilterIdLow=0x0000;
	  	fil.FilterMaskIdHigh=0x0000;
	  	fil.FilterMaskIdLow=0x0000;
	  	fil.FilterFIFOAssignment=0;
	  	fil.FilterBank=0;
	  	fil.FilterMode=CAN_FILTERMODE_IDMASK;
	  	fil.FilterScale=CAN_FILTERSCALE_32BIT;
	  	fil.FilterActivation=CAN_FILTER_ENABLE;
	  	fil.SlaveStartFilterBank=0;

	  	if (HAL_CAN_ConfigFilter(&hcan1, &fil) != HAL_OK)
	  		    {
	  		        /* Filter configuration Error */
	  		        HAL_UART_Transmit(&huart3,"error in configfil\n\r", 20, 1000);
	  		        Error_Handler();
	  		    }
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t mssg[25];

	HAL_UART_Transmit(&huart3, "txcalbckM0\n\r", 25, 1000);
	memset(mssg,0,25);


}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)

{
	uint8_t mssg[25];

		HAL_UART_Transmit(&huart3, "txcalbckM1\n\r", 25, 1000);
		memset(mssg,0,25);

}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t mssg[25];

		HAL_UART_Transmit(&huart3, "txcalbckM2\n\r", 25, 1000);
		memset(mssg,0,25);

}

//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	HAL_ResumeTick();
//
//    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&rxhead, arr) != HAL_OK)
//    {
//
//    	HAL_UART_Transmit(&huart3,"error in reception\n\r", 20, 1000);
//    	Error_Handler();
//    }
//
///*
//   switch(rxhead.StdId)
//    {
//    case 69 :
//    	{
//
//    		HAL_UART_Transmit(&huart3,"STDID 45", strlen("STDID 45"), 1000);
//    		//HAL_UART_Transmit(&huart3,"ID RELATED TO POWER AND SPEED \n\r",strlen("ID RELATED TO POWER\n\r"), 1000);
//
//
//    		value1=((arr[0])|(arr[1]<<8)|(arr[2]<<16)|(arr[3]<<24));
//    		value2=((arr[4])|(arr[5]<<8)|(arr[6]<<16)|(arr[7]<<24));
//    		data1=((arr[0]&(0xF0))|arr[1]<<8|(((arr[2])&(0x0F))<<16));
//    		data2=((arr[1]&(0xF0))|arr[2]<<8|(((arr[3])&(0x0F))<<16));
//
//    	    break;
//    	}
//    case 70:
//    {
//    	HAL_UART_Transmit(&huart3,"STDID 46", strlen("STDID 45"), 1000);
//    	//HAL_UART_Transmit(&huart3,"ID RELATED TO TEMP, \n\r",strlen("ID RELATED TO SPEED\n\r"), 1000);
//
//    	value1=((arr[0])|(arr[1]<<8));
//    	value2=((arr[2])|(arr[3]<<8));
//    	value3=((arr[4])|(arr[5]<<8));
//    	value4=((arr[6])|(arr[7]<<8));
//    	data1=((arr[2]&(0xF0))|arr[3]<<8|(((arr[4])&(0x0F))<<16));
//    	data2=((arr[5]&(0xF0))|arr[6]<<8|(((arr[7])&(0x0F))<<16));
//
//  		break;
//    }
//    case 71:
//       {
//       	HAL_UART_Transmit(&huart3,"STDID 47", strlen("STDID 47"), 1000);
//       	//HAL_UART_Transmit(&huart3,"ID RELATED TO CURRENT\n\r",strlen("ID RELATED TO CURRENT\n\r"), 1000);
//
//       	value1=((arr[0])|(arr[3]<<8)|(arr[6]<<16));
//       	value2=((arr[1])|(arr[4]<<8)|(arr[7]<<16));
//       	value3=((arr[2])|(arr[5]<<8));
//       	data1=(arr[2]&(0xF0));
//       	data2=(arr[6]&(0xF0))|(((arr[7])&(0x0F))<<16);
//       	data3=((arr[5]&(0xF0))|arr[6]<<8|(((arr[7])&(0x0F))<<16));
//
//
//       	break;
//       }
//
//    case 72:
//     {
//     	HAL_UART_Transmit(&huart3,"STDID 48", strlen("STDID 48"), 1000);
//     	//HAL_UART_Transmit(&huart3,"ID RELATED TO VOLTAGE\n\r",strlen("ID RELATED TO VOLTAGE\n\r"), 1000);
//
//     	value1=((arr[0])|(arr[4]<<8));
//     	value2=((arr[1])|(arr[5]<<8));
//       	value3=((arr[2])|(arr[6]<<8));
//       	value4=((arr[3])|(arr[7]<<8));
//
//     	break;
//     }
//    case 73:
//     {
//     	HAL_UART_Transmit(&huart3,"STDID 49", strlen("STDID 49"), 1000);
//     	//HAL_UART_Transmit(&huart3,"ID RELATED TO SOC\n\r",strlen("ID RELATED TO SOC\n\r"), 1000);
//
//     	value1=((arr[0]));
//     	value2=((arr[1])|(arr[2]<<8));
//     	value3=((arr[3])|(arr[4]<<8));
//     	value4=((arr[5])|(arr[6]<<8));
//     	value5=(arr[7]<<8);
//
//  		break;
//     }
//    case 74:
//     {
//     	HAL_UART_Transmit(&huart3,"STDID 50", strlen("STDID 50"), 1000);
//
//
//     	value1=((arr[0]));
//     	value2=((arr[1]));
//      	value3=((arr[2])|(arr[3]<<8));
//      	value4=((arr[4])|(arr[5]<<8));
//       	value5=(arr[6]);
//       	value6=(arr[7]);
//
//     	break;
//     }
//    case 75:
//     {
//     	HAL_UART_Transmit(&huart3,"STDID 51", strlen("STDID 51"), 1000);
//
//
//     	value1=(arr[0]);
//        value2=(arr[1]);
//     	value3=(arr[2]);
//       	value4=(arr[3]);
//       	value5=(arr[4]);
//       	value6=(arr[5]);
//		value7=((arr[6])|(arr[7]<<8));
//
//     	break;
//     }
//    case 76:
//     {
//     	HAL_UART_Transmit(&huart3,"STDID 52", strlen("STDID 52"), 1000);
//     	//HAL_UART_Transmit(&huart3,"ID RELATED TO TORQUE\n\r",strlen("ID RELATED TO TORQUE\n\r"), 1000);
//
//     	value1=((arr[0])|(arr[1]<<8)|(arr[2]<<16));
//     	value2=((arr[3])|(arr[4]<<8));
//     	value3=((arr[5])|(arr[6]<<8));
//     	value4=(arr[7]);
//
//
//     	break;
//     }
//    case 77:
//     {
//     	HAL_UART_Transmit(&huart3,"STDID 53", strlen("STDID 53"), 1000);
//     	//HAL_UART_Transmit(&huart3,"ID RELATED TO LOCKS\n\r",strlen("ID RELATED TO LOCKS\n\r"), 1000);
//
//     	value1=((arr[0])|(arr[1]<<8)|(arr[2]<<16));
//     	value2=((arr[3])|(arr[4]<<8)|(arr[5]<<16));
//     	value3=(arr[6]);
//       	value4=(arr[7]);
//
//     	break;
//     }
//     case 78 :
//    	{
//    		HAL_UART_Transmit(&huart3,"STDID 54", strlen("STDID 54"), 1000);
//    		//HAL_UART_Transmit(&huart3,"ID RELATED TO THROTTLE\n\r",strlen("ID RELATED TO THROTTLE\n\r"), 1000);
//
//    		value1=((arr[0])|(arr[1]<<8));
//    		value2=((arr[2])|(arr[3]<<8));
//         	value3=((arr[4])|(arr[5]<<8));
//         	value4=((arr[6])|(arr[7]<<8));
//
//    		break;
//
//    	}
//
//    default : HAL_UART_Transmit(&huart3,"error in stdid\n\r", strlen("error in stdid\n\r"), 1000);
//    }
//    */
//
//     //void (*fun_ptr)(int) = &switch_fp;
//     //void (*fun_ptr[4])(rxhead.StdId);
//    data=((arr[0])|(arr[1]<<8)|(arr[2]<<16)|(arr[3]<<24)|((arr[4]<<32)|(arr[5]<<40)|(arr[6]<<48)|(arr[7]<<56)));
//    //var.id=rxhead.StdId;
//    //var.dlc=rxhead.DLC;
//    //arr[0]=var.d0;
//    //arr[1]=var.d1;arr[2]=var.d2;arr[3]=var.d3;arr[4]=var.d4;arr[5]=var.d5;arr[6]=var.d6;arr[7]=var.d7;
//
//
//    void (*fp[4])(void);
//
//    fp[0]=call_to_speed_details;
//    fp[1]=call_to_battery_details;
//    fp[2]=call_to_voltage_details;
//    fp[3]=call_to_default;
//    choice=rxhead.StdId;
//    fp[choice]();
//
//    val1=(float)value1*floval;
//   	val2=(float)value2*floval;
//    val3=(float)value3*floval;
//    val4=(float)value4*floval;
//    val5=(float)value5*floval;
//    val6=(float)value6*floval;
//  	val7=(float)value7*floval;
//  	val8=(float)value8*floval;
//
//  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
////  	  HAL_Delay(1000);
////  	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
////  		  HAL_Delay(1000);
//  	 HAL_SuspendTick();
//
//  		  HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&rxhead, arr) != HAL_OK)
    {

    	HAL_UART_Transmit(&huart3,"error in reception\n\r", 20, 1000);
    	Error_Handler();
    }
	J1939_ReceiveMessages();
	 readMsg();

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t mssg[25];

			HAL_UART_Transmit(&huart3, "callbck_error\n\r", 25, 1000);
			memset(mssg,0,25);

}
void call_to_speed_details(uint32_t var)
{
	value1=((arr[0])|(arr[1]<<8)|(arr[2]<<16)|(arr[3]<<24));
	HAL_UART_Transmit(&huart3,str1, strlen(str1), 1000);
	value2=((arr[4])|(arr[5]<<8)|(arr[6]<<16)|(arr[7]<<24));
	HAL_UART_Transmit(&huart3, str2, strlen(str2), 1000);
    data1=((arr[0]&(0xF0))|arr[1]<<8|(((arr[2])&(0x0F))<<16));
    data2=((arr[1]&(0xF0))|arr[2]<<8|(((arr[3])&(0x0F))<<16));

}
void call_to_battery_details(uint32_t var)
{
	value1=((arr[0])|(arr[1]<<8));
	value2=((arr[2])|(arr[3]<<8));
	value3=((arr[4])|(arr[5]<<8));
	value4=((arr[6])|(arr[7]<<8));
	data1=((arr[2]&(0xF0))|arr[3]<<8|(((arr[4])&(0x0F))<<16));
	data2=((arr[5]&(0xF0))|arr[6]<<8|(((arr[7])&(0x0F))<<16));

}
void call_to_voltage_details(uint32_t var)
{
	value1=((arr[0])|(arr[3]<<8)|(arr[6]<<16));
	value2=((arr[1])|(arr[4]<<8)|(arr[7]<<16));
	value3=((arr[2])|(arr[5]<<8));
	data1=(arr[2]&(0xF0));
	data2=(arr[6]&(0xF0))|(((arr[7])&(0x0F))<<16);
	data3=((arr[5]&(0xF0))|arr[6]<<8|(((arr[7])&(0x0F))<<16));

}
void call_to_default(uint32_t var)
{
	value1=0xFFFFFFFF;
	value2=0xFFFFFFFF;
	value3=0xFFFFFFFF;
	value4=0xFFFFFFFF;
	value5=0xFFFFFFFF;
	value6=0xFFFFFFFF;
	value7=0xFFFFFFFF;
	value8=0xFFFFFFFF;

}





void can_tx()
{

	uint8_t message[5]="KINGS";
	uint32_t mailbox;
	txhead.ExtId=0x06;
	//txhead.ExtId=extid;
	txhead.IDE=CAN_ID_EXT;
	txhead.RTR=CAN_RTR_DATA;
	txhead.DLC=5;
	HAL_CAN_AddTxMessage(&hcan1,&txhead, message, &mailbox);

}

//void pgn()
//{
//	uint8_t ps;
//	uint8_t pf;
//	uint8_t datapage;
//	uint8_t reserved_bit;
//	ps=txhead.ExtId;
//	pf=txhead.ExtId>>8;
//	datapage=txhead.ExtId>>16;
//	reserved_bit=txhead.ExtId>>24;
//
//}


void CAN_SEND(uint8_t Data[])
{

	 //HAL_CAN_AddTxMessage(&hcan1,&txhead, Data, &mailbox);


}

void CAN_INIT(CAN_Handler1 *canh1, void (*caninit)(), void(*canstb)(),void (*canconfig)())
{
	//uint8_t status=CAN_INIT();
	canh1->caninit=caninit;//initialization
	canh1->canstb=canstb;//stb
	canh1->canconfig=canconfig;//filter configuration
	canh1->caninit();
	canh1->canstb();
	canh1->canconfig();
}

void CAN_ACTIVATE(CAN_Handler2 *canh2,void(*can_activate)(),void (*can_start)())
{
	canh2->can_activate=can_activate;//int activate
	canh2->can_start=can_start;//can_start
	canh2->can_activate();
	canh2->can_start();

}




