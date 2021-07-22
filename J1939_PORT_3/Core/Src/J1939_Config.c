
//# include  "J1939.H"
#include "J1939_Config.H"
#include "fun.h"
//extern CAN_RxHeaderTypeDef rxhead;
//extern CAN_RxHeaderTypeDef txhead;

//extern CAN_NODE Can_Node;    // CAN hardware selection

void J1939_SetAddressFilter()
{

	can_fil_config();

}

/*
*Input: *MsgPtr, the message to be sent by the protocol,
*Output:
*Description: Assign data from the MsgPtr structure to the structure that comes with the CAN driver
		First write the data in the MsgPtr passed into the function to the CAN structure, and then call the CAN driver's sending function
		By default, it supports the sending and receiving of 4 CAN hardware. If there are less than 4 channels, you only need to configure the corresponding Can_Node switch code area,
		Others (Select_CAN_NODE) ​​remain unchanged. Just return directly (break).
*/
void  J1939_CAN_Transmit(J1939_MESSAGE *MsgPtr)
{
	uint32_t mailbox;
			/* Load the 29-bit ID of the first CAN hardware */
								can_id.dummy=0;
								can_id.Prior=(MsgPtr->Mxe).Priority;
//								can_id.Reserved=(MsgPtr->Mxe).Reserve;
								can_id.Reserved=0;
								can_id.Data_Page=(MsgPtr->Mxe).DataPage;
								can_id.PDU_Format=(MsgPtr->Mxe).PDUFormat;
								can_id.PDU_Specific=(MsgPtr->Mxe).PDUSpecific;
								can_id.SA=(MsgPtr->Mxe).SourceAddress;


								uint32_t *extid = (uint32_t*)(&can_id);
								uint32_t value = *extid;
								//can_tx(value);

//			txhead.ExtId=0x06;
								//txhead.StdId=0x05;

								txhead.ExtId=value;
								txhead.IDE=CAN_ID_EXT;

			/* CAN hardware load data length */
			txhead.DLC=(MsgPtr->Mxe).DataLength;
			/* CAN hardware load data */
			//*MsgPtr->Array=message;
			/* CAN hardware load RTR */
			txhead.RTR=CAN_RTR_DATA;
			// CAN hardware starts to send data

			HAL_CAN_AddTxMessage(&hcan1,&txhead,(MsgPtr->Mxe).Data, &mailbox);

}
/*
*Input: *MsgPtr The pointer of the memory where the data is to be stored
*Output: 1 | 0
*Note: Read CAN drive data, if there is no data, return 0
		Take out the data in CAN and store it in the J1939_MESSAGE structure
		By default, it supports the sending and receiving of 4 CAN hardware. If there are less than 4 channels, you only need to configure the corresponding Can_Node switch code area,
		Others (Select_CAN_NODE) ​​remain unchanged. Just return directly (return 0)
*/

int J1939_CAN_Receive(J1939_MESSAGE *MsgPtr)
{

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0,&rxhead, arr) == HAL_OK) // Judge whether there is data coming in CAN hardware 1
							{
								//HAL_UART_Transmit(&huart3,"msg_from node_1 in reception\n\r", 30, 1000);
							    	//Error_Handler();
								// Your code, after reading the data from CAN hardware 1, store it in MsgPtr
								uint8_t cc;
								(MsgPtr->Mxe).DataPage=0;
								(MsgPtr->Mxe).Res=0;
								cc=(uint8_t)((rxhead.ExtId)>>24);
								(MsgPtr->Mxe).Priority=(uint8_t)((rxhead.ExtId)>>26);
								(MsgPtr->Mxe).Reserve=0;
								(MsgPtr->Mxe).PDUFormat=(uint8_t)((rxhead.ExtId)>>16);
								(MsgPtr->Mxe).PDUSpecific=(uint8_t)((rxhead.ExtId)>>8);
								if((MsgPtr->Mxe).PDUFormat<=239)
								{
									if ((MsgPtr->Mxe).PDUSpecific!=J1939_STARTING_ADDRESS_1)
									{
										return 0;
									}
								}
								(MsgPtr->Mxe).SourceAddress=(uint8_t)(rxhead.ExtId);
								(MsgPtr->Mxe).RTR=rxhead.RTR;
								(MsgPtr->Mxe).DataLength=rxhead.DLC;

								 (MsgPtr->Mxe).Data[0]=arr[0];
								 (MsgPtr->Mxe).Data[1]=arr[1];
								 (MsgPtr->Mxe).Data[2]=arr[2];
								 (MsgPtr->Mxe).Data[3]=arr[3];
								 (MsgPtr->Mxe).Data[4]=arr[4];
								 (MsgPtr->Mxe).Data[5]=arr[5];
								 (MsgPtr->Mxe).Data[6]=arr[6];
								 (MsgPtr->Mxe).Data[7]=arr[7];

								return  1 ;
							}


	return  0 ; // No message
}

/* Do not use interrupt mode, and do not transplant the following functions */
# if J1939_POLL_ECAN == J1939_FALSE
/*
*enter:
*Output:
*Note: Enable to accept interrupt
*/
	void  J1939_RXinterruptEnable ()
	{
		__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
/*
*enter:
*Output:
*Note: Disability accept interruption
*/
	void  J1939_RXinterruptDisable ()
	{
		__HAL_CAN_DISABLE_IT(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	}
/*
*enter:
*Output:
*Note: Enable sending interrupt
*/
	void  J1939_TXinterruptEnable ()
	{
		__HAL_CAN_ENABLE_IT(&hcan1,CAN_IT_TX_MAILBOX_EMPTY);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);

	}
/*
*enter:
*Output:
*Note: Disable sending interrupt
*/
	void  J1939_TXinterruptDisable ()
	{
		__HAL_CAN_DISABLE_IT(&hcan1,CAN_IT_TX_MAILBOX_EMPTY);
	}
/*
*enter:
*Output:
*Note: Trigger to send interrupt Peugeot bit, when the protocol stack is in interrupt mode, to send a message, this function will be called
	CAN driver function, the message will be sent directly, no agreement is needed to call any can driver function
*/
	void  J1939_TXinterruptOk ()
	{
		;
	}
/*
*enter:
*Output:
*Note: Clear the interrupt generation flag bit related to CAN driver, including (send interrupt flag bit, accept interrupt flag
	Flag bit, can bus error flag bit)
*/
	void  CAN_identifier_clc ()
	{
		;
	}
# endif


