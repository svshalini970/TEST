# ifndef __J1939_Config_H
#define __J1939_Config_H

#include "J1939.H"


/***************************J1939 address configuration ****************** **********/
//The default address of the device (address naming is specified, please refer to Appendix B of J1939 Address and Identifier Assignment)
#define J1939_STARTING_ADDRESS_1 0x33


/******************************J1939 function configuration**************** ***********/
#define J1939_RX_QUEUE_SIZE 10
//When the MCU is too late to process the message, whether the receiving message queue is allowed to be overwritten by the new message
#define J1939_OVERWRITE_RX_QUEUE J1939_FALSE
#define J1939_TX_QUEUE_SIZE 5
//When the MCU is too late to process the message, whether the message queue is allowed to be overwritten by the new message
#define J1939_OVERWRITE_TX_QUEUE J1939_FALSE
//Whether to use polling mode (otherwise use interrupt mode)
#define J1939_POLL_ECAN J1939_TRUE
//Whether to enable software filter
#define J1939SoftwareFilterEn J1939_TRUE
/******************************J1939 transplant configuration function ************** *********/

#define Port_CAN_Transmit(MsgPtr) J1939_CAN_Transmit(MsgPtr)
#define Port_CAN_Receive(MsgPtr) J1939_CAN_Receive(MsgPtr)
#define Port_SetAddressFilter(Address) J1939_SetAddressFilter(Address)
/*Do not use interrupt mode, and do not transplant the following functions*/
#if J1939_POLL_ECAN == J1939_FALSE
#define Port_RXinterruptEnable() J1939_RXinterruptEnable()
#define Port_RXinterruptDisable() J1939_RXinterruptDisable()
#define Port_TXinterruptEnable() J1939_TXinterruptEnable()
#define Port_TXinterruptDisable() J1939_TXinterruptDisable()
#define Port_TXinterruptOk() J1939_TXinterruptOk()
#define Port_CAN_identifier_clc() CAN_identifier_clc()
#endif

# endif
