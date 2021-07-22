#ifndef __FUN_H
#define __FUN_H
//#include "stm32l4xx_hal_def.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

uint8_t arr[8];
#pragma pack(1)
#define id_69 fp[0]
//#define id_70 fp[1]
//#define id_71 fp[2]
//#define id_72 fp[3]

 CAN_FilterTypeDef fil;
CAN_TxHeaderTypeDef txhead;
 CAN_RxHeaderTypeDef rxhead;
CAN_HandleTypeDef hcan1;
 UART_HandleTypeDef huart3;

typedef struct
{
	uint8_t reserved_bit : 1;
	uint8_t data_page : 1;
	uint8_t pdu_format;
	uint8_t pdu_specific;
}pgn_var;

typedef struct
{
	uint8_t d0,d1,d2,d3,d4,d5,d6,d7;
}data_var;

//unsigned char arr[70]="1578922367.777150;1;14FEF131;1;8;8;0;0;0;CFFFFFF300FFFF30";

//int MAXSIZE = 100;
//int stack[70];
//int top = -1;

struct pgn
{
	uint8_t reserved_bit :1;
	uint8_t datapage_select :1;
	uint8_t pdu_format;
	uint8_t pdu_specific;


};
void pgn();

 typedef struct _CAN_Handler1
 {
	 void (*caninit)();
	 void (*canstb)();
	 void (*canconfig)();

 }CAN_Handler1;

 typedef struct _CAN_Handler2
 {
	  void (*can_activate)();
	  void (*can_start)();


 }CAN_Handler2;


 typedef enum
 {
   CAN_OK       = 0x00,
   CAN_ERROR    = 0x01,
   CAN_BUSY     = 0x02,
   CAN_TIMEOUT  = 0x03
 } status;




uint32_t  value1,value2,value3,value4,value5,value6,value7,value8;
uint8_t  *str1,*str2,*str3,*str4,*str5,*str6,*str7,*str8;
uint32_t data1,data2,data3,data4,data5,data6,data7,data8,result;
uint64_t data;
uint32_t choice;
float val1,val2,val3,val4,val5,val6,val7,val8;
//uint8_t arr[80]={0};
//uint8_t buf[80]={0};
//float floval=0.0101;
//void can_fil_config();
float floval;
void call_to_speed_details();
void call_to_battery_details();
void call_to_voltage_details();
void call_to_default();
void (*fp[4])(void);
void can_tx();
void CAN_INIT(CAN_Handler1 *canh1, void (*caninit)(), void(*canstb)(),void (*canconfig)());
void CAN_ACTIVATE(CAN_Handler2 *canh2,void(*can_activate)(),void(*can_start)());
void can_fil_config();
//void (*fun_ptr[4])(rxhead.StdId);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
#endif /* __FUN_H */
