 #ifndef __J1939_H
#define __J1939_H
/******************************Type declaration******************** ****************/
#define FALSE 0
#define TRUE 1

#include <stdint.h>

/** Unified type definition
 * The number of bits of int, short, long may be different for different MCU compilers
 *
 * When porting J1939 protocol stack, you should configure here first
 */
typedef unsigned int j1939_uint32_t;   /** <32-bit unsigned integer*/
typedef int j1939_int32_t;             /** <32-bit integer */
typedef unsigned short j1939_uint16_t; /** <16-bit unsigned integer*/
typedef unsigned char j1939_uint8_t;   /** <8-bit unsigned integer*/
typedef char j1939_int8_t;             /** <8-bit unsigned integer */
#define J1939_NULL 0

//Function return code
#define RC_SUCCESS 0        /**< Success*/
#define RC_QUEUEEMPTY 1     /**< The queue is empty*/
#define RC_QUEUEFULL 1      /**< The queue is full*/
#define RC_CANNOTRECEIVE 2  /**< Cannot receive*/
#define RC_CANNOTTRANSMIT 2 /**< Cannot transmit*/
#define RC_PARAMERROR 3     /**<Parameter error*/

//Internal constant
#define J1939_FALSE 0 /**< stands for function error return*/
#define J1939_TRUE 1  /**< means the function returns correctly*/

// J1939 default priority (refer to J1939 document)
#define J1939_CONTROL_PRIORITY 0x03     /**< J1939 document default priority*/
#define J1939_INFO_PRIORITY 0x06        /**< J1939 document default priority*/
#define J1939_PROPRIETARY_PRIORITY 0x06 /**< J1939 document default priority*/
#define J1939_REQUEST_PRIORITY 0x06     /**< J1939 document default priority*/
#define J1939_ACK_PRIORITY 0x06         /**< J1939 document default priority*/
#define J1939_TP_CM_PRIORITY 0x01       /**< J1939 document default priority*/
#define J1939_TP_DT_PRIORITY 0x07       /**< J1939 document default priority*/

// Address defined by J1939
#define J1939_GLOBAL_ADDRESS 255 /**<Global address*/
#define J1939_NULL_ADDRESS 254   /**< Null address*/

//The PNG request response of the J1939 protocol stack, related definitions
#define J1939_PF_REQUEST2 201 /**< J1939 protocol stack request PF */
#define J1939_PF_TRANSFER 202 /**< J1939 protocol stack transfer PF */

#define J1939_PF_REQUEST 234        /**< Request or for handshake mechanism*/
#define J1939_PF_ACKNOWLEDGMENT 232 /**< Acknowledge request or for handshake mechanism*/

#define J1939_ACK_CONTROL_BYTE 0            /**< Used for TP (long frame data), representing confirmation*/
#define J1939_NACK_CONTROL_BYTE 1           /**< Used for TP (long frame data), PNG is not supported. Negative message*/
#define J1939_ACCESS_DENIED_CONTROL_BYTE 2  /**< Access is denied, but the information is supported and cannot respond temporarily (request to be sent again)*/
#define J1939_CANNOT_RESPOND_CONTROL_BYTE 3 /**< Can't respond, there is space but the received buffer is not enough, or the sending resource is occupied, temporarily unable to respond (need to send the request again)*/

//Some macro definitions of TP protocol
#define J1939_PF_DT 235    /**< Protocol transmission---Data transmission PF*/
#define J1939_PF_TP_CM 236 /**< Protocol transmission---link management PF*/

//TP timeout time, unit (ms)
#define J1939_TP_Tr 200                  /**< Macro defines the timeout time of TP*/
#define J1939_TP_Th 500                  /**< Macro defines the timeout time of TP*/
#define J1939_TP_T1 750                  /**< Macro defines the timeout time of TP*/
#define J1939_TP_T2 1250                 /**< Macro defines the timeout time of TP*/
#define J1939_TP_T3 1250                 /**< Macro defines the timeout time of TP*/
#define J1939_TP_T4 1050                 /**< Macro defines the timeout time of TP*/
#define J1939_TP_TIMEOUT_NORMAL 0        /**<No timeout is normal*/
#define J1939_TP_TIMEOUT_ABNORMAL 1      /**< Timeout*/
#define J1939_RTS_CONTROL_BYTE 16        /**< TP.CM_RTS*/
#define J1939_CTS_CONTROL_BYTE 17        /**< TP.CM_CTS*/
#define J1939_EOMACK_CONTROL_BYTE 19     /**< End of message response*/
#define J1939_BAM_CONTROL_BYTE 32        /**< Broadcast announcement message*/
#define J1939_CONNABORT_CONTROL_BYTE 255 /**< Connection interrupt control byte (abandon connection)*/
#define J1939_RESERVED_BYTE 0xFF         /**< The value of the reserved bit of the variable*/

//Definitions related to the J1939 network layer
#define J1939_PGN2_REQ_ADDRESS_CLAIM 0x00
#define J1939_PGN1_REQ_ADDRESS_CLAIM 0xEA
#define J1939_PGN0_REQ_ADDRESS_CLAIM 0x00

#define J1939_PGN2_COMMANDED_ADDRESS 0x00
#define J1939_PGN1_COMMANDED_ADDRESS 0xFE /**<Command address message*/
#define J1939_PGN0_COMMANDED_ADDRESS 0xD8 /**< Refer to J1939-81 address command configuration*/

#define J1939_PF_ADDRESS_CLAIMED 238
#define J1939_PF_CANNOT_CLAIM_ADDRESS 238
#define J1939_PF_PROPRIETARY_A 239 /**< Special A*/
#define J1939_PF_PROPRIETARY_B 255 /**< dedicated B*/

/**< Whether to support the TP protocol (whether to support the sending and receiving of long frames (data larger than 8 bytes))*/
#define J1939_TP_RX_TX J1939_TRUE
/**< The maximum message length supported by the TP protocol (maximum configurable to 1785)*/
#define J1939_TP_MAX_MESSAGE_LENGTH 240

/**Select enumeration of CAN node
*
* By default, it supports up to 4 CAN hardware\n
*/


#if J1939_TP_RX_TX
/**TP state description enumeration
*
*/
typedef enum
{
    J1939_TP_NULL,   /**< Long data transmission is idle, only the TP system is idle, can it be used to process the next send and receive request*/
    J1939_TP_RX,     /**< Long data transmission is receiving*/
    J1939_TP_TX,     /**< Long data transmission is sending*/
    J1939_TP_OSBUSY, /**< The long data transmission is busy, for example, it has just received a whole piece of long data, but the CPU has not been able to process it, and another long data request comes. In order to prevent the data from being overwritten, set the status to this value*/
} J1939_TP_State;

/**J1939 message object structure
*
* This structure implements the J1939 message object
*/
typedef struct
{
    j1939_uint32_t PGN;                              /**< PGN of J1939's message object*/
    j1939_uint8_t data[J1939_TP_MAX_MESSAGE_LENGTH]; /**<Data of the message object of J1939*/
    j1939_uint16_t byte_count;                       /**<Data size of J1939 message object*/
    j1939_uint8_t SA;                                /**< The destination address of the message object of J1939 (sending destination or receiving source)*/

} J1939_MESSAGE_T;
/**J1939 message object structure
*
* This structure implements the multi-frame message object of J1939
*/
typedef struct
{
    j1939_uint8_t *data;       /**< Buffer pointer*/
    j1939_uint16_t data_num;   /**< buffer size*/
    j1939_uint8_t SA;          /**< The data source address of the message object of J1939*/
    j1939_uint16_t byte_count; /**<Data size of J1939 message object*/
    j1939_uint32_t PGN;        /**< PGN of J1939's message object*/
} TP_RX_MESSAGE;
/**J1939_TP_Tx_Step enumeration
*
* The steps to record the TX of long frame (multi-frame) transmission are realized
*/
typedef enum
{
    J1939_TP_TX_WAIT,
    J1939_TP_TX_CM_START,
    J1939_TP_TX_CM_WAIT,
    J1939_TP_TX_DT,
    J1939_TP_WAIT_ACK,
    J1939_TP_TX_ERROR,
    J1939_TX_DONE,
} J1939_TP_Tx_Step; //The sending step of the protocol
/**J1939_TRANSPORT_TX_INFO structure
*
* Realize temporary data generated in long frame transmission, and exchange data with some transmissions
*/
typedef struct
{
    J1939_MESSAGE_T tp_tx_msg;         /**< J1939 message object*/
    j1939_uint16_t time;               /**< time*/
    j1939_uint8_t packet_offset_p;     /**< Packet offset pointer*/
    j1939_uint8_t packets_total;       /**< How many packets are there in total*/
    j1939_uint8_t packets_request_num; /**<The number of data packets requested to be sent (the number of data packets that the recipient is ready to accept)*/
    J1939_TP_Tx_Step state;            /**< Protocol sending step*/
} J1939_TRANSPORT_TX_INFO;
/**J1939_TP_Rx_Step enumeration
*
* Realize the steps of recording RX of long frame (multi-frame) transmission
*/
typedef enum
{
    J1939_TP_RX_WAIT,
    J1939_TP_RX_READ_DATA,
    J1939_TP_RX_DATA_WAIT,
    J1939_TP_RX_ERROR,
    J1939_RX_DONE,
} J1939_TP_Rx_Step; //The receiving step of the protocol

/**J1939_TRANSPORT_RX_INFO structure
*
* Realize temporary data generated in long frame transmission, and exchange data with some transmissions
*/
typedef struct
{
    J1939_MESSAGE_T tp_rx_msg;    /**< J1939 message object*/
    j1939_uint8_t osbusy;         /**< This position is 1, which means that the system is busy, and the cpu needs to process other things, and directly rejects all link requests.\n If this position is being accepted, this position is 1, and a link maintaining message frame will be sent. */
    j1939_uint16_t time;          /**< time*/
    j1939_uint8_t packets_total;  /**< How many packets are there in total*/
    j1939_uint8_t packets_ok_num; /**<Number of received packets*/
    J1939_TP_Rx_Step state;       /**< Acceptance steps of the agreement*/
} J1939_TRANSPORT_RX_INFO;

#endif //J1939_TP_RX_TX

/**
* @note realizes the response of Request_PGN
*/
struct Request_List
{
    j1939_uint8_t *data;
    j1939_uint16_t lenght;
    j1939_uint32_t PGN;

    void (*update)();          /**< The data needs to be updated in the function, if you don’t need to update the data, assign it to J1939_NULL*/
    struct Request_List *next; /**< At the end of the linked list, you need to keep J1939_NULL*/
};

// J1939 Data Structures
// J1939_MESSAGE_STRUCT aims to map the J1939 message block to the address mapping of the device. Only the field PDU format is not mapped to the device register.
// The structure should simply use PDUFormat and ignore PDUFormat_Top. Adjustments will be received immediately before transmission.
// Note: The compiler creates a structure from a lower position to a higher position, so there may be mismatched device registers.
#define J1939_MSG_LENGTH 9  //Message length
#define J1939_DATA_LENGTH 8 //Data length

//
struct CAN_ID
{
	j1939_uint8_t 	SA;
	j1939_uint8_t 	PDU_Specific;
	j1939_uint8_t 	PDU_Format;
	j1939_uint8_t 	Data_Page: 1 ;
	j1939_uint8_t 	Reserved: 1 ;
	j1939_uint8_t 	Prior: 3 ;
	j1939_uint8_t 	dummy: 3 ;

};
struct CAN_ID can_id;


//user defined struct
 struct Key_Info
{
	uint8_t Key_In:1;
	uint8_t Key_Off:1;
}Key_Infovar;

 struct Microcontroller_command
{
	uint8_t brake:1;
	uint8_t ignition_command:1;
}Microcontroller_commandvar;

 struct Switch_Info
{
	uint8_t brake:1;
	uint8_t highbeam:1;
	uint8_t indicator:2;
	uint8_t killswitch:1;
	uint8_t nav:3;
}Switch_Infovar;

struct BMS_Status
{
	uint8_t BMS_Status:1;
	uint8_t SOC;

}BMS_Statusvar;

 struct Microcontroller_Status
{
	uint8_t inverter_temperature;
	uint8_t motor_temperature;
	uint8_t motorcontroller_state:1;
	uint8_t ridemode_status:4;
	uint16_t speed;
	uint8_t startstop_status:1;

}Microcontroller_Statusvar;

 struct Ride_Command
{

	uint8_t Ridemode_command:4;

}Ride_Commandvar;

 struct VCU_Status
{
	uint8_t VCU_State:1;


}VCU_Statusvar;

/** J1939_MESSAGE_UNION structure
* Implemented J1939 message object
*
*
*/
union J1939_MESSAGE_UNION
{
    /** The ID of j1939 composes the structure
*
*/
    struct j1939_PID
    {
        j1939_uint8_t DataPage : 1;            /**< DataPage*/
        j1939_uint8_t Res : 1;                 /**< Res bit*/
        j1939_uint8_t Priority : 3;            /**< Priority*/
        j1939_uint8_t Reserve : 3;             /**< free*/
        j1939_uint8_t PDUFormat;               /**< PF*/
        j1939_uint8_t PDUSpecific;             /**< PS*/
        j1939_uint8_t SourceAddress;           /**< SA*/
        j1939_uint8_t DataLength : 4;          /**< data length*/
        j1939_uint8_t RTR : 4;                 /**< RTR bit*/
        j1939_uint8_t Data[J1939_DATA_LENGTH]; /**< data*/
        j1939_uint32_t PGN : 24;               /**<Parameter group number*/
        j1939_uint32_t ReservePGN : 8;         /**< idle*/
    };
    struct j1939_PID Mxe;                                      /**< The ID of j1939 forms a structure*/
    j1939_uint8_t Array[J1939_MSG_LENGTH + J1939_DATA_LENGTH]; /**< Union array, convenient to quickly process structure assignment*/
};

#define GroupExtension PDUSpecific
#define DestinationAddress PDUSpecific
/** A macro definition, specific variable name function naming
*
*/
typedef union J1939_MESSAGE_UNION J1939_MESSAGE;

union J1939_FLAGS_UNION
{
    struct
    {
        j1939_uint8_t TransmitMessagesdCover : 1; //When sending data, the J1939 protocol acceptance buffer has data coverage
        j1939_uint8_t ReceivedMessagesdCover : 1;  //When receiving data, the J1939 protocol acceptance buffer has data coverage
        j1939_uint8_t ReceivedMessagesDropped : 1; //When receiving data, the J1939 protocol receiving buffer overflowed with data
    };
    j1939_uint8_t FlagVal;
};

typedef union J1939_FLAGS_UNION J1939_FLAG;

/********************************************API**** ************************************************** ********/

//Initialization function
extern void J1939_Initialization();
//CAN drive transceiver interrupt entry
extern void J1939_ISR();
//Heartbeat function, called regularly
extern void J1939_Poll();
//Read a single frame message
extern j1939_uint8_t J1939_Read_Message(J1939_MESSAGE *MsgPtr);
//Send a single frame message
extern j1939_uint8_t J1939_Send_Message(J1939_MESSAGE *MsgPtr);
//Multi-frame (multi-group) message sending function (RTS/CTS transmission protocol)
extern j1939_int8_t J1939_TP_TX_Message(j1939_uint32_t PGN, j1939_uint8_t DA, j1939_uint8_t *data, j1939_uint16_t data_num);
//Multi-frame (multi-group) message acceptance function (RTS/CTS transmission protocol)
extern j1939_int8_t J1939_TP_RX_Message(TP_RX_MESSAGE *msg);
//Request to get a PGN
extern void J1939_Request_PGN(j1939_uint32_t pgn, j1939_uint8_t DA);
//Create a PGN response
extern void J1939_Create_Response(j1939_uint8_t data[], j1939_uint16_t dataLenght, j1939_uint32_t PGN, void (*dataUPFun)());
extern void               J1939_ReceiveMessages();
#endif //__J1939_H
