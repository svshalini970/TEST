
#ifndef __J1939_SOURCE
#define __J1939_SOURCE
#endif

#include "J1939.H"
#include "J1939_config.H"
#include <stdint.h>

#define ADDRESS_CLAIM_TX 1 /**< Enter address contention transmission processing mode*/
#define ADDRESS_CLAIM_RX 2 /**< Enter address contention acceptance processing mode*/

//Global variables.
/** The nominal character of the device
 *
 * We need to configure in "J1939_config.H"
 * @note assigns values ​​during initialization. For assignments, please refer to the 1939-81 document
 */
j1939_uint8_t CA_Name[J1939_DATA_LENGTH];

j1939_uint8_t J1939_Address;
J1939_FLAG J1939_Flags;
J1939_MESSAGE OneMessage;

//Node address
j1939_uint8_t NodeAddress;


j1939_uint8_t RXHead;
j1939_uint8_t RXTail;
j1939_uint8_t RXQueueCount;
J1939_MESSAGE RXQueue[J1939_RX_QUEUE_SIZE];

j1939_uint8_t TXHead;
j1939_uint8_t TXTail;
j1939_uint8_t TTXQueueCount;
J1939_MESSAGE TXQueue[J1939_TX_QUEUE_SIZE];

struct Request_List REQUEST_LIST;

uint8_t buffer[8];

#if J1939_TP_RX_TX
//TP protocol global variables
J1939_TP_State J1939_TP_State_t;
J1939_TRANSPORT_RX_INFO TP_RX_MSG;
J1939_TRANSPORT_TX_INFO TP_TX_MSG;
#endif //J1939_TP_RX_TX

 void J1939_ReceiveMessages(void);
 j1939_uint8_t J1939_TransmitMessages(void);




/**
* @note hardware filter 2 or software filter filter configuration (set PS segment)\n
*/
void SetAddressFilter(j1939_uint8_t Address)
{
/*Software filtering*/
#if J1939SoftwareFilterEn == J1939_TRUE

        NodeAddress = Address;

#endif //J1939SoftwareFilterEn
    /*Hardware filtering*/
    Port_SetAddressFilter(Address);
}

/**
* @param[in] J1939_MESSAGE *
* @note sends *MsgPtr information, all data fields (such as data length, priority, and source address) must have been set. \n
*/
void SendOneMessage(J1939_MESSAGE *MsgPtr)
{
    //Set the last part of the message to ensure that the DataLength specification. (Refer to CAN B2.0)
    MsgPtr->Mxe.Res = 0; //Refer to the data link layer of J1939 (SAE J1939-21)
    MsgPtr->Mxe.RTR = 0;
    if (MsgPtr->Mxe.DataLength > 8)
        MsgPtr->Mxe.DataLength = 8;
    //Send a frame of message and load all the messages in J1939_MESSAGE into the own structure of the can module
    Port_CAN_Transmit(MsgPtr);
}

/**
* @param[in] MsgPtr The message that the user wants to leave the team
* @return RC_SUCCESS message dequeue successfully
* @return RC_QUEUEEMPTY no message to return
* @note reads a message from the receiving queue to *MsgPtr. If we are using interrupts, we need to disable the interrupts. When getting data from the receive queue
*/
j1939_uint8_t J1939_DequeueMessage(J1939_MESSAGE *MsgPtr)
{
    j1939_uint8_t _rc = RC_SUCCESS;

//*************************** Off accept interruption ***************** **************
#if J1939_POLL_ECAN == J1939_FALSE
    Port_RXinterruptDisable();
#endif

    if (RXQueueCount == 0)
    {
        _rc = RC_QUEUEEMPTY;
    }
    else
    {
        *MsgPtr = RXQueue[RXHead];
        RXHead++;
        if (RXHead >= J1939_RX_QUEUE_SIZE)
            RXHead = 0;
        RXQueueCount--;
    }

    //***************************Open to accept interruption **************** **************
#if J1939_POLL_ECAN == J1939_FALSE
    Port_RXinterruptEnable();
#endif

    return _rc;
}
/**
* @param[in] MsgPtr is the cache for storing read messages
* @return RC_SUCCESS read the message successfully,
* @return RC_QUEUEEMPTY Reading the message is unsuccessful and there is no message.
* @note reads a message from the receiving queue to *MsgPtr.
*/
j1939_uint8_t J1939_Read_Message(J1939_MESSAGE *MsgPtr)
{
    return J1939_DequeueMessage(MsgPtr);
}
/**
* @param[in] MsgPtr The message that the user wants to join the team
* @return RC_SUCCESS message successfully joined the team
* @return RC_QUEUEFULL The sending queue is full, and the message has failed to join the queue
* @return RC_CANNOTTRANSMIT The system currently cannot send messages
* @note This program puts *MsgPtr in the message queue\n
If the information cannot be enqueued or sent, there will be a corresponding return prompt,\n
If the transmit interrupt is set (available), when the message is queued, the transmit interrupt is enabled
*/
j1939_uint8_t J1939_EnqueueMessage(J1939_MESSAGE *MsgPtr)
{
    j1939_uint8_t _rc = RC_SUCCESS;

#if J1939_POLL_ECAN == J1939_FALSE
    Port_TXinterruptDisable();
#endif

    if (0)
        _rc = RC_CANNOTTRANSMIT;
    else
    {
        if ((J1939_OVERWRITE_TX_QUEUE == J1939_TRUE) ||
            (TTXQueueCount < J1939_TX_QUEUE_SIZE))
        {
            if (TTXQueueCount < J1939_TX_QUEUE_SIZE)
            {
                TTXQueueCount++;
                TXTail++;
                if (TXTail >= J1939_TX_QUEUE_SIZE)
                    TXTail = 0;
            }
            else
            {
                J1939_Flags.TransmitMessagesdCover = 1; //Send data is covered, the previous frame data is covered
            }
            TXQueue[TXTail] = *MsgPtr;
        }
        else
            _rc = RC_QUEUEFULL;
    }

#if J1939_POLL_ECAN == J1939_FALSE
    Port_TXinterruptEnable();
    //Trigger to send interrupt
    Port_TXinterruptOk();
#endif
    return _rc;
}
/**
* @param[in] MsgPtr is a buffer for storing sent messages
* @return RC_SUCCESS Send message successfully
* @return RC_QUEUEFULL Sending the message is unsuccessful, the sending queue is full, and the message entering the queue fails
* @return RC_CANNOTTRANSMIT Sending the message is unsuccessful, the system cannot send the message currently
* @note If the message cannot be enqueued or sent, there will be a corresponding return prompt,\n
*/
j1939_uint8_t J1939_Send_Message(J1939_MESSAGE *MsgPtr)
{
    return J1939_EnqueueMessage(MsgPtr);
}
/**
*
* @note This code is called during system initialization, (placed after CAN device initialization)\n
Initialize J1939 global variables\n
*/
void J1939_Initialization()
{
    /*Initialize global variables*/
    J1939_Flags.FlagVal = 0; //No address is declared, other flags will be set to 0 (reset)

    /*Initialize receiving and sending queues*/
    TXHead = 0;
    TXTail = 0xFF;
    RXHead = 0;
    RXTail = 0xFF;
    TTXQueueCount = 0;
    RXQueueCount = 0;
    /*Initialize node address*/
    NodeAddress = J1939_STARTING_ADDRESS_1;

    /*Initialize the selection of CAN nodes*/
    /*Initialize the request list*/
    REQUEST_LIST.PGN = 0;
    REQUEST_LIST.data = J1939_NULL;
    REQUEST_LIST.update = J1939_NULL;
    REQUEST_LIST.lenght = 0;

    REQUEST_LIST.next = J1939_NULL;
    /*Set the TP protocol to idle*/
#if J1939_TP_RX_TX
    J1939_TP_State_t = J1939_TP_NULL;

    TP_TX_MSG.packets_request_num = 0;
    TP_TX_MSG.packets_total = 0;
    TP_TX_MSG.packet_offset_p = 0;
    TP_TX_MSG.time = 0;
    TP_TX_MSG.state = J1939_TP_TX_WAIT;

    TP_RX_MSG.packets_ok_num = 0;
    TP_RX_MSG.packets_total = 0;
    TP_RX_MSG.time = 0;
    TP_RX_MSG.state = J1939_TP_RX_WAIT;
#endif
}

/**
* @note This function is called when the device generates a CAN interrupt (may be receiving interrupt or sending interrupt)\n
        First we have to clear the interrupt flag\n
        Then call the accept or send function.
*/
#if J1939_POLL_ECAN == J1939_FALSE
void J1939_ISR(void)
{
    //Determine the relevant flag, whether to accept or send
    //Clear the flag
    Port_CAN_identifier_clc();
    //Call the relevant processing function
    J1939_ReceiveMessages();
    J1939_TransmitMessages();
#if J1939_TP_RX_TX
    J1939_TP_Poll();
#endif //J1939_TP_RX_TX
    //There may be an interrupt due to an error, directly clear the relevant flag
}
#endif

/**
* @param[in] ElapsedTime is an approximate number of milliseconds, usually set to 5 or 3
* @note If we obtain information by polling, this function will be called every few milliseconds. \n
        Constantly receiving and sending messages from the message queue\n
        In addition, if we are waiting for an address contention response. \n
        If it times out, we only receive specific messages (destination address = J1939_Address)\n

        If the device uses interrupts, this function is called, after calling the J1939_Initialization() function, because\n
        J1939_Initialization() may initialize the WaitingForAddressClaimContention flag to 1.\n

        If a command address message is received, this function must also be called, just in case the bus requires us to change the address\n

        If the interrupt mode is used, this program will not process receiving and sending messages, only address contention timeout. \n
*/
//Declare TP polling function
void J1939_TP_Poll();
void J1939_Poll()
{
    //We must call the J1939_ReceiveMessages accept function before the time is reset to 0.
#if J1939_POLL_ECAN == J1939_TRUE

    //J1939_Address = NodeAddress;//TODO :remove
    J1939_ReceiveMessages();
    J1939_TransmitMessages();

#if J1939_TP_RX_TX
    J1939_TP_Poll();
#endif //J1939_TP_RX_TX
#endif //J1939_POLL_ECAN == J1939_TRUE
}
void J1939_Response(const j1939_uint32_t PGN);

#if J1939SoftwareFilterEn == J1939_TRUE

/**
* @return RC_SUCCESS message is acceptable
* @return RC_CANNOTTRANSMIT message is not acceptable
* @note software filter\n
* @note Based on the SAE J1939 protocol, we need the CAN controller to provide at least 3 filters for the J1939 protocol code. The three filters are configured as follows:
        1. Set filter 0 to only accept broadcast information (PF = 240 -255).
        2. Set filter 1, 2 only accept global address (J1939_GLOBAL_ADDRESS)
        3. As the program runs, filter 2 will be changed to adapt to the program logic.
*/
j1939_uint8_t J1939_Messages_Filter(J1939_MESSAGE *MsgPtr)
{
    /*Filter 0*/
    if ((MsgPtr->Mxe.PDUFormat) >= 240)
    {
        return RC_SUCCESS;
    }
    /*Filter 1*/
    if (((MsgPtr->Mxe.PDUFormat) < 240) && (MsgPtr->Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS))
    {
        return RC_SUCCESS;
    }

        if (((MsgPtr->Mxe.PDUFormat) < 240) && (MsgPtr->Mxe.PDUSpecific == NodeAddress))
        {
            return RC_SUCCESS;
        }

    return RC_CANNOTTRANSMIT;
}

#endif //J1939SoftwareFilterEn

/**
* @note This program is called when the CAN transceiver receives data (interrupt or polling). \n
        If a message is accepted, it will be called\n
        If the information is a network management information or long frame transmission (TP), the received information will be processed in this function. \n
        Otherwise, the information will be placed in the user's receiving queue. \n
        Note: The interrupt is disabled during the running of this program. \n
*/
void J1939_ReceiveMessages(void)
{
#if J1939_TP_RX_TX
    j1939_uint32_t _pgn = 0;
#endif //J1939_TP_RX_TX
    /*Read information from the receiving cache to OneMessage, OneMessage is a global variable*/
    /*Port_CAN_Receive function reads data and returns 1, and returns 0 if there is no data*/
    if (Port_CAN_Receive(&OneMessage))
    {
#if J1939SoftwareFilterEn == J1939_TRUE
        if (J1939_Messages_Filter(&OneMessage) != RC_SUCCESS)
        {
            return;
        }
#endif //J1939SoftwareFilterEn
        switch (OneMessage.Mxe.PDUFormat)
        {
#if J1939_TP_RX_TX
        case J1939_PF_TP_CM: //Refer to J1939-21 TP multi-frame transmission protocol
            _pgn = (j1939_uint32_t)((OneMessage.Mxe.Data[7] << 16) & 0xFF0000) + (j1939_uint32_t)((OneMessage.Mxe.Data[6] << 8) & 0xFF00) + (j1939_uint32_t)((OneMessage.Mxe.Data[5]) & 0xFF);
            if ((J1939_TP_State_t == J1939_TP_NULL) && (TP_RX_MSG.state == J1939_TP_RX_WAIT))
            {
                if (OneMessage.Mxe.Data[0] == 16)
                {
                    J1939_TP_State_t = J1939_TP_RX;

                    TP_RX_MSG.tp_rx_msg.SA = OneMessage.Mxe.SourceAddress;
                    TP_RX_MSG.tp_rx_msg.PGN = (j1939_uint32_t)((OneMessage.Mxe.Data[7] << 16) & 0xFF0000) + (j1939_uint32_t)((OneMessage.Mxe.Data[6] << 8) & 0xFF00) + (j1939_uint32_t)((OneMessage.Mxe.Data[5]) & 0xFF);
                    /*If the system is busy*/
                    if (TP_RX_MSG.osbusy)
                    {
                        TP_RX_MSG.state = J1939_TP_RX_ERROR;
                        break;
                    }
                    /* Determine whether there is enough memory to receive data, if not directly, disconnect the connection*/
                    if (((j1939_uint32_t)((OneMessage.Mxe.Data[2] << 8) & 0xFF00) + (j1939_uint32_t)((OneMessage.Mxe.Data[1]) & 0xFF)) > J1939_TP_MAX_MESSAGE_LENGTH)
                    {
                        TP_RX_MSG.state = J1939_TP_RX_ERROR;
                        break;
                    }
                    TP_RX_MSG.tp_rx_msg.byte_count = ((j1939_uint32_t)((OneMessage.Mxe.Data[2] << 8) & 0xFF00) + (j1939_uint32_t)((OneMessage.Mxe.Data[1]) & 0xFF));
                    TP_RX_MSG.packets_total = OneMessage.Mxe.Data[3];
                    TP_RX_MSG.time = J1939_TP_T2;
                    TP_RX_MSG.state = J1939_TP_RX_READ_DATA;
                    break;
                }
                goto PutInReceiveQueue;
                break;
            }
            if (J1939_TP_State_t == J1939_TP_TX)
            {
                /*Check PGN*/
                if (_pgn == TP_TX_MSG.tp_tx_msg.PGN)
                {
                    switch (OneMessage.Mxe.Data[0])
                    {
                    case J1939_RTS_CONTROL_BYTE:
                        /* The program runs here, indicating that a virtual link has been established with device 1 in the network (as the sender), but a link request from device 2 has been received, and the same PGN message request*/
                        /* According to the data link layer of J1939-21, we need to keep the original link and do nothing. Device 2 will automatically give up the link for timeout */
                        break;
                    case J1939_CTS_CONTROL_BYTE:
                        if ((J1939_TP_TX_CM_WAIT == TP_TX_MSG.state) || (J1939_TP_WAIT_ACK == TP_TX_MSG.state))
                        {
                            /* Send and wait to be held */
                            if (0x00u == OneMessage.Mxe.Data[1])
                            {
                                /* Refresh waiting counter */
                                TP_TX_MSG.time = J1939_TP_T4;
                            }
                            else
                            {
                                if ((OneMessage.Mxe.Data[2] + OneMessage.Mxe.Data[1]) > (TP_TX_MSG.packets_total + 1))
                                {
                                    /*Request is out of data packet range*/
                                    TP_TX_MSG.state = J1939_TP_TX_ERROR;
                                }
                                else
                                { /* response parameter OK */
                                    TP_TX_MSG.packets_request_num = OneMessage.Mxe.Data[1];
                                    TP_TX_MSG.packet_offset_p = (j1939_uint8_t)(OneMessage.Mxe.Data[2] - 1);
                                    TP_TX_MSG.state = J1939_TP_TX_DT;
                                }
                            }
                        }
                        break;
                    case J1939_EOMACK_CONTROL_BYTE:
                        if (J1939_TP_WAIT_ACK == TP_TX_MSG.state)
                        {
                            TP_TX_MSG.state = J1939_TX_DONE;
                        }
                        //Here can add a verification of the data
                        break;
                    case J1939_CONNABORT_CONTROL_BYTE:
                        //Receive a connection abandonment, do nothing, the agreement will actively abandon the connection after a delay
                        break;
                    default:
                        break;
                    }
                }
            }
            goto PutInReceiveQueue;
            break;
#endif //J1939_TP_RX_TX

#if J1939_TP_RX_TX
        case J1939_PF_DT:
            if ((TP_RX_MSG.state == J1939_TP_RX_DATA_WAIT) && (TP_RX_MSG.tp_rx_msg.SA == OneMessage.Mxe.SourceAddress))
            {
                TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0] - 1) * 7u] = OneMessage.Mxe.Data[1];
                TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0] - 1) * 7u + 1] = OneMessage.Mxe.Data[2];
                TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0] - 1) * 7u + 2] = OneMessage.Mxe.Data[3];
                TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0] - 1) * 7u + 3] = OneMessage.Mxe.Data[4];
                TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0] - 1) * 7u + 4] = OneMessage.Mxe.Data[5];
                TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0] - 1) * 7u + 5] = OneMessage.Mxe.Data[6];
                TP_RX_MSG.tp_rx_msg.data[(OneMessage.Mxe.Data[0] - 1) * 7u + 6] = OneMessage.Mxe.Data[7];
                /*Special processing to re-accept the received data packet*/
                if ((OneMessage.Mxe.Data[0]) > TP_RX_MSG.packets_ok_num)
                {
                    TP_RX_MSG.packets_ok_num++;
                }
                TP_RX_MSG.time = J1939_TP_T1;
                /*Determine whether an even number of data packets have been received or the last data packet has been read*/
                if ((TP_RX_MSG.packets_ok_num % 2 == 0) || (TP_RX_MSG.packets_ok_num == TP_RX_MSG.packets_total))
                {
                    TP_RX_MSG.state = J1939_TP_RX_READ_DATA;
                    break;
                }
                break;
            }
            //The program cannot run to this point, but we can't give up the received data packet
            goto PutInReceiveQueue;
#endif //J1939_TP_RX_TX
        case J1939_PF_REQUEST:
            /*Use OneMessage.Mxe.PGN to store the requested PGN*/
            if (OneMessage.Mxe.Data[1] < 240)
            {
                OneMessage.Mxe.PGN = (j1939_uint32_t)((OneMessage.Mxe.Data[2] << 16) & 0x030000) + (j1939_uint32_t)((OneMessage.Mxe.Data[1] << 8) & 0xFF00) + 0x00;
            }
            else
            {
                OneMessage.Mxe.PGN = (j1939_uint32_t)((OneMessage.Mxe.Data[2] << 16) & 0x030000) + (j1939_uint32_t)((OneMessage.Mxe.Data[1] << 8) & 0xFF00) + (j1939_uint32_t)((OneMessage.Mxe.Data[0]) & 0xFF);
            }
            J1939_Response(OneMessage.Mxe.PGN);
            break;
        default:
        PutInReceiveQueue:
        {
            /*
if(OneMessage.Mxe.PDUFormat <240){
OneMessage.Mxe.PGN = (j1939_uint32_t)((OneMessage.Array[0]<<16)&0x030000)
+(j1939_uint32_t)((OneMessage.Array[1]<<8)&0xFF00)
+0x00;
}else{
OneMessage.Mxe.PGN = (j1939_uint32_t)((OneMessage.Array[0]<<16)&0x030000)
+(j1939_uint32_t)((OneMessage.Array[1]<<8)&0xFF00)
+(j1939_uint32_t)((OneMessage.Array[2])&0xFF);
}
*/
            if (OneMessage.Mxe.PDUFormat < 240)
            {
                OneMessage.Mxe.PGN = (OneMessage.Mxe.Res << 17) + (OneMessage.Mxe.DataPage << 16) + (OneMessage.Mxe.PDUFormat << 8);
            }
            else
            {
                OneMessage.Mxe.PGN = (OneMessage.Mxe.Res << 17) + (OneMessage.Mxe.DataPage << 16) + (OneMessage.Mxe.PDUFormat << 8) + OneMessage.Mxe.PDUSpecific;
            }


                if ((J1939_OVERWRITE_RX_QUEUE == J1939_TRUE) ||
                    (RXQueueCount < J1939_RX_QUEUE_SIZE))
                {
                    if (RXQueueCount < J1939_RX_QUEUE_SIZE)
                    {
                        RXQueueCount++;
                        RXTail++;
                        if (RXTail >= J1939_RX_QUEUE_SIZE)
                            RXTail = 0;
                    }
                    else
                    {
                        J1939_Flags.ReceivedMessagesdCover = 1; //Generate data coverage
                    }
                    RXQueue[RXTail] = OneMessage;
                }
                else
                    J1939_Flags.ReceivedMessagesDropped = 1; //Data overflow occurs


        }
        }
    }
}

/**
* @return RC_SUCCESS message sent successfully
* @return RC_CANNOTTRANSMIT The system did not send a message, there is no message to send, or the wrong CAN device
* @note After calling this function, if there is a message in the sending message queue, the message will be sent. If the message cannot be sent, the related error code will be returned. \n
             During the running of the program, the interrupt is disabled.
*/
 j1939_uint8_t J1939_TransmitMessages()
{

        if (TTXQueueCount == 0)
        {
//If there is no message to be sent from the sending message queue, resume interruption (clear the sending flag)
#if J1939_POLL_ECAN == J1939_FALSE
            Port_TXinterruptEnable();
#endif
            return RC_CANNOTTRANSMIT;
        }
        else
        {
            while (TTXQueueCount > 0)
            {
                /*Ensure that the last data was sent successfully*/
                /**************A judgment function can be added ****************************/

                TXQueue[TXHead].Mxe.SourceAddress = NodeAddress;

                SendOneMessage((J1939_MESSAGE *)&(TXQueue[TXHead]));
                TXHead++;
                if (TXHead >= J1939_TX_QUEUE_SIZE)
                    TXHead = 0;
                TTXQueueCount--;
            }

/*Some flag bits are configured to enable interrupts*/
#if J1939_POLL_ECAN == J1939_FALSE
            Port_TXinterruptEnable();
#endif
        }


    return RC_SUCCESS;
}
#if J1939_TP_RX_TX
/**
* @note sends TP.DT, refer to J1939-21
*/
void J1939_TP_DT_Packet_send(void)
{
    J1939_MESSAGE _msg;
    j1939_uint16_t _packet_offset_p;
    j1939_int32_t _i = 0;
    _msg.Mxe.Priority = J1939_TP_DT_PRIORITY;
    _msg.Mxe.DataPage = 0;
    _msg.Mxe.PDUFormat = J1939_PF_DT;
    _msg.Mxe.DestinationAddress = TP_TX_MSG.tp_tx_msg.SA;
    _msg.Mxe.DataLength = 8;

    /*Get the number of data packets sent by request*/
    if (TP_TX_MSG.packets_request_num > 0)
    {
        TP_TX_MSG.packets_request_num--;
        /*Get data offset pointer*/
        _packet_offset_p = (j1939_uint16_t)(TP_TX_MSG.packet_offset_p * 7u);
        /*Load data package number*/
        _msg.Mxe.Data[0] = (j1939_uint8_t)(1u + TP_TX_MSG.packet_offset_p);

        for (_i = 0; _i < 7; _i++)
        {
            _msg.Mxe.Data[_i + 1] = TP_TX_MSG.tp_tx_msg.data[_packet_offset_p + _i];
        }
        /*Is it the last packet of data message*/
        if (TP_TX_MSG.packet_offset_p == (TP_TX_MSG.packets_total - 1u))
        {
            /* Whether the parameter group can be filled, whether it needs to be filled, */
            if (_packet_offset_p > TP_TX_MSG.tp_tx_msg.byte_count - 7)
            {
                /*Calculate the number of data to be filled*/
                _i = TP_TX_MSG.tp_tx_msg.byte_count - _packet_offset_p - 7;

                for (; _i < 0; _i++)
                {
                    /*By default, the parameter group size of J1939 is 8*/
                    _msg.Mxe.Data[_i + 8] = J1939_RESERVED_BYTE;
                }
            }

            TP_TX_MSG.packets_request_num = 0;
            TP_TX_MSG.packet_offset_p = 0;
            TP_TX_MSG.time = J1939_TP_T3;
            /* Jump step, wait for end confirmation or resend data request*/
            TP_TX_MSG.state = J1939_TP_WAIT_ACK;
        }
        else
        {
            /*Prepare for the next data transmission*/
            TP_TX_MSG.packet_offset_p++;
            TP_TX_MSG.state = J1939_TP_TX_DT;
        }

        /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
        J1939_EnqueueMessage(&_msg);
    }
    else
    {

        TP_TX_MSG.packets_request_num = 0;
        TP_TX_MSG.packet_offset_p = 0;
        TP_TX_MSG.time = J1939_TP_T3;
        TP_TX_MSG.state = J1939_TP_WAIT_ACK;
    }
}
/**
* @note sends TP. CM-RTS, 16, 23, 4, 255, PGN message, refer to J1939-21,
*/
void J1939_CM_Start(void)
{
    j1939_uint32_t pgn_num;
    J1939_MESSAGE _msg;

    pgn_num = TP_TX_MSG.tp_tx_msg.PGN;

    _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
    _msg.Mxe.DataPage = 0;
    _msg.Mxe.PDUFormat = 0xf1;
    _msg.Mxe.DestinationAddress = TP_TX_MSG.tp_tx_msg.SA;
    _msg.Mxe.DataLength = 8;
    _msg.Mxe.Data[0] = J1939_RTS_CONTROL_BYTE;
    _msg.Mxe.Data[1] = (j1939_uint8_t)TP_TX_MSG.tp_tx_msg.byte_count;
    _msg.Mxe.Data[2] = (j1939_uint8_t)((TP_TX_MSG.tp_tx_msg.byte_count) >> 8);
    _msg.Mxe.Data[3] = TP_TX_MSG.packets_total;
    _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num >> 16) & 0xff);
    _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num >> 8 & 0xff);
    _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

    /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
    J1939_EnqueueMessage(&_msg);

    /*Refresh waiting time, trigger the next step ()*/
    TP_TX_MSG.time = J1939_TP_T3;
    TP_TX_MSG.state = J1939_TP_TX_CM_WAIT;
}
/**
* @note break TP link
*/
void J1939_TP_TX_Abort(void)
{
    J1939_MESSAGE _msg;
    j1939_uint32_t pgn_num;

    pgn_num = TP_TX_MSG.tp_tx_msg.PGN;

    _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
    _msg.Mxe.DataPage = 0;
    _msg.Mxe.PDUFormat = J1939_PF_TP_CM;
    _msg.Mxe.DestinationAddress = TP_TX_MSG.tp_tx_msg.SA;
    _msg.Mxe.DataLength = 8;
    _msg.Mxe.Data[0] = J1939_CONNABORT_CONTROL_BYTE;
    _msg.Mxe.Data[1] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[2] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num >> 16) & 0xff);
    _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num >> 8 & 0xff);
    _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

    /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
    J1939_EnqueueMessage(&_msg);
    /*End sending*/
    TP_TX_MSG.state = J1939_TX_DONE;
}
/**
* @note break TP link
*/
void J1939_TP_RX_Abort(void)
{
    J1939_MESSAGE _msg;
    j1939_uint32_t pgn_num;

    pgn_num = TP_RX_MSG.tp_rx_msg.PGN;

    _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
    _msg.Mxe.DataPage = 0;
    _msg.Mxe.PDUFormat = J1939_PF_TP_CM;
    _msg.Mxe.DestinationAddress = TP_RX_MSG.tp_rx_msg.SA;
    _msg.Mxe.DataLength = 8;
    _msg.Mxe.Data[0] = J1939_CONNABORT_CONTROL_BYTE;
    _msg.Mxe.Data[1] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[2] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
    _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num >> 16) & 0xff);
    _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num >> 8 & 0xff);
    _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

    /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
    J1939_EnqueueMessage(&_msg);
    /*End sending*/
    TP_RX_MSG.state = J1939_RX_DONE;
}
/**
* @note TP's timer
*/
j1939_uint8_t J1939_TP_TX_RefreshCMTimer(j1939_uint16_t dt_ms)
{
    if ((J1939_TP_TX_CM_WAIT == TP_TX_MSG.state) || (J1939_TP_WAIT_ACK == TP_TX_MSG.state))
    {
        if (TP_TX_MSG.time > dt_ms)
        {
            TP_TX_MSG.time = TP_TX_MSG.time - dt_ms;
            return J1939_TP_TIMEOUT_NORMAL;
        }
        else
        {
            /*time out */
            TP_TX_MSG.time = 0u;
            return J1939_TP_TIMEOUT_ABNORMAL;
        }
    }
    else
    {
        return J1939_TP_TIMEOUT_NORMAL;
    }
}
/**
* @note TP's timer
*/
j1939_uint8_t J1939_TP_RX_RefreshCMTimer(j1939_uint16_t dt_ms)
{
    if ((J1939_TP_RX_DATA_WAIT == TP_RX_MSG.state))
    {
        if (TP_RX_MSG.time > dt_ms)
        {
            TP_RX_MSG.time = TP_RX_MSG.time - dt_ms;
            return J1939_TP_TIMEOUT_NORMAL;
        }
        else
        {
            /*time out */
            TP_RX_MSG.time = 0u;
            return J1939_TP_TIMEOUT_ABNORMAL;
        }
    }
    else
    {
        return J1939_TP_TIMEOUT_NORMAL;
    }
}
/**
* @note sends read data TP.CM_CTS and EndofMsgAck messages.
*/
void J1939_read_DT_Packet()
{
    J1939_MESSAGE _msg;
    j1939_uint32_t pgn_num;
    pgn_num = TP_RX_MSG.tp_rx_msg.PGN;

    _msg.Mxe.Priority = J1939_TP_CM_PRIORITY;
    _msg.Mxe.DataPage = 0;
    _msg.Mxe.PDUFormat = J1939_PF_TP_CM;
    _msg.Mxe.DestinationAddress = TP_RX_MSG.tp_rx_msg.SA;
    _msg.Mxe.DataLength = 8;

    /*If the system is busy, keep the link but do not send messages*/
    if (TP_RX_MSG.osbusy)
    {
        _msg.Mxe.Data[0] = J1939_CTS_CONTROL_BYTE;
        _msg.Mxe.Data[1] = 0;
        _msg.Mxe.Data[2] = J1939_RESERVED_BYTE;
        _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
        _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
        _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num >> 16) & 0xff);
        _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num >> 8 & 0xff);
        _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);
        /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
        J1939_EnqueueMessage(&_msg);
        return;
    }
    if (TP_RX_MSG.packets_total > TP_RX_MSG.packets_ok_num)
    {
        /*The last response, if less than 2 packets of data*/
        if ((TP_RX_MSG.packets_total - TP_RX_MSG.packets_ok_num) == 1)
        {
            _msg.Mxe.Data[0] = J1939_CTS_CONTROL_BYTE;
            _msg.Mxe.Data[1] = 1;
            _msg.Mxe.Data[2] = TP_RX_MSG.packets_total;
            _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
            _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
            _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num >> 16) & 0xff);
            _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num >> 8 & 0xff);
            _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);
            /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
            J1939_EnqueueMessage(&_msg);
            TP_RX_MSG.state = J1939_TP_RX_DATA_WAIT;
            return;
        }
        _msg.Mxe.Data[0] = J1939_CTS_CONTROL_BYTE;
        _msg.Mxe.Data[1] = 2;
        _msg.Mxe.Data[2] = (TP_RX_MSG.packets_ok_num + 1);
        _msg.Mxe.Data[3] = J1939_RESERVED_BYTE;
        _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
        _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num >> 16) & 0xff);
        _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num >> 8 & 0xff);
        _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);

        /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
        J1939_EnqueueMessage(&_msg);
        TP_RX_MSG.state = J1939_TP_RX_DATA_WAIT;
        return;
    }
    else
    {
        /*Send the transmission normal end message, EndofMsgAck*/
        _msg.Mxe.Data[0] = J1939_EOMACK_CONTROL_BYTE;
        _msg.Mxe.Data[1] = (TP_RX_MSG.tp_rx_msg.byte_count & 0x00ff);
        _msg.Mxe.Data[2] = ((TP_RX_MSG.tp_rx_msg.byte_count >> 8) & 0x00ff);
        _msg.Mxe.Data[3] = TP_RX_MSG.packets_total;
        _msg.Mxe.Data[4] = J1939_RESERVED_BYTE;
        _msg.Mxe.Data[7] = (j1939_uint8_t)((pgn_num >> 16) & 0xff);
        _msg.Mxe.Data[6] = (j1939_uint8_t)(pgn_num >> 8 & 0xff);
        _msg.Mxe.Data[5] = (j1939_uint8_t)(pgn_num & 0xff);
        /*Maybe the queue is full and cannot be sent out, but here you cannot rely on the return value to wait infinitely*/
        J1939_EnqueueMessage(&_msg);
        TP_RX_MSG.state = J1939_RX_DONE;
        return;
    }
}
/**
* @note The heartbeat of the TP protocol, in order to meet the timing accuracy on the bus, poll once every 10ms J1939_TP_TX_RefreshCMTimer(10)\n
If you want a higher resolution, poll once every 1ms, but you need to change the following timing function J1939_TP_TX_RefreshCMTimer(1)
*/
void J1939_TP_Poll()
{
    if (J1939_TP_State_t == J1939_TP_NULL || J1939_TP_State_t == J1939_TP_OSBUSY)
    {
        return;
    }
    if (J1939_TP_State_t == J1939_TP_RX)
    {

        switch (TP_RX_MSG.state)
        {
        case J1939_TP_RX_WAIT:;
            break;
        case J1939_TP_RX_READ_DATA:
            /*Send read data TP.CM_CTS and EndofMsgAck message*/
            J1939_read_DT_Packet();
            break;
        case J1939_TP_RX_DATA_WAIT:
            /*Waiting for the message transmitted by the TP.DT frame*/
            if (J1939_TP_TIMEOUT_ABNORMAL == J1939_TP_RX_RefreshCMTimer(10))
            {
                /* Waiting for timeout, a connection exception occurs, jump to the abnormal step */
                TP_RX_MSG.state = J1939_TP_RX_ERROR;
            }
            break;
        case J1939_TP_RX_ERROR:
            J1939_TP_RX_Abort();

            break;
        case J1939_RX_DONE:
            TP_RX_MSG.packets_ok_num = 0;
            TP_RX_MSG.packets_total = 0;
            TP_RX_MSG.time = J1939_TP_T3;
            TP_RX_MSG.state = J1939_TP_RX_WAIT;
            J1939_TP_State_t = J1939_TP_NULL;
            break;
        default:
            break;
        }
        return;
    }
    if (J1939_TP_State_t == J1939_TP_TX)
    {

        switch (TP_TX_MSG.state)
        {
        case J1939_TP_TX_WAIT:
            /*There is no data to send*/
            break;
        case J1939_TP_TX_CM_START:
            /*Send the message transmitted by the TP.CM_RTS frame (refer to j1939-21)*/
            J1939_CM_Start();
            break;
        case J1939_TP_TX_CM_WAIT:
            /*Waiting for the message transmitted by the TP.CM_CTS frame*/
            if (J1939_TP_TIMEOUT_ABNORMAL == J1939_TP_TX_RefreshCMTimer(10))
            {
                /* Waiting for timeout, a connection exception occurs, jump to the abnormal step */
                TP_TX_MSG.state = J1939_TP_TX_ERROR;
            }
            break;
        case J1939_TP_TX_DT:
            J1939_TP_DT_Packet_send();
            break;
        case J1939_TP_WAIT_ACK:
            /*Waiting for the message transmitted by the TP.EndofMsgACK frame*/
            if (J1939_TP_TIMEOUT_ABNORMAL == J1939_TP_TX_RefreshCMTimer(10))
            {
                /* Waiting for timeout, a connection exception occurs, jump to the abnormal step */
                TP_TX_MSG.state = J1939_TP_TX_ERROR;
            }
            break;
        case J1939_TP_TX_ERROR:
            J1939_TP_TX_Abort();

            break;
        case J1939_TX_DONE:
            TP_TX_MSG.packets_request_num = 0;
            TP_TX_MSG.packet_offset_p = 0;
            TP_TX_MSG.time = J1939_TP_T3;
            TP_TX_MSG.state = J1939_TP_TX_WAIT;
            J1939_TP_State_t = J1939_TP_NULL;
            break;
        default:
            //The program will not run here, you can add a debug output
            break;
        }
        return;
    }
}

/**This is a non-blocking io interface
*
* @param[in] PGN TP session parameter group number
* @param[in] The destination address of the SA TP session
* @param[in] *data TP session data cache address
* @param[in] data_num TP session data size
* @return RC_SUCCESS successfully opened the TP link and started to enter the sending process
* @return RC_CANNOTTRANSMIT cannot be sent because the TP protocol has established a virtual link and has not been disconnected
* @note TP protocol sending function
*/
j1939_int8_t J1939_TP_TX_Message(j1939_uint32_t PGN, j1939_uint8_t DA, j1939_uint8_t *data, j1939_uint16_t data_num)
{
    j1939_uint16_t _byte_count = 0;
    /*Get sending permission*/
    if (J1939_TP_State_t == J1939_TP_NULL)
    {
        J1939_TP_State_t = J1939_TP_TX;
    }
    else
    {
        return RC_CANNOTTRANSMIT; //Cannot send, because the TP protocol has established a virtual link and has not been disconnected
    }

    TP_TX_MSG.tp_tx_msg.PGN = PGN;
    TP_TX_MSG.tp_tx_msg.SA = DA;
    TP_TX_MSG.tp_tx_msg.byte_count = data_num;
    for (_byte_count = 0; _byte_count < data_num; _byte_count++)
    {
        TP_TX_MSG.tp_tx_msg.data[_byte_count] = data[_byte_count];
    }
    TP_TX_MSG.packet_offset_p = 0;
    TP_TX_MSG.packets_request_num = 0;
    TP_TX_MSG.packets_total = data_num / 7;
    if ((data_num % 7) != 0)
    {
        TP_TX_MSG.packets_total++;
    }
    TP_TX_MSG.time = J1939_TP_T3;
    //Trigger start CM_START
    TP_TX_MSG.state = J1939_TP_TX_CM_START;

    return RC_SUCCESS;
}

/**
* @param[in] msg.data Read data cache
* @param[in] msg.data_num read data buffer size
* @param[out] msg.SA data source address
* @param[out] msg.byte_count data size
* @param[out] msg.PGN data parameter group number
* @return RC_CANNOTRECEIVE cannot be accepted, TP protocol is receiving data
* @return RC_SUCCESS read data successfully
* @note TP accepting function, the size of the receiving buffer must be greater than the size of the receiving data, it is recommended to initialize the buffer size with J1939_TP_MAX_MESSAGE_LENGTH\n
Please enter the size of the buffer area correctly, the parameter error program operation is risky
*/
j1939_int8_t J1939_TP_RX_Message(TP_RX_MESSAGE *msg)
{
    j1939_uint16_t _a = 0;
    /*Judge whether the data can be read*/
    if (J1939_TP_State_t == J1939_TP_NULL && TP_RX_MSG.tp_rx_msg.PGN != 0)
    {
        J1939_TP_State_t = J1939_TP_OSBUSY;
    }
    else
    {
        return RC_CANNOTRECEIVE; //Unacceptable, TP protocol is accepting data, or there is no data
    }
    //Judge whether to read the CAN data
    // if (_Can_Node != J1939_TP_Flags_t.TP_RX_CAN_NODE)
    // {
    //     /*Release TP takeover authority*/
    //     if (J1939_TP_State_t == J1939_TP_OSBUSY)
    //     {
    //         J1939_TP_State_t = J1939_TP_NULL;
    //     }
    //     return RC_CANNOTRECEIVE;
    // }
    //Judging whether the data cache is enough
    if ((msg->data_num) < TP_RX_MSG.tp_rx_msg.byte_count)
    {
        return RC_CANNOTRECEIVE; //Unacceptable, the buffer area is too small
    }

    /*retrieve data*/
    for (_a = 0; _a < msg->data_num; _a++)
    {
        msg->data[_a] = TP_RX_MSG.tp_rx_msg.data[_a];
    }
    /*Get data source address*/
    msg->SA = TP_RX_MSG.tp_rx_msg.SA;
    /*Get the size of the data*/
    msg->byte_count = TP_RX_MSG.tp_rx_msg.byte_count;
    /*Get data PGN*/
    msg->PGN = TP_RX_MSG.tp_rx_msg.PGN;

    /*Discard the read data*/
    TP_RX_MSG.tp_rx_msg.byte_count = 0u;
    TP_RX_MSG.tp_rx_msg.PGN = 0;

    /*Release TP takeover authority*/
    if (J1939_TP_State_t == J1939_TP_OSBUSY)
    {
        J1939_TP_State_t = J1939_TP_NULL;
    }

    return RC_SUCCESS;
}
/**
* @param[in] pgn The requested parameter group
* @param[in] DA destination address (DestinationAddress) When DA = 0xff, it means a global request
* @note request (from the global scope or specific destination) parameter group, page 16-17 of the request rule J1939-21, with clear instructions
*/
void J1939_Request_PGN(j1939_uint32_t pgn, j1939_uint8_t DA)
{
    J1939_MESSAGE _msg;

    _msg.Mxe.DataPage = 0;
    _msg.Mxe.Priority = J1939_REQUEST_PRIORITY;
    _msg.Mxe.DestinationAddress = DA;
    _msg.Mxe.DataLength = 3;
    _msg.Mxe.PDUFormat = J1939_PF_REQUEST;
    _msg.Mxe.Data[0] = (j1939_uint8_t)(pgn & 0x000000FF);
    _msg.Mxe.Data[1] = (j1939_uint8_t)((pgn & 0x0000FF00) >> 8);
    _msg.Mxe.Data[2] = (j1939_uint8_t)((pgn & 0x00FF0000) >> 16);

    while (J1939_EnqueueMessage(&_msg) != RC_SUCCESS)
        ;
}
/**
* @param[in] data Need to send data buffer
* @param[in] dataLenght The buffer size of the sent data
* @param[in] PGN needs to send data PGN (parameter group number)
* @param[in] void (*dataUPFun)() The function address pointer used to update the cache data
* @note Create a PGN request corresponding to the response\n If you receive a change request, run REQUEST_LIST.dataUPFun() first, and then send the data REQUEST_LIST.data
* @warning This function can only be called serially, (multi-threaded) parallel calls please add mutual exclusion operation to the function
*/
void J1939_Create_Response(j1939_uint8_t data[], j1939_uint16_t dataLenght, j1939_uint32_t PGN, void (*dataUPFun)())
{
    /*Find available linked list items*/
    struct Request_List *_requestList = &REQUEST_LIST;
    while (J1939_NULL != _requestList->next)
    {
        _requestList = _requestList->next;
    }
    _requestList->next = (struct Request_List *)malloc(sizeof(struct Request_List));
    _requestList = _requestList->next;

    /*Assign a value to the new linked list item*/
    _requestList->data = data;
    _requestList->lenght = dataLenght;
    _requestList->PGN = PGN;
    _requestList->update = dataUPFun;
    _requestList->next = J1939_NULL;
}


void dataUPFun()
{
	buffer[0]='b';
	buffer[1]='o';
	buffer[2]='a';
	buffer[3]='t';
	buffer[4]='s';

}

/**
* @note When receiving a PGN request, if there is a corresponding PGN in REQUEST_LIST, the PGN in REQUEST_LIST will be sent automatically. \n
  If not, a NACK will be sent; the response logic of this function, refer to J1939-21 page 17 Table 4
*/
void J1939_Response(const j1939_uint32_t PGN)
{
    J1939_MESSAGE _msg;

    /*Find available linked list items*/
    struct Request_List *_requestList = &REQUEST_LIST;
    while ((PGN != _requestList->PGN) )
    {
        if (_requestList->next == J1939_NULL)
        {
            /*The original document stipulates that NACK cannot be responded to when the global request is not supported*/
            if (OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
            {
                return;
            }
            if ((PGN & 0xFF00) >= 0xF000)
            {
                return;
            }

            /*No corresponding PGN response is created, send a NACK to the bus*/
            _msg.Mxe.Priority = J1939_ACK_PRIORITY;
            _msg.Mxe.DataPage = 0;
            _msg.Mxe.PDUFormat = J1939_PF_ACKNOWLEDGMENT;
            _msg.Mxe.DestinationAddress = OneMessage.Mxe.SourceAddress;
            _msg.Mxe.DataLength = 8;
            _msg.Mxe.SourceAddress = J1939_Address;
            _msg.Mxe.Data[0] = J1939_NACK_CONTROL_BYTE;
            _msg.Mxe.Data[1] = 0xFF;
            _msg.Mxe.Data[2] = 0xFF;
            _msg.Mxe.Data[3] = 0xFF;
            _msg.Mxe.Data[4] = 0xFF;
            _msg.Mxe.Data[5] = (PGN & 0x0000FF);
            _msg.Mxe.Data[6] = ((PGN >> 8) & 0x0000FF);
            _msg.Mxe.Data[7] = ((PGN >> 16) & 0x0000FF);

            SendOneMessage((J1939_MESSAGE *)&_msg);
            return;
        }
        else
        {
            _requestList = _requestList->next;
        }
    }

    /*Call dataUPFun() function, mainly used for parameter group data update*/
    if (J1939_NULL != _requestList->update)
    {
        _requestList->update();
    }

    /*Respond to request*/
    if (_requestList->lenght > 8)
    {
        /*An acknowledgment response multi-frame (non-broadcast multi-frame)*/
        if (RC_SUCCESS != J1939_TP_TX_Message(_requestList->PGN, OneMessage.Mxe.SourceAddress, _requestList->data, _requestList->lenght))
        {
            /*The original document stipulates that NACK cannot be responded to when the global request is not supported*/
            if (OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
            {
                return;
            }

            /*If the long frame is not sent successfully*/
            _msg.Mxe.Priority = J1939_ACK_PRIORITY;
            _msg.Mxe.DataPage = 0;
            _msg.Mxe.PDUFormat = J1939_PF_ACKNOWLEDGMENT;
            _msg.Mxe.DestinationAddress = OneMessage.Mxe.SourceAddress;
            _msg.Mxe.DataLength = 8;
            _msg.Mxe.SourceAddress = J1939_Address;
            _msg.Mxe.Data[0] = J1939_ACCESS_DENIED_CONTROL_BYTE;
            _msg.Mxe.Data[1] = 0xFF;
            _msg.Mxe.Data[2] = 0xFF;
            _msg.Mxe.Data[3] = 0xFF;
            _msg.Mxe.Data[4] = 0xFF;
            _msg.Mxe.Data[5] = (PGN & 0x0000FF);
            _msg.Mxe.Data[6] = ((PGN >> 8) & 0x0000FF);
            _msg.Mxe.Data[7] = ((PGN >> 16) & 0x0000FF);

            SendOneMessage((J1939_MESSAGE *)&_msg);
            return;
        }

        /*An acknowledgment response*/
        _msg.Mxe.Priority = J1939_ACK_PRIORITY;
        _msg.Mxe.DataPage = 0;
        _msg.Mxe.PDUFormat = J1939_PF_ACKNOWLEDGMENT;
        /*The original document stipulates that the global request responds to the global*/
        if (OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
        {
            _msg.Mxe.DestinationAddress = 0XFF;
        }
        else
        {
            _msg.Mxe.DestinationAddress = OneMessage.Mxe.SourceAddress;
        }
        _msg.Mxe.DataLength = 8;
        _msg.Mxe.SourceAddress = J1939_Address;
        _msg.Mxe.Data[0] = J1939_ACK_CONTROL_BYTE;
        _msg.Mxe.Data[1] = 0xFF;
        _msg.Mxe.Data[2] = 0xFF;
        _msg.Mxe.Data[3] = 0xFF;
        _msg.Mxe.Data[4] = 0xFF;
        _msg.Mxe.Data[5] = (PGN & 0x0000FF);
        _msg.Mxe.Data[6] = ((PGN >> 8) & 0x0000FF);
        _msg.Mxe.Data[7] = ((PGN >> 16) & 0x0000FF);
        SendOneMessage((J1939_MESSAGE *)&_msg);
    }
    else
    {

        /*An acknowledgment response*/
        _msg.Mxe.Priority = J1939_ACK_PRIORITY;
        _msg.Mxe.DataPage = 0;
        _msg.Mxe.PDUFormat = J1939_PF_ACKNOWLEDGMENT;
        _msg.Mxe.SourceAddress = J1939_Address;
        /*The original document stipulates that the global request responds to the global*/
        if ((OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS) || ((PGN & 0xFF00) >= 0xF000))
        {
            _msg.Mxe.DestinationAddress = 0XFF;
        }
        else
        {
            _msg.Mxe.DestinationAddress = OneMessage.Mxe.SourceAddress;
        }
        _msg.Mxe.DataLength = 8;
        _msg.Mxe.SourceAddress = J1939_Address;
        _msg.Mxe.Data[0] = J1939_ACK_CONTROL_BYTE;
        _msg.Mxe.Data[1] = 0xFF;
        _msg.Mxe.Data[2] = 0xFF;
        _msg.Mxe.Data[3] = 0xFF;
        _msg.Mxe.Data[4] = 0xFF;
        _msg.Mxe.Data[5] = (PGN & 0x0000FF);
        _msg.Mxe.Data[6] = ((PGN >> 8) & 0x0000FF);
        _msg.Mxe.Data[7] = ((PGN >> 16) & 0x0000FF);
        SendOneMessage((J1939_MESSAGE *)&_msg);

        /*Return a single frame of confirmation response*/
        _msg.Mxe.Priority = J1939_ACK_PRIORITY;
        _msg.Mxe.DataPage = (((_requestList->PGN) >> 16) & 0x1);
        _msg.Mxe.PDUFormat = ((_requestList->PGN) >> 8) & 0xFF;
        _msg.Mxe.SourceAddress = J1939_Address;
        /*The original document stipulates that the global request responds to the global*/
        if (OneMessage.Mxe.PDUSpecific == J1939_GLOBAL_ADDRESS)
        {
            _msg.Mxe.DestinationAddress = 0XFF;
        }
        else
        {
            _msg.Mxe.DestinationAddress = OneMessage.Mxe.SourceAddress;
        }
        _msg.Mxe.DataLength = _requestList->lenght;
        {
            j1939_uint8_t _i = 0;
            for (_i = 0; _i < (_requestList->lenght); _i++)
            {
                _msg.Mxe.Data[_i] = _requestList->data[_i];
            }
            for (; _i < 8; _i++)
            {
                _msg.Mxe.Data[_i] = 0xFF;
            }
        }
        SendOneMessage((J1939_MESSAGE *)&_msg);
    }
}
#endif
