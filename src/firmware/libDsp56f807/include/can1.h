/*
 * can1.h
 * The can bus interface (inspired by the metrowerks code).
 *
 */

#ifndef __can1h__
#define __can1h__

#include "dsp56f807.h"


#ifndef __BWUserType_CAN1_TError
#define __BWUserType_CAN1_TError
  typedef union {
    word err;
    struct {
      bool OverRun : 1;                /* Overrun error flag */
      bool : 1;
      bool : 1;
      bool BusOff : 1;                 /* Bus-off state */
      bool TxPassive : 1;              /* Transmitter error passive */
      bool RxPassive : 1;              /* Reciver error passive */
      bool TxWarning : 1;              /* Transmitter warning */
      bool RxWarning : 1;              /* Receiver warning */
    }errName;
  } CAN1_TError;                       /* Error flags. For languages which don't support bit access is byte access only to error flags possible. */
#endif

/* can bus message structure */
typedef struct canmsg_tag 
{
	byte 	CAN_data[16];					// CAN bus message 
	dword 	CAN_messID;						// message ID - arbitration 
	byte 	CAN_frameType;					// data or remote frame 
	byte 	CAN_frameFormat;				// standard or extended frame 
	byte 	CAN_length;
	byte 	CAN_ID_class;
	byte 	CAN_ID_src;					
	byte 	CAN_ID_dst;
} canmsg_t;



#define CAN_EI		setRegBit (CAN_RIER, RXFIE)
#define CAN_DI		clrRegBit (CAN_RIER, RXFIE) 
#define CAN_TX_EI   setRegBits (CAN_TCR, 7) // TXEIE2=1 TXEIE1=1 TXEIE0=1
#define CAN_TX_DI   clrRegBits (CAN_TCR, 7) // TXEIE2=1 TXEIE1=1 TXEIE0=1

#define CAN_BCAST_OVERFLOW			5	
#define CAN_BCAST_PRINT				6
#define CAN_TX_SOFTWARE_BUFFER_SIZE  10     //Number of messages that can be
									        // stored in a LIFO buffer
#define CAN_FIFO_LEN	20					/* length of the fifo buffer */

/* User constants */

/* Message filterring */
#define SINGLE_32_FILTER           0
#define TWO_16_FILTER              1
#define FOUR_8_FILTER              2
#define FILTER_CLOSED              3

/* Frame formats */
#define STANDARD_FORMAT            0
#define EXTENDED_FORMAT            1

/* Frame types   */
#define DATA_FRAME                 0
#define REMOTE_FRAME               1

#define CAN1_getStateTX() (getReg(CAN_TFLG) & 7)

byte CAN1_setAcceptanceMode (byte Mode);
byte CAN1_getStateRX (void);
byte CAN1_setAcceptanceCode (dword Mask, dword Mask1);
byte CAN1_getError (CAN1_TError *Err);
void CAN1_setHigh (void);
byte CAN1_send(dword MessageID,byte FrameType,byte Length,byte *Data);
byte CAN1_sendFrame (byte BufferNum, dword MessageID,byte FrameType,byte Length,byte *Data);
byte CAN1_sendFrameEx (byte BufferNum, dword MessageID,byte FrameType,byte Length,byte *Data);
byte CAN1_readFrame (dword *MessageID, byte *FrameType, byte *FrameFormat, byte *Length, byte *Data);
void CAN1_init (byte board_id);
byte CAN1_setAcceptanceMask (dword Mask, dword Mask1);
byte CAN1_getErrorValues (byte *rcv, byte *tx);
void CAN1_print_error(CAN1_TError *e);
byte getCanTXstatus();
void setCanTXstatus(byte val);
byte getCanStatus();
byte setCanStatus(byte val);
byte getCanBusOffstatus();
byte getTxError();
byte setTxError();
byte getRxError();
byte setRxError();

/*************************************************************************************/
/**
 * This method is used to print a string on the canbus
 *
 * @param string the string to be printed
 *************************************************************************************/
#define MAX_STRINGS 4
void can_print_string(char* text);

/*************************************************************************************/
/**
 * This method is used to print a dword on the canbus
 *
 * @param data the data to be printed
 *************************************************************************************/
void can_print_dword(dword data);

/*************************************************************************************/
/**
 * This method is used to print on can bus like printf
 *
 *************************************************************************************/
int can_printf(const char *format, ...);

/* isr's */
void CAN1_interruptTx (void);
void CAN1_interruptRx (void);
void CAN1_interruptError (void);
void CAN1_interruptWakeup (void);

void enable_can_print(); 
void disable_can_print(); 

#endif /* ifndef __can1h__ */
