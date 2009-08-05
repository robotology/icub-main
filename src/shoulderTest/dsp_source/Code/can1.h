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
byte CAN1_sendFrame (byte BufferNum, dword MessageID,byte FrameType,byte Length,byte *Data);
byte CAN1_sendFrameEx (byte BufferNum, dword MessageID,byte FrameType,byte Length,byte *Data);
byte CAN1_readFrame (dword *MessageID, byte *FrameType, byte *FrameFormat, byte *Length, byte *Data);
void CAN1_init (void);
byte CAN1_setAcceptanceMask (dword Mask, dword Mask1);
byte CAN1_getErrorValues (byte *rcv, byte *tx);

/* isr's */
void CAN1_interruptTx (void);
void CAN1_interruptRx (void);
void CAN1_interruptError (void);
void CAN1_interruptWakeup (void);



#endif /* ifndef __can1h__ */
