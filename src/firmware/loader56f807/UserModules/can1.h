/*
 * can1.h
 * The can bus interface (inspired by the metrowerks code).
 *
 */

#ifndef __can1h__
#define __can1h__

#include "PE_Types.h"

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

#define CAN1_getStateTX() (getReg(CANTFLG) & 7)

byte CAN1_getStateRX (void);
byte CAN1_setFilter (dword Mask0,dword Mask1,dword Code0,dword Code1,byte Mode);
void CAN1_setHigh (void);
byte CAN1_sendFrame (dword MessageID, byte Length,byte *Data);
byte CAN1_readFrame (dword *MessageID, byte *Length, byte *Data);
void CAN1_init (void);






#endif /* ifndef __can1h__ */
