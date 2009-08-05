/*
 * can1.c
 *
 */

/**
 *
 * \file can1.c contains the implementation of the CAN bus interface.
 *
 */   
 
#include "can1.h"
#include "asc.h"
#include <stdarg.h>  

canmsg_t can_fifo[CAN_FIFO_LEN];
Int16 write_p = 0;
Int16 read_p = -1;

/// CAN TX BUFFER 
static Int16 canTxBufferIndex;
static canmsg_t canTxBuffer[CAN_TX_SOFTWARE_BUFFER_SIZE]; 
static UInt8 TxError=0;
static UInt8 RxError=0;
static UInt8 BusOff=0;
static byte status=0;
byte	can_board_id;

bool print_enable = false;

#define CAN_MAX_DATA_LEN      8        /* Max number of data to be sent in one frame */
#define FULL_RX_BUF           1        /* RX buffer full           */

#define EXTENDED_FRAME_ID     2047     /* Max value of the standard frame ID */
#define MB_ID_IDE             0x00080000UL
#define CAN_TX_MBUFFERS       3        /* Number of TX buffers */

/**************************************************************
 **
 * can_print globals 
 **************************************************************/
canmsg_t _print_canmsg;
#define CAN_BUFFER write_buffer
#define CAN_DATA _print_canmsg.CAN_data
#define CAN_FRAME_TYPE _print_canmsg.CAN_frameType
#define CAN_LEN _print_canmsg.CAN_length
#define CAN_ID _print_canmsg.CAN_messID

#define PAD_RIGHT 1
#define PAD_ZERO 2
#define PRINT_BUF_LEN 12

int buf_pointer=0;
char buffer [100];
byte bufferfull=0;
/**
 * the TMsgBuff structure.
 */
typedef struct
{
	word IDR0;
	word IDR1;
	word IDR2;
	word IDR3;
	word Data[8];
	word DLR;
	word TBPR;
	word Reserved;
}
TMsgBuff;

const word TXMsgBuffer[CAN_TX_MBUFFERS] = { (word)CAN_TB0_IDR0, (word)CAN_TB1_IDR0, (word)CAN_TB2_IDR0 }; /* TX message buffer addresses */

static byte ErrFlag;                   /* Error flags mirror of the status register */
static byte SerFlag;                   /* Flags for CAN communication */
                                       /* Bits: 0 - OverRun error */
                                       /*       1 - Framing error */
                                       /*       2 - Parity error */
                                       /*       3 - Full RX buffer */
                                       /*       4 - Full TX buffer */

/**
 * gets the ID out of the received IDR registers.
 * @return the ID (11 or 29 bits).
 */
static inline dword Idr2Id (dword idr)
{
	if (idr & MB_ID_IDE)
		return ((idr >> 1) & 0x3FFFFUL) | ((idr >> 3) & 0x1FFC0000); /* Extended frame */
	else
		return idr >> 21;                  /* Standard frame */
}


/**
 * gets the IDR from the CAN registers.
 * @return the IDR in a dword.
 */
static inline dword GetRxBufferIdr (void)
{
	dword Idr;

	Idr = getReg(CAN_RB_IDR0);
	Idr <<= 8;
	Idr |= getReg(CAN_RB_IDR1);
	Idr <<= 8;
	Idr |= getReg(CAN_RB_IDR2);
	Idr <<= 8;
	Idr |= getReg(CAN_RB_IDR3);
	return Idr;
}

/**
 * converts an ID into an IDR.
 * @param id is the ID.
 * @return the IDR registers corresponding to the ID.
 */
static inline dword Id2Idr (dword id)
{
	if (id > EXTENDED_FRAME_ID)
		return (((id & 0x1FFC0000UL) << 3) | 0x00180000UL | ((id & 0x0003FFFFUL) << 1)); /* Extended frame */
	else
		return id << 21;                   /* Standard frame */
}

/**
 * prepares the message TX ID.
 * @param idr is the IDR value.
 * @param TxBuff is the TX buffer struct.
 */
static inline void SetTxBufferIdr (dword idr, TMsgBuff *TxBuff)
{
	TxBuff->IDR3 = idr & 0x00FF;
	idr >>=8;
	TxBuff->IDR2 = idr & 0x00FF;
	idr >>=8;
	TxBuff->IDR1 = idr & 0x00FF;
	idr >>=8;
	TxBuff->IDR0 = idr & 0x00FF;
}

/**
 * set the acceptance mode schema used.
 * @param Mode is one of SINGLE_32_FILTER, TWO_16_FILTER, 
 * FOUR_8_FILTER, or FILTER_CLOSED.
 * @return ERR_OK if succeeds, ERR_SPEED or ERR_DISABLED otherwise.
 */
byte CAN1_setAcceptanceMode (byte Mode)
{
	if (Mode > 3)                        /* Is mode parameter greater then 3 */
		return ERR_VALUE;                /* If yes then error */
		
	EnterCritical ();
	setRegBit (CAN_CTL0, SFTRES);         /* Disable device, soft reset */
	clrRegBits (CAN_IDAC, 0x0030);
	*(char *)(CAN_IDAC) |= (Mode << 4);
	clrRegBit (CAN_CTL0, SFTRES);         /* Start device */
	setRegBits (CAN_RFLG, 0xfe);          /* Reset error flags, doesn't reset the RXF flag */
	setReg (CAN_RIER, 0xff);              /* Enable interrupts, all of them! */
	ExitCritical ();
	
	return ERR_OK;                       /* OK */
}

/**
 * get the error counts.
 * @param rcv is a pointer to the receiver error count.
 * @param tx is a pointer to the transmitter error count.
 * @return ERR_OK always.
 */
byte CAN1_getErrorValues (byte *rcv, byte *tx)
{
	EnterCritical ();
	setRegBit (CAN_CTL0, SFTRES);         /* Disable device, soft reset */
	
	*rcv = getReg (CAN_RXERR);
	*tx = getReg (CAN_TXERR);
	
	clrRegBit (CAN_CTL0, SFTRES);         /* Start device */
	setRegBits (CAN_RFLG, 0xfe);          /* Reset error flags, doesn't reset the RXF flag */
	setReg (CAN_RIER, 0xff);              /* Enable interrupts, all of them! */
	ExitCritical ();
	
	return ERR_OK;                       /* OK */
} 
 
/**
 * gets the state of the receive buffer.
 * @return 1 if the RX buffer is full.
 */
byte CAN1_getStateRX (void)
{
  return ((SerFlag & FULL_RX_BUF) != 0)? 1:0;
}


/**
 * sets the CAN filter RX acceptance code (Mask is masked by the
 * mask - see CAN1_setAcceptanceMask() - and messages are accepted only if they match
 * with this Mask). Note that bytes are stored as BYTE2 BYTE3 BYTE0 BYTE1 in the Mask.
 * @param Mask is the code to check messages against.
 * @return ERR_OK if successful.
 */
byte CAN1_setAcceptanceCode (dword Mask, dword Mask1)
{
	dword tmpMask;
	dword tmpMask1;
	EnterCritical ();                     /* Enter critical section */
	tmpMask = Mask;
	tmpMask1 = Mask1;
	setRegBit (CAN_CTL0, SFTRES);          /* Disable device */
	setReg (CAN_IDAR0, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CAN_IDAR4, (byte)tmpMask1);     /* Set acceptance mask register */
	tmpMask>>=8;
	tmpMask1>>=8;
	setReg (CAN_IDAR1, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CAN_IDAR5, (byte)tmpMask1);     /* Set acceptance mask register */
	tmpMask>>=8;
	tmpMask1>>=8;
	setReg (CAN_IDAR2, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CAN_IDAR6, (byte)tmpMask1);     /* Set acceptance mask register */
	tmpMask>>=8;
	tmpMask1>>=8;
	setReg (CAN_IDAR3, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CAN_IDAR7, (byte)tmpMask1);     /* Set acceptance mask register */
	clrRegBit (CAN_CTL0, SFTRES);          /* Start device */
	setRegBits (CAN_RFLG, 254);            /* Reset error flags */
	setReg (CAN_RIER, 255);                /* Enable interrupts */
	ExitCritical ();                      /* Exit critical section */
	
	return ERR_OK;                       /* OK */
}


/**
 * returns the error status of the CAN bus.
 * @param Err a pointer to the CAN1_TError struct to contain the error status.
 * @return ERR_OK if successful.
 */
byte CAN1_getError (CAN1_TError *Err)
{
	Err->err = 0;                        /* Clear all errors */
	Err->errName.BusOff =  ((ErrFlag & CAN_RFLG_BOFFIF_MASK) != 0); /* Bus-Off state */
	Err->errName.TxPassive = ((ErrFlag & CAN_RFLG_TERRIF_MASK) != 0); /* Transmitter error */
	Err->errName.RxPassive = ((ErrFlag & CAN_RFLG_RERRIF_MASK) != 0); /* Receiver error */
	Err->errName.TxWarning = ((ErrFlag & CAN_RFLG_TWRNIF_MASK) != 0); /* Transmitter warning */
	Err->errName.RxWarning = ((ErrFlag & CAN_RFLG_RWRNIF_MASK) != 0); /* Receiver warning */
	Err->errName.OverRun = ((ErrFlag & CAN_RFLG_OVRIF_MASK) != 0); /* Receiver overrun */
	ErrFlag = 0;                         /* Clear error flags */
	
	return ERR_OK;
}

/**
 * initializes the CAN bus by, clearing the SFTRES, clearing error flags, and enabling
 * all interrupts.
 */
static void HWEnDi (void)
{
	clrRegBit (CAN_CTL0, SFTRES);          /* Start device */
	setRegBits (CAN_RFLG, 254);            /* Reset error flags */
	setReg (CAN_RIER, 255);                /* Enable error and receive interrupts */
}

/**
 * initializes the CAN bus timing registers. This is hardwired to 1 mbit/s for now.
 */
void CAN1_setHigh (void)
{
	setReg (CAN_BTR0, 65);                 /* Set the device timing register */
	setReg (CAN_BTR1, 122);                /* Set the device timing register */
	setRegBit (CAN_CTL1, CLKSRC);          /* Select the clock source from bus clock */
	HWEnDi ();                              /* Enable/disable device according to status flags */
}

/**
 * sends a message. The ID determines whether is a STANDARD or EXTENDED message (11 or 29 bits).
 * @param BufferNum the buffer number to use (0, 1, 2).
 * @param MessageID is the ID of the message (aka arbitration ID).
 * @param FrameType is either DATA_FRAME of REMOTE_FRAME.
 * @param Length is the lenght of the message (0 to 8).
 * @param Data is the pointer to Data (8 bytes).
 */
byte CAN1_sendFrameEx (byte BufferNum, 
					 dword MessageID,
					 byte FrameType,
					 byte Length,
					 byte *Data)
{
	register byte i;      
	register byte bufmask = ((word)1 << BufferNum);
	register TMsgBuff *MsgBuff;

	if ( (BufferNum > CAN_TX_MBUFFERS-1) || (Length > CAN_MAX_DATA_LEN) )
		return ERR_VALUE;
		
	if (FrameType > REMOTE_FRAME)
		return ERR_VALUE;

	if (!(getReg(CAN_TFLG) & bufmask))
		return ERR_TXFULL;
		
	MsgBuff = (TMsgBuff *)TXMsgBuffer[BufferNum];
	
	EnterCritical();                     			/* Disable global interrupts */
	SetTxBufferIdr (Id2Idr(MessageID), MsgBuff); 	/* Set the message ID */
	if (FrameType == DATA_FRAME) 
	{
		for (i=0; i<Length; i++)
			MsgBuff->Data[i] = Data[i];      		/* Store data to the transmit register */
			
		if (MessageID > EXTENDED_FRAME_ID)
			MsgBuff->IDR3 &= 254;            		/* If no then set message type as "data frame" */
		else
			MsgBuff->IDR1 &= 239;            		/* If yes then set message type as "data frame" */
	}
	else 
	{                               				/* Remote frame */
		if (MessageID > EXTENDED_FRAME_ID)
			MsgBuff->IDR3 |= 1;             	 	/* If yes then set message type as "remote frame" */
		else
			MsgBuff->IDR1 |= 16;             		/* If yes then set message type as "remote frame" */
	}
	
	MsgBuff->DLR = Length;               			/* Set the length of the message */
	MsgBuff->TBPR = 0;                   			/* Set the priority (high) */
	setReg(CAN_TFLG, bufmask);            			/* Start transmission */
	
	ExitCritical();                      			/* Enable global interrupts */
	
	return ERR_OK;
}


/**
 * copy the message to the canTxBuffer. 
 *
 * @param MessageID is the ID of the message (aka arbitration ID).
 * @param FrameType is either DATA_FRAME of REMOTE_FRAME.
 * @param Length is the lenght of the message (0 to 8).
 * @param Data is the pointer to Data (8 bytes).
 */

byte CAN1_send(dword MessageID,byte FrameType,byte Length,byte *Data)
{
	byte i=0;
	Int16 tout=0;
	
	CAN_TX_DI;
	if (canTxBufferIndex<CAN_TX_SOFTWARE_BUFFER_SIZE-1)
	{
		
		canTxBufferIndex++;
		canTxBuffer[canTxBufferIndex].CAN_messID=MessageID;
		canTxBuffer[canTxBufferIndex].CAN_frameType=FrameType;
		canTxBuffer[canTxBufferIndex].CAN_length=Length;
		for(i=0;i<Length;i++)
		{
			canTxBuffer[canTxBufferIndex].CAN_data[i]=Data[i];	
		}
		CAN_TX_EI;
		return ERR_OK;
	}
	else 
	{
		canTxBufferIndex=-1;
		setCanTXstatus(1);
		return ERR_TXFULL;
	}
}

byte getCanTXstatus()
{
	return bufferfull;
}

void setCanTXstatus(byte val)
{
	bufferfull=val;
}

byte getCanBusOffstatus()
{
	return BusOff;
}

byte getCanStatus()
{
	return status;
}
byte setCanStatus(byte val)
{
	status=val;
	setCanTXstatus(val); //reset of the can tx status
}

byte getTxError()
{
	return TxError;
}
byte setTxError()
{
	TxError=0;
}
byte getRxError()
{
	return RxError;
}
byte setRxError()
{
	RxError=0;
}


/**
 * sends a message. The ID determines whether is a STANDARD or EXTENDED message (11 or 29 bits).
 * the buffer is chosen automatically. At least one of the three buffers must be available.
 * @param BufferNo is NOT USED.
 * @param MessageID is the ID of the message (aka arbitration ID).
 * @param FrameType is either DATA_FRAME of REMOTE_FRAME.
 * @param Length is the lenght of the message (0 to 8).
 * @param Data is the pointer to Data (8 bytes).
 */
byte CAN1_sendFrame (byte BufferNo, dword MessageID,
					   byte FrameType,
					   byte Length,
					   byte *Data)
{
	register byte i;      
	register byte bufmask = ((word)1);
	register TMsgBuff *MsgBuff;
	register byte BufferNum = 0;
	
	static   byte StaticBuffer = 0x1;
	
	if ( (BufferNum > CAN_TX_MBUFFERS-1) || (Length > CAN_MAX_DATA_LEN) )
		return ERR_VALUE;
		
	if (FrameType > REMOTE_FRAME)
		return ERR_VALUE;

	StaticBuffer = getReg(CAN_TFLG) & 0x07;
	
	if (StaticBuffer & 0x1)
	{
		BufferNum = 0;
		bufmask = 0x1;
	}
	else
	if (StaticBuffer & 0x2)
	{
		BufferNum = 1;
		bufmask = 0x2;
	}
	else
	if (StaticBuffer & 0x4)
	{
		BufferNum = 2;
		bufmask = 0x4;
	}
	else	
		return ERR_TXFULL;
		
	MsgBuff = (TMsgBuff *)TXMsgBuffer[BufferNum];
	
	EnterCritical();                     			/* Disable global interrupts */
	SetTxBufferIdr (Id2Idr(MessageID), MsgBuff); 	/* Set the message ID */
	if (FrameType == DATA_FRAME) 
	{
		for (i=0; i<Length; i++)
			MsgBuff->Data[i] = Data[i];      		/* Store data to the transmit register */
			
		if (MessageID > EXTENDED_FRAME_ID)
			MsgBuff->IDR3 &= 254;            		/* If no then set message type as "data frame" */
		else
			MsgBuff->IDR1 &= 239;            		/* If yes then set message type as "data frame" */
	}
	else 
	{                               				/* Remote frame */
		if (MessageID > EXTENDED_FRAME_ID)
			MsgBuff->IDR3 |= 1;             	 	/* If yes then set message type as "remote frame" */
		else
			MsgBuff->IDR1 |= 16;             		/* If yes then set message type as "remote frame" */
	}
	
	MsgBuff->DLR = Length;               			/* Set the length of the message */
	MsgBuff->TBPR = 0;                   			/* Set the priority (high) */
	
	setReg(CAN_TFLG, bufmask);            			/* Start transmission */
	
	ExitCritical();                      			/* Enable global interrupts */
	
	return ERR_OK;
}


/**
 * reads a message.
 * @param MessageID is a pointer to the ID of the message (also arbitration ID).
 * @param FrameType is a pointer to the type of the message (either DATA_FRAME or REMOTE_FRAME).
 * @param FrameFormat is a pointer to the format (either STANDARD_FORMAT or EXTENDED_FORMAT).
 * @param Lemgth is a pointer to the length of the message.
 * @param Data is a pointer to the data buffer.
 */
byte CAN1_readFrame (dword *MessageID,
					 byte *FrameType,
					 byte *FrameFormat,
					 byte *Length,
					 byte *Data)
{
	byte i;
	dword ID;

	if (!SerFlag & FULL_RX_BUF)
		return ERR_RXEMPTY;
	
	ID = Idr2Id(GetRxBufferIdr());       /* Read the identification of the received message */
	
	if (ID > EXTENDED_FRAME_ID)
		*FrameType = (getReg(CAN_RB_IDR3) & 1)? REMOTE_FRAME : DATA_FRAME;
	else
		*FrameType = (getReg(CAN_RB_IDR1) & 16)? REMOTE_FRAME : DATA_FRAME;
	
	*MessageID = ID;
	*FrameFormat = (getReg(CAN_RB_IDR1) & 8)? EXTENDED_FORMAT : STANDARD_FORMAT;
	*Length = getReg(CAN_RB_DLR) & 15;
	if (*FrameType == DATA_FRAME) 
	{
		for (i = 0; i < *Length; i++) /* should be checking max len of the message */
			Data[i] = *((byte *)CAN_RB_DSR0 + i);
	}
	
	SerFlag &= ~FULL_RX_BUF;             /* Clear flag "full RX buffer" */
	
	return ERR_OK;
}


/**
 * initializes the CAN bus. It must be called before anything else.
 * 
 */ 
void CAN1_init (byte board_id)
{
	can_board_id = board_id;
	canTxBufferIndex=-1;
	/* CANCTL0: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,RXFRM=0,RXACT=0,CSWAI=0,SYNCH=0,??=0,SLPAK=0,SLPRQ=0,SFTRES=1 */
	setReg(CAN_CTL0, 0x0001);

	/* CANCTL1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CANE=1,??=0,??=0,??=0,??=0,LOOPB=0,WUPM=0,CLKSRC=0 */
	setReg(CAN_CTL1, 0x0080);

	clrRegBits (CAN_IDAC, 0x0030); /* mode = 01 */
	setRegBits (CAN_IDAC, 0x0010);
	
	/* resets all acceptance codes */
	setReg(CAN_IDAR0, 0);
	setReg(CAN_IDAR1, 0);
	setReg(CAN_IDAR2, 0);
	setReg(CAN_IDAR3, 0);
	setReg(CAN_IDAR4, 0);
	setReg(CAN_IDAR5, 0);
	setReg(CAN_IDAR6, 0);
	setReg(CAN_IDAR7, 0);
	setReg(CAN_IDMR0, 0);
	setReg(CAN_IDMR1, 0);
	setReg(CAN_IDMR2, 0);
	setReg(CAN_IDMR3, 0);
	setReg(CAN_IDMR4, 0);
	setReg(CAN_IDMR5, 0);
	setReg(CAN_IDMR6, 0);
	setReg(CAN_IDMR7, 0);
	
	CAN1_setHigh();                      /* Initial speed CPU mode is high */
	setCanTXstatus(0);
}


/** 
 * sets the acceptance mask.
 * @param Mask bits that are 0 must match those on the acceptance code for
 * a message to be received.
 * @return ERR_OK if successful.
 */
byte CAN1_setAcceptanceMask (dword Mask0, dword Mask1)
{
	dword tmpMask0;
	dword tmpMask1;

	tmpMask0 = Mask0;
	tmpMask1 = Mask1;	
	setRegBit (CAN_CTL0, SFTRES);          /* Disable device */
	
	setReg (CAN_IDMR0, (byte)tmpMask0);     /* Set acceptance mask register */
	setReg (CAN_IDMR4, (byte)tmpMask1);     /* Set acceptance mask register */
	tmpMask0 >>= 8;
	tmpMask1 >>= 8;
	setReg (CAN_IDMR1, (byte)tmpMask0);     /* Set acceptance mask register */
	setReg (CAN_IDMR5, (byte)tmpMask1);     /* Set acceptance mask register */
	tmpMask0 >>= 8;
	tmpMask1 >>= 8;
	setReg (CAN_IDMR2, (byte)tmpMask0);     /* Set acceptance mask register */
	setReg (CAN_IDMR6, (byte)tmpMask1);     /* Set acceptance mask register */
	tmpMask0 >>= 8;
	tmpMask1 >>= 8;
	setReg (CAN_IDMR3, (byte)tmpMask0);     /* Set acceptance mask register */
	setReg (CAN_IDMR7, (byte)tmpMask1);     /* Set acceptance mask register */
	
	clrRegBit (CAN_CTL0, SFTRES);          /* Start device */
	setRegBits (CAN_RFLG, 0xfe);           /* Reset error flags */
	setReg (CAN_RIER, 0xff);               /* Enable interrupts */
	
	return ERR_OK;
}


/**
 * the TX isr routine. Beware that register protection is not enabled, use
 * #pragma interrupt saveall to save all registers in the stack.
 */
#pragma interrupt saveall
void CAN1_interruptTx (void)
{
	
	byte buffer = getReg(CAN_TFLG) & 7;           /* Temporary variable */
	CAN_TX_DI;
	if (canTxBufferIndex!=-1)
	{
		CAN1_sendFrame (buffer, canTxBuffer[canTxBufferIndex].CAN_messID,
					   canTxBuffer[canTxBufferIndex].CAN_frameType,
					   canTxBuffer[canTxBufferIndex].CAN_length,
					   canTxBuffer[canTxBufferIndex].CAN_data);	
		canTxBufferIndex--;
		if (canTxBufferIndex!=-1)
			CAN_TX_EI;
	}

}


/**
 * the RX isr routine.
 */
#pragma interrupt saveall
void CAN1_interruptRx (void)
{
	canmsg_t *p;
	
	// setReg(CANRFLG, CANRFLG_RXF_MASK);   /* Reset the reception complete flag */
	SerFlag |= FULL_RX_BUF;              /* Set flag "full RX buffer" */
	
	CAN_DI;
	write_p ++;
	if (write_p >= CAN_FIFO_LEN)
		write_p = 0;

	// check here for buffer full.
	p = can_fifo + write_p;
	CAN1_readFrame (&(p->CAN_messID), 
				  &(p->CAN_frameType), 
				  &(p->CAN_frameFormat), 
				  &(p->CAN_length), 
				  p->CAN_data);
#if 0
	if (read_p != -1)
	{

	}
	else
#endif
	
	if (read_p == -1)
	{
		read_p = write_p;
	
	}

	setReg (CAN_RFLG, CAN_RFLG_RXF_MASK);
	CAN_EI;
}

/**
 * the error ISR.
 */
#pragma interrupt saveall
void CAN1_interruptError (void)
{
	CAN_DI;
	
	if getRegBits(CAN_RFLG, CAN_RFLG_TERRIF_MASK)
	{
		TxError =1;
	}
	if getRegBits(CAN_RFLG, CAN_RFLG_RERRIF_MASK)
	{
	    RxError =1;
	}
	if getRegBits(CAN_RFLG, CAN_RFLG_BOFFIF_MASK)
	{
		BusOff=1;
	}
	status=0;
	status=getCanTXstatus() & 0x01;
	status|= ((BusOff>0)<<1); /* Bus-Off state */
	status|= ((TxError>0)<<2); /* Transmitter error */
    status|= ((RxError>0)<<3); /* Receiver error */
	status|= ((getRegBits(CAN_RFLG, CAN_RFLG_TWRNIF_MASK) != 0)<<4); /* Transmitter warning */
	status|= ((getRegBits(CAN_RFLG, CAN_RFLG_RWRNIF_MASK) != 0)<<5); /* Receiver warning */
	status|= ((getRegBits(CAN_RFLG, CAN_RFLG_OVRIF_MASK)  != 0)<<6); /* Receiver overrun */
    setRegBits(CAN_RFLG,0xfe);

	CAN_EI;
}

/**
 * the wakeup ISR.
 */
#pragma interrupt saveall
void CAN1_interruptWakeup (void)
{
	CAN_DI;

	setRegBits (CAN_RFLG, 0xfe);           /* Reset error flags */
	CAN_EI;
}


/**************************************************************
 **
 * print_can_error 
 **************************************************************/
#define SEND_BOOL(x) ((x) ? AS1_printStringEx("1") : AS1_printStringEx("0"))

void CAN1_print_error(CAN1_TError *e)
{
	AS1_printStringEx ("f: ");
	SEND_BOOL(e->errName.BusOff);
	SEND_BOOL(e->errName.TxPassive);
	SEND_BOOL(e->errName.RxPassive);
	SEND_BOOL(e->errName.TxWarning);
	SEND_BOOL(e->errName.RxWarning);
	SEND_BOOL(e->errName.OverRun);
	AS1_printStringEx ("\r\n");
}

/***********************************************************************/
void can_print_string(char* text)
{
	byte j = 0;
	byte write_buffer = 0;
    int  can_blocks = 0;
    int  can_last_block = 0;
    int  counter = 0;
    int  dataSize = 0;
    static byte string_id=0; 
    
    if (!(print_enable)) return;
    	
    while (text[dataSize] != '\0')
	{
		dataSize++;
		if (j<6) { j++;  }
		else {	j=1; can_blocks++; }
  	}
  	can_last_block=j;
   
   	CAN_ID = 0x100;
	CAN_ID |= (can_board_id) << 4;
	CAN_ID |= CAN_BCAST_PRINT;
		
    for (counter = 0 ; counter < can_blocks+1; counter++)
    {
		{  
			CAN_DATA[0]=CAN_BCAST_PRINT;
			CAN_DATA[1]=counter+(string_id<<4);
										
			if (counter < can_blocks)
				{
					CAN_LEN = 8;
					for (j=0; j<6; j++)	
						CAN_DATA[2+j] = text[j+counter*6];
				}
			else
				{
					CAN_DATA[0]+=128;
					CAN_LEN = can_last_block+2;
					for (j=0; j<can_last_block; j++)
						CAN_DATA[2+j] = text[j+counter*6];
				}	
							
			CAN1_send(CAN_ID, CAN_FRAME_TYPE, CAN_LEN, CAN_DATA); 
		} 	
    }
    
    string_id++;
	if (string_id==MAX_STRINGS) string_id=0; 	
}

/***********************************************************************/
void can_print_dword(dword data)
{
	dword dwrd;
	dword sign;
	char c[10];
	int i;
	
	if (!(print_enable)) return;
		
	sign = 0x80000000 & data;
	if (sign)
	{
		data = -data;
		c[0] = '-';
	}
	else
		c[0] = '+';
	
	for (i = 0; i < 9; i++)
	{
		dwrd = data % 10;
		data /= 10;
		c[9-i] = dwrd + 48;
	}

	can_print_string (c);	
}

/***********************************************************************/
static void printchar(char **str, int c)
{
	if (buf_pointer>254) return;
	buffer[buf_pointer++]=c;
}

/***********************************************************************/
static int prints(char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (out, padchar);
		++pc;
	}

	return pc;
}

/***********************************************************************/
static int printi(char **out, Int32 i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register Int32 t;
	byte neg = 0;
	register int pc = 0;
	//register unsigned int u = i;
	dword u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			printchar (out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (out, s, width, pad);
}

/***********************************************************************/
static int print(char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = (char *)va_arg( args, Int16 );
				pc += prints (out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (out, va_arg( args, Int16 ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'f' ) {
				pc += printi (out, va_arg( args, Int32 ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (out, va_arg( args, Int16 ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (out, va_arg( args, Int16 ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (out, va_arg( args, Int16 ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				scr[0] = (char)va_arg( args, Int16 );
				scr[1] = '\0';
				pc += prints (out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			printchar (out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

/***********************************************************************/
int can_printf(const char *format, ...)
{	
	int ret=0;
	va_list args;

	if (!(print_enable)) return 0;
//	if (!(broadcast_mask & 0x20)) return 0;
	
	va_start( args, format );
	
	buf_pointer=0;
	ret = print( 0, format, args );	
	buffer[buf_pointer]=0;
	
	can_print_string(buffer);
		
	return ret;
}

/***********************************************************************/
void enable_can_print()
{
	print_enable=true;
}

/***********************************************************************/
void disable_can_print()
{
	print_enable=false;
}
