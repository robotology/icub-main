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
#include "controller.h"
#include "IO_Map.h"
#include "PE_Error.h"

#define CAN_MAX_DATA_LEN      8        /* Max number of data to be sent in one frame */
#define FULL_RX_BUF           1        /* RX buffer full           */

#define EXTENDED_FRAME_ID     2047     /* Max value of the standard frame ID */
#define MB_ID_IDE             0x00080000UL
#define CAN_TX_MBUFFERS       3        /* Number of TX buffers */

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

//const word TXMsgBuffer[CAN_TX_MBUFFERS] = { (word)CAN_TB0_IDR0, (word)CAN_TB1_IDR0, (word)CAN_TB2_IDR0 }; /* TX message buffer addresses */
//const word *TXMsgBuffer[CAN_TX_MBUFFERS] = { CAN_TB0_IDR0, CAN_TB1_IDR0, CAN_TB2_IDR0 }; /* TX message buffer addresses */

static byte ErrFlag;                   /* Error flags mirror of the status register */
static byte SerFlag;                   /* Flags for CAN communication */
                                       /* Bits: 0 - OverRun error */
                                       /*       1 - Framing error */
                                       /*       2 - Parity error */
                                       /*       3 - Full RX buffer */
                                       /*       4 - Full TX buffer */




/**
 * gets the IDR from the CAN registers.
 * @return the IDR in a dword.
 */
static inline dword GetRxBufferId (void)
{
	dword Idr;
	Idr = getReg(CAN_RB_IDR0);
	Idr <<= 3;
	Idr |= (getReg(CAN_RB_IDR1) >>5);
	return Idr;
}

/**
 * prepares the message TX ID.
 * @param idr is the IDR value.
 * @param TxBuff is the TX buffer struct.
 */
static inline void SetTxBufferIdr (dword idr, TMsgBuff *TxBuff)
{
	TxBuff->IDR1 = (idr & 0x0007) << 5;
	TxBuff->IDR0 = (idr >> 3) & 0x00FF;
}


/**
 * gets the state of the receive buffer.
 * @return 1 if the RX buffer is full.
 */
byte CAN1_getStateRX (void)
{
  SerFlag=getReg(CANRFLG);
  return ((SerFlag & FULL_RX_BUF) != 0)? 1:0;
}

/**
 * initializes the CAN bus by, clearing the SFTRES, clearing error flags, and enabling
 * all interrupts.
 */
static void HWEnDi (void)
{
	clrRegBit (CANCTL0, SFTRES);          /* Start device */
	setRegBits (CANRFLG, 254);            /* Reset error flags */
	setReg (CANRIER, 255);                /* Enable error and receive interrupts */
}


/** 
 * sets the acceptance mask.
 * @param Mask bits that are 0 must match those on the acceptance code for
 * a message to be received.
 * @return ERR_OK if successful.
 */
 /**
 * sets the CAN filter RX acceptance code (Mask is masked by the
 * mask - see CAN1_setAcceptanceMask() - and messages are accepted only if they match
 * with this Mask). Note that bytes are stored as BYTE2 BYTE3 BYTE0 BYTE1 in the Mask.
 * @param Mask is the code to check messages against.
 * @return ERR_OK if successful.
 */

byte CAN1_setFilter (dword Mask0,dword Mask1,dword Code0,dword Code1,byte Mode)
{
	dword tmpMask;

	tmpMask = Mask0;
	setRegBit (CANCTL0, SFTRES);          /* Disable device */
	
	setReg (CANIDMR0, (byte)tmpMask);     /* Set acceptance mask register */
	tmpMask>>=8;	
	setReg (CANIDMR1, (byte)tmpMask);     /* Set acceptance mask register */
	tmpMask = Mask1;
	setReg (CANIDMR4, (byte)tmpMask);     /* Set acceptance mask register */
	tmpMask>>=8;	
	setReg (CANIDMR5, (byte)tmpMask);     /* Set acceptance mask register */
	tmpMask = 0xFF;
	setReg (CANIDMR2, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CANIDMR6, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CANIDMR3, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CANIDMR7, (byte)tmpMask);     /* Set acceptance mask register */
	tmpMask = Code0;
	setReg (CANIDAR0, (byte)tmpMask);     /* Set acceptance mask register */
	tmpMask>>=8;	
	setReg (CANIDAR1, (byte)tmpMask);     /* Set acceptance mask register */	
	tmpMask = Code1;
	setReg (CANIDAR4, (byte)tmpMask);     /* Set acceptance mask register */
	tmpMask>>=8;	
	setReg (CANIDAR5, (byte)tmpMask);     /* Set acceptance mask register */	
	tmpMask = 0x00;
	setReg (CANIDAR2, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CANIDAR6, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CANIDAR3, (byte)tmpMask);     /* Set acceptance mask register */
	setReg (CANIDAR7, (byte)tmpMask);     /* Set acceptance mask register */
	
	clrRegBits (CANIDAC, 0x0030);
	*(char *)(CANIDAC) |= (Mode << 4);


	HWEnDi();
	
	return ERR_OK;
}



/**
 * initializes the CAN bus timing registers. This is hardwired to 1 mbit/s for now.
 */
void CAN1_setHigh (void)
{
	setReg (CANBTR0, 65);                 /* Set the device timing register */
	setReg (CANBTR1, 122);                /* Set the device timing register */
	setRegBit (CANCTL1, CLKSRC);          /* Select the clock source from bus clock */
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
byte CAN1_sendFrame (dword MessageID,
					 byte Length,
					 byte *Data)
{
	register byte i;      
	register TMsgBuff *MsgBuff;

	if (Length > CAN_MAX_DATA_LEN) 
		return ERR_VALUE;
		
	if (!(getReg(CANTFLG) & 0x2))
		return ERR_TXFULL;

	MsgBuff = (TMsgBuff *)(&CAN_TB1_IDR0);
	
//	(TMsgBuff *)TXMsgBuffer[BufferNum];
	
//	EnterCritical();                     			/* Disable global interrupts */
	SetTxBufferIdr (MessageID, MsgBuff); 	/* Set the message ID */
	for (i=0; i<Length; i++)
		MsgBuff->Data[i] = Data[i];      		/* Store data to the transmit register */
			
	MsgBuff->IDR1 &= 239;            		/* If yes then set message type as "data frame" */
	
	MsgBuff->DLR = Length;               			/* Set the length of the message */
	MsgBuff->TBPR = 0;                   			/* Set the priority (high) */
	setReg(CANTFLG, 0x2);            			/* Start transmission */
	
//	ExitCritical();                      			/* Enable global interrupts */
	
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
					 byte *Length,
					 byte *Data)
{
	byte i;
	volatile word *pt;
	dword ID;

	if (!SerFlag & FULL_RX_BUF)
		return ERR_RXEMPTY;
	setReg(CANRFLG, CANRFLG_RXF_MASK);   /* Reset the reception complete flag */
	
	ID = GetRxBufferId();       /* Read the identification of the received message */
	
	*MessageID = ID;
	*Length = getReg(CAN_RB_DLR) & 15;
	pt = &CAN_RB_DSR0;
	for (i = 0; i < *Length; i++) /* should be checking max len of the message */
		Data[i] = *pt++;
	SerFlag &= ~FULL_RX_BUF;             /* Clear flag "full RX buffer" */
	
	return ERR_OK;
}


/**
 * initializes the CAN bus. It must be called before anything else.
 * 
 */ 
void CAN1_init (void)
{
	/* CANCTL0: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,RXFRM=0,RXACT=0,CSWAI=0,SYNCH=0,??=0,SLPAK=0,SLPRQ=0,SFTRES=1 */
	setReg(CANCTL0, 0x0001);

	/* CANCTL1: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,CANE=1,??=0,??=0,??=0,??=0,LOOPB=0,WUPM=0,CLKSRC=0 */
	setReg(CANCTL1, 0x0080);

//	clrRegBits (CANIDAC, 0x0030); /* mode = 0 */

	/* resets all acceptance codes */
/*	setReg(CANIDAR0, 0);
	setReg(CANIDAR1, 0);
	setReg(CANIDAR2, 0);
	setReg(CANIDAR3, 0);
	setReg(CANIDAR4, 0);
	setReg(CANIDAR5, 0);
	setReg(CANIDAR6, 0);
	setReg(CANIDAR7, 0);
	setReg(CANIDMR0, 0);
	setReg(CANIDMR1, 0);
	setReg(CANIDMR2, 0);
	setReg(CANIDMR3, 0);
	setReg(CANIDMR4, 0);
	setReg(CANIDMR5, 0);
	setReg(CANIDMR6, 0);
	setReg(CANIDMR7, 0);*/
	
	CAN1_setHigh();                      /* Initial speed CPU mode is high */
}





