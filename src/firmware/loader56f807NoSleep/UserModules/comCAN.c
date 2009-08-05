#include "Cpu.h"
#include "CAN1.h"
#include "PE_Types.h"

#include "bootloader.h"
#include "comCAN.h"
#include "sparser.h"
#include "prog.h"
#include "flashprog.h"
#include "Timer.h"

#define  COM_BUFFER_LEN    20

static UWord16  comIndex;
static UWord16  comReadLength = 1;
static bool     comContinue;
static byte 	CAN_data[8];					/* CAN bus message */

bool   comTimerEn;
canmsg_t comBuffer;

extern dword 	CAN_messID;

////////////////////////////////////////////////////////////////////////
//
// comMainLoop()
//
// Communication loop. Wait for CAN frames and parses the messagge.
// The loop ends when comContinue is false.
//
////////////////////////////////////////////////////////////////////////
void comMainLoop(void)
{
	byte ReadCANState;
	byte i;
	comContinue = TRUE;
	comTimerEn = TRUE;
	while (comContinue)
	{
		/* wait CAN data or while timer expired, if set */
		while (CAN1_getStateRX() == 0) 
		{
			// Timer check
			if (comTimerEn) 
			{
		        if (getRegBit(TMRC1_SCR,TCF))      /* Is the interrupt request flag set? */
		        {
					clrRegBit(TMRC1_SCR,TCF);        /* If yes then reset this flag */
					if (--TmpXdataVar == 0) 
					{
						setRegBitGroup(TMRC0_CTRL,CM,0);   /* Stop counter */
						return;
					}
				}
			}
		}

		if (CAN1_getStateRX() != 0)
		{
			/* Received CAN frame */
			ReadCANState=CAN1_readFrame (&comBuffer.CAN_messID,
										  &comBuffer.CAN_length,
										  comBuffer.CAN_data);
			if ((ReadCANState != ERR_OK) && (ReadCANState != ERR_RXEMPTY))
			{
				userError(INDICATE_ERROR_RECEIVE);
			}
			sprsReady(&comBuffer);      /* call CAN protocol parser */
		}
	
	}  /* while */
}

////////////////////////////////////////////////////////////////////////
//
// comExit()
//
// End communication loop.
//
////////////////////////////////////////////////////////////////////////
void comExit (void)
{
	comContinue = FALSE;
}

////////////////////////////////////////////////////////////////////////
//
// comACK()
//
// Send acknowledge message.
//
////////////////////////////////////////////////////////////////////////
void comACK (byte cmd)
{
	CAN_data[0] = cmd;
	CAN_data[1] = 1;
	CAN1_sendFrame (CAN_messID, 2, CAN_data);	
}

////////////////////////////////////////////////////////////////////////
//
// comSendType()
//
// Send board characteristic: board type, version e revision of firmware.
//
////////////////////////////////////////////////////////////////////////
void comSendType(void)
{
	CAN_data[0] = CMD_BROADCAST;
	CAN_data[1] = 0;		// board type (always 0 for motor control card). 
	CAN_data[2] = _board_FW_VER >> 8;  // firmware version.
	CAN_data[3] = _board_FW_VER & 0xff; // firmware revision.
	CAN1_sendFrame (CAN_messID, 4, CAN_data);	
}

////////////////////////////////////////////////////////////////////////
//
// comWaitEnd()
//
// Wait the end of trasmission
//
////////////////////////////////////////////////////////////////////////
void comWaitEnd(void)
{
	do 
	{
		while (CAN1_getStateRX() == 0);
		CAN1_readFrame (&comBuffer.CAN_messID,
						&comBuffer.CAN_length,
						comBuffer.CAN_data);
	}
	while ((comBuffer.CAN_data[0]==CMD_START) ||
			(comBuffer.CAN_data[0]==CMD_END));
}			