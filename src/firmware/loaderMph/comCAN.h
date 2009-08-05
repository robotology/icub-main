/*****************************************************************************
*
* Freescale Inc.
* (c) Copyright 2000 Freescale, Inc.
* ALL RIGHTS RESERVED.
*
******************************************************************************
*
* File Name:         com.h
*
* Description:       Global parameters and functions for communication modulle
*
*****************************************************************************/
#if !defined(__COM_H)
#define __COM_H

// void comMainLoop                 ( void );
// void comACK            			( int8 cmd );
// void comExit                     ( void );
// void comSendType					( void );
// void comWaitEnd					( void );

// void userError(int Error);

//extern unsigned int TmpXdataVar;

#define   CMD_BOARD 	0
#define   CMD_ADDRESS   1
#define   CMD_START		2
#define   CMD_DATA		3
#define   CMD_END		4
#define   CMD_ERR		5
#define   CMD_BROADCAST 0xFF

#define INDICATE_ERROR_RECEIVE   1
#define INDICATE_ERROR_CHARACTER 2
#define INDICATE_ERROR_FORMAT    3
#define INDICATE_ERROR_CHECKSUM  4
#define INDICATE_ERROR_OVERRUN   5
#define INDICATE_ERROR_FLASH     6
#define INDICATE_ERROR_INTERNAL  7
#define INDICATE_ERROR_PARITY    8
#define INDICATE_ERROR_PROTECTED_BOOT_SECTION 9


/* can bus message structure */
typedef struct canmsg_tag
{
	int8 	CAN_data[8];					/* CAN bus message */
	int32 	CAN_messID;						/* message ID - arbitration */
	int8 	CAN_frameType;					/* data or remote frame */
	int8 	CAN_frameFormat;				/* standard or extended frame */
	int8 	CAN_length;						/* len of the data */
} canmsg_t;



#endif /* !defined(__COM_H) */
