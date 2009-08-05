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

#include "PE_Types.h"

/*****************************************************************************/
#define  COM_TIMEOUT_VALUE    ((UWord16)(ZCLOCK_FREQUENCY / 800000ul)* 10)
//#define  COM_TIMEOUT_INIT_SECOND ((UWord16)(ZCLOCK_FREQUENCY / 800000ul)* 500)

/*****************************************************************************/
extern void comMainLoop                 ( void );
extern void comACK            			( byte cmd );
extern void comExit                     ( void );
extern void comSendType					( void );
extern void comWaitEnd					( void );

extern unsigned int TmpXdataVar;

#define   CMD_BOARD 	0
#define   CMD_ADDRESS   1
#define   CMD_START		2	
#define   CMD_DATA		3
#define   CMD_END		4
#define   CMD_ERR		5
#define   CMD_BROADCAST 0xFF

#endif /* !defined(__COM_H) */
