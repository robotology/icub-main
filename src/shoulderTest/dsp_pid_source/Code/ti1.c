/*
 * ti1.c
 * implementation of the timer interface.
 *
 */
 
#include "ti1.h"

volatile bool _wait = true;	

/**
 * initializes the counter/timer. the timer is initialized w/ 1ms period.
 */
void TI1_init (void)
{
	/* TMRA0_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,Co_INIT=0,OM=0 */
	setReg (TMRA0_CTRL, 0x20);           /* Stop all functions of the timer */

	/* TMRA0_SCR: TCF=0,TCFIE=1,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
	setReg (TMRA0_SCR, 0x4000);
	setReg (TMRA0_LOAD, 0);                /* Reset load register */
	setReg (TMRA0_CMP1, 39999);            /* Store appropriate value to the compare register according to the selected high speed CPU mode */

	clrRegBits (TMRA0_CTRL, 0x1e00);
	setRegBits (TMRA0_CTRL, 0x08 << 9);    /* Set prescaler register according to the selected high speed CPU mode */
	setReg 	   (TMRA0_CNTR, 0); 		   /* Reset counter */
		
	clrRegBits (TMRA0_CTRL, 0xe000);
	setRegBits (TMRA0_CTRL, 0x2000);	   /* counter on! */
}

/**
 * isr timer. 
 */
#pragma interrupt
void TI1_interrupt (void)
{
	clrRegBit (TMRA0_SCR, TCF);            /* Reset interrupt request flag */
	_wait = false;
}

