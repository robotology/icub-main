/*
 * ti1.c
 * implementation of the timer interface.
 *
 */

/**
 * \file ti1.c
 *	implementation of the timer interrupt interface.
 */
 
#include "ti1.h"
#include "asc.h"
#include "controller.h"

extern bool _wait;

/**
 * set the comparator value (when to trigger the interrupt).
 * @param Val is the comparator value (15 bits).
 */
static void setCV (word Val)
{
	setReg (TMRA0_CMP1, Val);
}

/*
 * sets the primary count source.
 * @param Val is the count source (e.g. in pin, clock, etc.).
 *
 */
static void setPV(byte Val)
{
	clrRegBits (TMRA0_CTRL, 0x1e00);
	setRegBits (TMRA0_CTRL, Val << 9);
	setReg (TMRA0_CNTR, 0); /* Reset counter */
}

/**
 * initializes the counter/timer. the timer is initialized w/ 1ms period.
 */
void TI1_init (void)
{
	/* TMRA0_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,Co_INIT=0,OM=0 */
	setReg (TMRA0_CTRL, 0x20);           /* Stop all functions of the timer */

	/* TMRA0_SCR: TCF=0,TCFIE=1,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
	setReg (TMRA0_SCR, 0x4000);
	setReg (TMRA0_CNTR, 0);                /* Reset counter register */
	setReg (TMRA0_LOAD, 0);                /* Reset load register */
	setCV (39999);                         /* Store appropriate value to the compare register according to the selected high speed CPU mode */
	setPV (8);                             /* Set prescaler register according to the selected high speed CPU mode */
	
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
	if (!_wait)
		AS1_sendCharSafe ('%');
	_wait = false;
	//TI1_OnInterrupt();                   /* Invoke user event */
}

/**
 * TimerC init, this is used to sync the ADC with the PWM.
 */
void TIC_init (void)
{
	setReg (TMRC2_CMP1, 256);           /* Set the Compare register 1 */ 
	setReg (TMRC2_CMP2, 0);             /* Set the Compare register 2 */ 
	setReg (TMRC2_LOAD, 0);             /* Set the Load register */ 
	setReg (TMRC2_CNTR, 0);             /* Set the Counter register */ 
	
	setReg (TMRC2_SCR, 0);              /* Set the Status and control register */ 
	setReg (TMRC2_CTRL, 0x2423);        /* Set the Control register */ 
	
	setReg (TMRC3_CMP1, 256);           /* Set the Compare register 1 */ 
	setReg (TMRC3_CMP2, 0);             /* Set the Compare register 2 */ 
	setReg (TMRC3_LOAD, 0);             /* Set the Load register */ 
	setReg (TMRC3_CNTR, 0);             /* Set the Counter register */ 
	
	setReg (TMRC3_SCR, 0);              /* Set the Status and control register */ 
	setReg (TMRC3_CTRL, 0x2623);        /* Set the Control register */ 
} 
