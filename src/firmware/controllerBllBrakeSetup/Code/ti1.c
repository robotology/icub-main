/*
 * ti1.c
 * implementation of the timer interface.
 *
 */
 
#include "ti1.h"
#include "controller.h"
#include "currents_interface.h"
#include "pid.h"
#include "pwm_interface.h"
#include "can1.h"


volatile bool _wait = true;	
volatile byte _count;
volatile UInt8 highcurrent[2];
/**
 * initializes the counter/timer. the timer is initialized w/ 1ms period.
 */
void TI1_init (void)
{
	/* TMRA3_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,Co_INIT=0,OM=0 */
	setReg (TMRA3_CTRL, 0x20);           /* Stop all functions of the timer */

	/* TMRA3_SCR: TCF=0,TCFIE=1,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
	setReg (TMRA3_SCR, 0x4000);
	setReg (TMRA3_LOAD, 0);                /* Reset load register */
	setReg (TMRA3_CMP1, 39999);            /* Store appropriate value to the compare register according to the selected high speed CPU mode */

	clrRegBits (TMRA3_CTRL, 0x1e00);
	setRegBits (TMRA3_CTRL, 4096);    /* Set prescaler register according to the selected high speed CPU mode */
	setReg 	   (TMRA3_CNTR, 0); 		   /* Reset counter */
		
	clrRegBits (TMRA3_CTRL, 0xe000);
	setRegBits (TMRA3_CTRL, 0x2000);	   /* counter on! */
}

/**
 * isr timer. 
 */
#pragma interrupt
void TI1_interrupt (void)
{
	byte i;
	clrRegBit (TMRA3_SCR, TCF);            /* Reset interrupt request flag */
	_count++;
	_wait = false;
	highcurrent[0]= 0;
	highcurrent[1]= 0;
			for (i=0; i<JN; i++) 
		{
			check_current(i, (_pid[i] > 0));		
			compute_filtcurr(i);
			if (_filt_current[i] > _max_allowed_current[i])
			{
				_control_mode[i] = MODE_IDLE;	
				_pad_enabled[i] = false;
				highcurrent[i]= 1;
				PWM_outputPadDisable(i);
#ifdef DEBUG_CAN_MSG
				can_printf("ERR: ax%d _high curr DIS PWM",i);
#endif	
			}			
		}
	
}

// The function get the counter value of Timer1
Int16 TI1_getCounter()
{
	return getReg(TMRA3_CNTR);
}

