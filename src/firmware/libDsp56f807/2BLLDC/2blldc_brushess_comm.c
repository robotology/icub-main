
#include "brushess_comm.h"
#include "phase_hall_sens.h"
#include "pwm_interface.h"
#include "pwm_a.h"
#include "pwm_b.h"
#include "asc.h"
#include "can1.h"
#include "leds_interface.h"
#include "currents_interface.h"
#include "AD.h"

// variabili per l'inseguimento del duty cycle
sDutyControlBL DutyCycle[2];
sDutyControlBL DutyCycleReq[2];

Int32 comm_count[2] = {0, 0};
void TD0_Enable(void)
{
	setRegBits (TMRD0_CTRL, 0x2000);
}

void TD0_Disable(void)
{
	clrRegBits (TMRD0_CTRL, 0x2000);	
}

//**********************************************************************
/**
 * initializes the counter/timer. the timer is initialized w/ 1ms period.
 */
void TD0_init (void)
{
	/* TMRD0_CTRL: CM=0,PCS=0,SCS=0,ONCE=0,LENGTH=1,DIR=0,Co_INIT=0,OM=0 */
	setReg (TMRD0_CTRL, 0x20);           /* Stop all functions of the timer */

	/* TMRD0_SCR: TCF=0,TCFIE=1,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
	setReg (TMRD0_SCR, 0x4000);
	setReg (TMRD0_LOAD, 0);                /* Reset load register */
	setReg (TMRD0_CMP1, 39999);            /* Store appropriate value to the compare register according to the selected high speed CPU mode */

	clrRegBits (TMRD0_CTRL, 0x1e00);
	setRegBits (TMRD0_CTRL, 4096);    /* Set prescaler register according to the selected high speed CPU mode */
	setReg 	   (TMRD0_CNTR, 0); 		   /* Reset counter */
		
	clrRegBits (TMRD0_CTRL, 0xe000);
	TD0_Enable();						   /* counter on! */
}

/**
 * isr timer. 
 */
#pragma interrupt saveall
void TD0_interrupt(void)
{
	UInt8 tmp,val;
	static Int16 counter = 0;
	
	if (DutyCycleReq[0].Duty < MIN_DUTY)
		DutyCycleReq[0].Duty=MIN_DUTY;
	if (DutyCycleReq[0].Duty > MAX_DUTY)
		DutyCycleReq[0].Duty=MAX_DUTY;
	if (DutyCycleReq[1].Duty < MIN_DUTY)
		DutyCycleReq[1].Duty=MIN_DUTY;
	if (DutyCycleReq[1].Duty > MAX_DUTY)
		DutyCycleReq[1].Duty=MAX_DUTY;

	if (DutyCycleReq[0].Dir != DutyCycle[0].Dir) 
	{
		if (DutyCycle[0].Duty <= (MIN_DUTY)) 
		{
			DutyCycle[0].Dir = DutyCycleReq[0].Dir;
		}		
		else 
		{	
			DutyCycle[0].Duty=DutyCycle[0].Duty-STEP;
		}
	}
	else {
			if (DutyCycleReq[0].Duty > DutyCycle[0].Duty) 
			{
				if (DutyCycleReq[0].Duty-DutyCycle[0].Duty>=STEP)
					DutyCycle[0].Duty=DutyCycle[0].Duty+STEP;
				else
					DutyCycle[0].Duty=DutyCycleReq[0].Duty;
			}
			else if (DutyCycleReq[0].Duty < DutyCycle[0].Duty) 
			{
				if (DutyCycle[0].Duty-DutyCycleReq[0].Duty>=STEP)
					DutyCycle[0].Duty=DutyCycle[0].Duty-STEP;
				else
					DutyCycle[0].Duty=DutyCycleReq[0].Duty;
			}		
	}
	
	//++++++
	
	if (DutyCycleReq[1].Dir != DutyCycle[1].Dir) 
	{
		if (DutyCycle[1].Duty <= (MIN_DUTY)) 
		{
			DutyCycle[1].Dir = DutyCycleReq[1].Dir;
		}		
		else 
		{
			DutyCycle[1].Duty=DutyCycle[1].Duty-STEP;
		}
	}
	else {
	 
			if (DutyCycleReq[1].Duty > DutyCycle[1].Duty) 
			{
				if (DutyCycleReq[1].Duty-DutyCycle[1].Duty>=STEP)
					DutyCycle[1].Duty=DutyCycle[1].Duty+STEP;
				else
					DutyCycle[1].Duty=DutyCycleReq[1].Duty;
			}
			else if (DutyCycleReq[1].Duty < DutyCycle[1].Duty) 
			{
				if (DutyCycle[1].Duty-DutyCycleReq[1].Duty>=STEP)
					DutyCycle[1].Duty=DutyCycle[1].Duty-STEP;
				else
					DutyCycle[1].Duty=DutyCycleReq[1].Duty;
			}
		
	}


	
//	DutyCycle[0].Duty=DutyCycleReq[0].Duty;
//	DutyCycle[0].Dir=DutyCycleReq[0].Dir;
	
//	DutyCycle[1].Duty=DutyCycleReq[1].Duty;
//	DutyCycle[1].Dir=DutyCycleReq[1].Dir;
	
	PWM_generate_DC(0, DutyCycle[0].Duty,DutyCycle[0].Dir); 
	PWM_generate_DC(1, DutyCycle[1].Duty,DutyCycle[1].Dir); 
	
		
	clrRegBit (TMRD0_SCR, TCF);            /* Reset interrupt request flag */
	
}

//*********************************************************
void Init_Brushess_Comm()
{
	UInt8 tmp=0;
	UInt8 nop=0;
	word temp=0;
	DutyCycle[0].Duty = MIN_DUTY;
	DutyCycle[0].Dir = 0;
	DutyCycleReq[0].Duty = MIN_DUTY;
	DutyCycleReq[0].Dir = 0;
	
	DutyCycle[1].Duty = MIN_DUTY;
	DutyCycle[1].Dir = 0;
	DutyCycleReq[1].Duty = MIN_DUTY;
	DutyCycleReq[1].Dir = 0;
	
	//Init PWM
    init_pwm();
	
	//Init Current
	init_currents();
	
	// Init duty cycle timer
	TD0_init();
	
}

/***************************************************************************/
/**
 * This method generates the PWM signal for a 2 BRUSHESS board
 * This must be used to give the dutycycle to the motor.
 * The function does not put immediately the value to the PWM channel, but 
 * it change the value of the DutyCycleReq. The DutyCycle will be increased or 
 * decreased until the reaching of the DutyCycleReq value by 1 unit each 1 ms. 
 * @param i is the axis number, can be 0/1
 * @param pwm_value is the duty cicle of the pwm (in clock ticks)
 ***************************************************************************/

void PWM_generate_DC(byte i, Int16 pwm_value, byte dir)
{
	if (pwm_value < MIN_DUTY) pwm_value = MIN_DUTY;
	if (pwm_value > (MAX_DUTY)) pwm_value = (MAX_DUTY);
	if (dir == 0)
		{
			if (i==0)
			{
				PWM_A_setDuty (0, (unsigned char)(pwm_value & 0x7fff));
				PWM_A_setDuty (2, 0);
				PWM_A_setDuty (4, 0);
			}
			else
			{
				PWM_B_setDuty (0, (unsigned char)(pwm_value & 0x7fff));
				PWM_B_setDuty (2, 0);
				PWM_B_setDuty (4, 0);		
			}
		}
	else
		{
			if (i==0)
			{
				PWM_A_setDuty (0, 0);
				PWM_A_setDuty (2, (unsigned char)((pwm_value) & 0x7fff));
				PWM_A_setDuty (4, 0);
			}
			else
			{
				PWM_B_setDuty (0, 0);
				PWM_B_setDuty (2, (unsigned char)((pwm_value) & 0x7fff));
				PWM_B_setDuty (4, 0);				
			}
		}
	
	if  (i==0)  PWM_A_load();
	else		PWM_B_load();
}

