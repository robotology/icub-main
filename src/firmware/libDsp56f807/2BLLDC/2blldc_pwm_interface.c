#include "dsp56f807.h"
#include "pwm_interface.h"
#include "pwm_a.h"
#include "pwm_b.h"
#include "faults_interface.h"
#include "leds_interface.h"
#include "can1.h"
#include "brushess_comm.h"
bool _pad_enabled[2] = {0,0};
extern sDutyControlBL DutyCycle[2];
extern sDutyControlBL DutyCycleReq[2];
/***************************************************************************/
/**
 * This method inits the PWM interface for the brushless board
 ***************************************************************************/
void init_pwm(void)
{
	
	PWM_A_init ();
	PWM_B_init ();	
}

/***************************************************************************/
#define ALL_CHANNELS 0x3F00
#define CHANNELS_01  0x0300
#define CHANNELS_23  0x0C00
#define CHANNELS_45  0x3000

/***************************************************************************/
/**
 * This method disables the PWM generation on the specified channel
 * @param axis is the axis number
 ***************************************************************************/
void PWM_outputPadDisable(byte axis)
{
	can_printf("PWM DIS CH%d", axis);
	
	if (axis == 0)	
	{
		PWM_generate_DC(0, 0, 0);
		PWM_A_outputPadDisable(ALL_CHANNELS);
		_pad_enabled[0]= false;
		led0_off
	}
	else    		
	{
    	PWM_generate_DC(1, 0, 0);
		_pad_enabled[1]= false;
		PWM_B_outputPadDisable(ALL_CHANNELS);	
		led2_off
	}
	FaultInterruptDisable(axis);
  
}

/***************************************************************************/
/**
 * This method enables the PWM generation on the specified channel
 * @param axis is the axis number
 ***************************************************************************/
void PWM_outputPadEnable(byte axis)
{
//	#ifdef DEBUG_CAN_MSG
	can_printf("PWM ENA CH%d", axis);
//	#endif
	if (axis == 0)
	{
		PWM_generate_DC(0, 0, 0);
		PWM_A_outputPadEnable(ALL_CHANNELS);
		_pad_enabled[0]= true;
		led0_on
	}
	else    		
	{
		
		PWM_generate_DC(1, 0, 0);
		PWM_B_outputPadEnable(ALL_CHANNELS);
		_pad_enabled[1]= true;
		led2_on
	}
	FaultInterruptEnable(axis);		
}



void PWM_generate (byte i, Int16 pwm_value)
{
	if (pwm_value>0)
		{
			DutyCycleReq[i].Dir = 0;
			DutyCycleReq[i].Duty=pwm_value;	
		}						
	else 
		{
			DutyCycleReq[i].Dir = 1;
			DutyCycleReq[i].Duty=-pwm_value;
		} 
}

