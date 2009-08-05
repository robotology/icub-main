#include "dsp56f807.h"
#include "pwm_interface.h"

#include "pwm_a.h"
#include "pwm_b.h"

#include "leds_interface.h"

/***************************************************************************/
/**
 * This method inits the PWM interface for the 2DC motor board
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
	if (axis == 0)	PWM_A_outputPadDisable(ALL_CHANNELS);
	else    		PWM_B_outputPadDisable(ALL_CHANNELS);
}

/***************************************************************************/
/**
 * This method enables the PWM generation on the specified channel
 * @param axis is the axis number
 ***************************************************************************/
void PWM_outputPadEnable(byte axis)
{
	if (axis == 0)	PWM_A_outputPadEnable(ALL_CHANNELS);
	else    		PWM_B_outputPadEnable(ALL_CHANNELS);
}

/***************************************************************************/
/**
 * This method generates the PWM signal for a 2 DC board
 * @param i is the axis number, can be 0/1
 * @param pwm_value is the duty cicle of the pwm (in clock ticks)
 ***************************************************************************/
void PWM_generate(byte i, Int16 pwm_value)
{
	if (pwm_value >= 0)
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
				PWM_A_setDuty (2, (unsigned char)((-pwm_value) & 0x7fff));
				PWM_A_setDuty (4, 0);
			}
			else
			{
				PWM_B_setDuty (0, 0);
				PWM_B_setDuty (2, (unsigned char)((-pwm_value) & 0x7fff));
				PWM_B_setDuty (4, 0);				
			}
		}
	
	if  (i==0)  PWM_A_load();
	else		PWM_B_load();
}
