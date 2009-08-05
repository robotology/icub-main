#include "dsp56f807.h"
#include "pwm_interface.h"

#include "pwm_a.h"
#include "pwm_b.h"

#include "leds_interface.h"
#include "faults_interface.h"
bool _pad_enabled[4] = {false,false,false,false};
/***************************************************************************/
/**
 * This method inits the PWM interface for the 4DC motor board
 ***************************************************************************/
void init_pwm(void)
{
	//digital outputs to enable L6206 IC
	setRegBits(GPIO_D_DDR,0xF);   
	clrRegBits(GPIO_D_PER,0xF); 
	clrRegBits(GPIO_D_DR, 0xF); 	
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
#pragma interrupt called
void PWM_outputPadDisable(byte axis)
{
 	if (axis == 0)	
 	{	
 		clrRegBits(GPIO_D_DR,0x1); 
 		PWM_A_setDuty (0, 0);
 		PWM_A_setDuty (1, 0);
 		PWM_A_load();

 		PWM_A_outputPadDisable(CHANNELS_01);
 		_pad_enabled[0]= false;
 		turn_led_off(1);	
 	}
 	else if (axis == 1)
 	{

 		clrRegBits(GPIO_D_DR,0x2); 
 		PWM_A_setDuty (2, 0);
 		PWM_A_setDuty (3, 0);
 		PWM_A_load();
 		_pad_enabled[1]= false;
 		PWM_A_outputPadDisable(CHANNELS_23);
 		turn_led_off(1);	
 	}
	else if (axis == 2)
	{
		clrRegBits(GPIO_D_DR,0x4);
		PWM_B_setDuty (0, 0);
 		PWM_B_setDuty (1, 0);
 		PWM_B_load();
 		_pad_enabled[2]= false;
		PWM_B_outputPadDisable(CHANNELS_01);	
	 

		turn_led_off(1);	
	}
	else if (axis == 3)
	{
		clrRegBits(GPIO_D_DR,0x8); 
		PWM_B_setDuty (2, 0);
 		PWM_B_setDuty (3, 0);
 		PWM_B_load();
 		_pad_enabled[3]= false;
		PWM_B_outputPadDisable(CHANNELS_23);
		turn_led_off(1);	
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
	Int16 status=0;
	status = getReg (PWMA_PMFSA);
		
 	if ((axis == 0)	&& !(status & PWMA_PMFSA_FPIN3_MASK)) 
 	{
 		PWM_A_setDuty (0, 0);
 		PWM_A_setDuty (1, 0);
 		PWM_A_load();
 		_pad_enabled[0]= true;
 		PWM_A_outputPadEnable(CHANNELS_01);
 		setRegBits(GPIO_D_DR,0x1); 
 		turn_led_on(1);	
 	}
 	else if ((axis == 1)	&& !(status & PWMA_PMFSA_FPIN3_MASK)) 
 	{
 		PWM_A_setDuty (2, 0);
 		PWM_A_setDuty (3, 0);
 		PWM_A_load();
 		_pad_enabled[1]= true;
 		PWM_A_outputPadEnable(CHANNELS_23);
 		setRegBits(GPIO_D_DR,0x2); 
 		turn_led_on(1);	
 	}
 	status = getReg (PWMB_PMFSA);
	if ((axis == 2)	&& !(status & PWMB_PMFSA_FPIN3_MASK)) 
	{
	 		
	 	PWM_B_setDuty (0, 0);
 		PWM_B_setDuty (1, 0);
 		PWM_B_load();
 		_pad_enabled[2]= true;
		PWM_B_outputPadEnable(CHANNELS_01);	
		setRegBits(GPIO_D_DR,0x4); 
		turn_led_on(1);	
	}
	else if ((axis == 3)	&& !(status & PWMB_PMFSA_FPIN3_MASK)) 
	{
	 	PWM_B_setDuty (2, 0);
 		PWM_B_setDuty (3, 0);
 		_pad_enabled[3]= true;
		PWM_B_outputPadEnable(CHANNELS_23);
		setRegBits(GPIO_D_DR,0x8); 
		turn_led_on(1);	
	}
	
	FaultInterruptEnable(axis);
}

/***************************************************************************/
/**
 * This method generates the PWM signal for a 4 DC board
 * @param i is the axis number, can be 0/3
 * @param pwm_value is the duty cicle of the pwm (in clock ticks)
 ***************************************************************************/
void PWM_generate (byte i, Int16 pwm_value)
{
	if (pwm_value >= 0)
		{	
			switch (i)
			{
				case 0:
					PWM_A_setDuty (0, (unsigned char)(pwm_value & 0x7fff));
					PWM_A_setDuty (1, 0);
				break;
				case 1:
					PWM_A_setDuty (2, (unsigned char)(pwm_value & 0x7fff));
					PWM_A_setDuty (3, 0);
				break;
				case 2:
					PWM_B_setDuty (0, (unsigned char)(pwm_value & 0x7fff));
					PWM_B_setDuty (1, 0);
				break;
				case 3:
					PWM_B_setDuty (2, (unsigned char)(pwm_value & 0x7fff));
					PWM_B_setDuty (3, 0);
				break;
			}
		}
	else
		{
			switch (i)
			{
				case 0:
					PWM_A_setDuty (0, 0);
					PWM_A_setDuty (1, (unsigned char)(-pwm_value & 0x7fff));
				break;
				case 1:
					PWM_A_setDuty (2, 0);
					PWM_A_setDuty (3, (unsigned char)(-pwm_value & 0x7fff));
				break;
				case 2:
					PWM_B_setDuty (0, 0);
					PWM_B_setDuty (1, (unsigned char)(-pwm_value & 0x7fff));
				break;
				case 3:
					PWM_B_setDuty (2, 0);
					PWM_B_setDuty (3, (unsigned char)(-pwm_value & 0x7fff));
				break;
			}
		}	
	
	if       (i== 0 || i== 1)  PWM_A_load();
	else if	 (i== 2 || i== 3)  PWM_B_load();
}

