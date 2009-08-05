#include "dsp56f807.h"
#include "pwm_interface.h"
#include "brushless_comm.h"

#include "pwm_a.h"
#include "pwm_b.h"
#include "faults_interface.h"
#include "leds_interface.h"
#include "can1.h"

extern sDutyControlBL DutyCycle[2];
extern sDutyControlBL DutyCycleReq[2];
bool _pad_enabled[2] = {0,0};

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
#ifdef DEBUG_CAN_MSG
	can_printf("PWM DIS CH%d", axis);
#endif
	if (axis == 0)
	{
		led0_off
		_pad_enabled[0]= false;
		PWM_A_outputPadDisable(ALL_CHANNELS);
		DutyCycle[0].Duty = MIN_DUTY; 
	//	PWM_generate_BLL(0, DutyCycle[0].Duty);	
	}
	else
	{
		led2_off			
		_pad_enabled[1]= false;
		PWM_B_outputPadDisable(ALL_CHANNELS);
		DutyCycle[1].Duty = MIN_DUTY; 
	//	PWM_generate_BLL(1, DutyCycle[1].Duty)	;
	
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
	if ((axis == 0) && !(status & PWMA_PMFSA_FPIN3_MASK)) 	 
	{	
		if (getRegBit(PWMA_PMOUT,PAD_EN)) 
		{
			led0_on
			_pad_enabled[0]= true;
			return;
		}
		led0_on
		_pad_enabled[0]= true;
		PWM_A_outputPadEnable(ALL_CHANNELS); 	
	}
	status = getReg (PWMB_PMFSA);
	if ((axis == 1)  && !(status & PWMB_PMFSA_FPIN3_MASK))
	{
		if (getRegBit(PWMB_PMOUT,PAD_EN))
		{
			led2_on
			_pad_enabled[1]= true;
			return;
		}
		led2_on
		_pad_enabled[1]= true;
		PWM_B_outputPadEnable(ALL_CHANNELS);
	}    
		FaultInterruptEnable(axis);		
}

/***************************************************************************/
/**
 * This method generates the PWM signal for a 2 BRUSHLESS board
 * This function can be called only by the TD0_interrupt or 
 * by the PWM_OUTPUT_ENABLE and DISABLE
 * @param i is the axis number, can be 0/1
 * @param pwm_value is the duty cicle of the pwm (in clock ticks)
 ***************************************************************************/
//#pragma interrupt called
void PWM_generate_BLL(byte i, Int16 pwm_value)
{
	if (pwm_value < MIN_DUTY) pwm_value = MIN_DUTY;
	if (pwm_value > (MAX_DUTY)) pwm_value = (MAX_DUTY);
	
	if (i==0)
	{
		PWM_A_setDuty (0, (unsigned char)(pwm_value & 0x7fff));
		PWM_A_setDuty (2, (unsigned char)(pwm_value & 0x7fff));
		PWM_A_setDuty (4, (unsigned char)(pwm_value & 0x7fff));
	}
	else if (i==1)
	{
		PWM_B_setDuty (0, (unsigned char)(pwm_value & 0x7fff));
		PWM_B_setDuty (2, (unsigned char)(pwm_value & 0x7fff));
		PWM_B_setDuty (4, (unsigned char)(pwm_value & 0x7fff));
	}
	

	if  (i==0)  PWM_A_load();
	else		PWM_B_load();
}

/***************************************************************************/
/**
 * This method generates the PWM signal for a 2 BRUSHLESS board
 * This must be used to give the dutycycle to the motor.
 * The function does not put immediately the value to the PWM channel, but 
 * it change the value of the DutyCycleReq. The DutyCycle will be increased or 
 * decreased until the reaching of the DutyCycleReq value by 1 unit each 1 ms. 
 * @param i is the axis number, can be 0/1
 * @param pwm_value is the duty cicle of the pwm (in clock ticks)
 ***************************************************************************/

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
