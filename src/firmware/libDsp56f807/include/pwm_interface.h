
#ifndef __pwm_interfaceh__
#define __pwm_interfaceh__

/***************************************************************************/
/**
 * This method inits the PWM interface for the motor board
 ***************************************************************************/
void init_pwm(void);

/***************************************************************************/
/**
 * This method disables the PWM generation on the specified channel
 * @param axis is the axis number
 ***************************************************************************/
void PWM_outputPadDisable(byte axis);

/***************************************************************************/
/**
 * This method enables the PWM generation on the specified channel
 * @param axis is the axis number
 ***************************************************************************/
void PWM_outputPadEnable(byte axis);

/***************************************************************************/
/**
 * This method generates the PWM signal for the motor board
 * @param i is the axis number
 * @param pwm_value is the duty cicle of the pwm (in clock ticks)
 ***************************************************************************/
void PWM_generate (byte i, Int16 pwm_value);

#endif 
