
#ifndef __pwm_a_h__
#define __pwm_a_h__

/**
 * \file pwm_a.h
 * contains the interface definition for the PWM channel A.
 */
#include "dsp56f807.h"

/**************************************************************************************/
/**
 * initializes the PWM module A
 *
 **************************************************************************************/
void PWM_A_init (void);

/**************************************************************************************/
/**
 * sets the PWM duty cycle.
 * @param channel is the channel number between 0 and 5.
 * @param val determines the duty cycle which is given by
 * the expression duty = val/PWMCM * 100. 0 means off, 
 * greater than 0x7fff will cause the pwm to be on the 
 * whole period.
 * @return ERR_OK if successful.
 */
/**************************************************************************************/
byte PWM_A_setDuty (byte channel, int duty);

/**************************************************************************************/
/**
 * Used after PWM_setDuty to load the new specified duty cycle
 */
/**************************************************************************************/
#define PWM_A_load() setRegBit(PWMA_PMCTL, LDOK)
#define PWM_A_PWMEN() setRegBit(PWMA_PMCTL, PWMEN)
/**************************************************************************************/
/**
 * sets the duty cycle as a percentage.
 * @param channel is the PWM channel to control (0-5).
 * @param duty is the duty cycle (0-100).
 * @return ERR_OK if successful.
 */
/**************************************************************************************/
byte PWM_A_setDutyPercent (byte channel, byte duty);

/**************************************************************************************/
/**
 * Enables the PWM pad and clears fault pins.
 * @param mask specifies the output channels to be enabled.
 */
/**************************************************************************************/
void PWM_A_outputPadEnable (word mask);

/**************************************************************************************/
/**
 * Disables the PWM pad.
 * @param mask specifies the output channels to be disabled.
 */
/**************************************************************************************/
void PWM_A_outputPadDisable(word mask); 


#endif /* ifndef __pwm_a_h__ */
