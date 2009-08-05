
#ifndef __pwm_b_h__
#define __pwm_b_h__

/**
 * \file pwm_b.h
 * contains the interface definition for the PWM channel B.
 */
#include "dsp56f807.h"

/**************************************************************************************/
/**
 * initializes the PWM module B
 *
 **************************************************************************************/
void PWM_B_init (void);

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
byte PWM_B_setDuty (byte channel, int duty);

/**************************************************************************************/
/**
 * Used after PWM_setDuty to load the new specified duty cycle
 */
/**************************************************************************************/
#define PWM_B_load() setRegBit(PWMB_PMCTL, LDOK)
#define PWM_B_PWMEN() setRegBit(PWMB_PMCTL,PWMEN)
/**************************************************************************************/
/**
 * sets the duty cycle as a percentage.
 * @param channel is the PWM channel to control (0-5).
 * @param duty is the duty cycle (0-100).
 * @return ERR_OK if successful.
 */
/**************************************************************************************/
byte PWM_B_setDutyPercent (byte channel, byte duty);

/**************************************************************************************/
/**
 * Enables the PWM pad and clears fault pins.
 * @param mask specifies the output channels to be enabled.
 */
/**************************************************************************************/
void PWM_B_outputPadEnable (word mask);

/**************************************************************************************/
/**
 * Disables the PWM pad.
 * @param mask specifies the output channels to be disabled.
 */
/**************************************************************************************/
void PWM_B_outputPadDisable(word mask);


#endif /* ifndef __pwm_b_h__ */
