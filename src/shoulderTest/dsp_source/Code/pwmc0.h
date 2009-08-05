/*
 * pwmc0.h
 *	the pwm generation interface (channel A).
 */

#ifndef __pwmc0h__
#define __pwmc0h__

/**
 * \file pwmc1.h
 * contains the interface definition for the PWM channel A.
 */
#include "dsp56f807.h"

#ifndef __BWUserType_TChannels
#define __BWUserType_TChannels
/**
 * 6-channel bit information structure.
 */
  typedef struct {
    byte channel0  : 1;                                           /* Channel 0 bit */
    byte channel1  : 1;                                           /* Channel 1 bit */
    byte channel2  : 1;                                           /* Channel 2 bit */
    byte channel3  : 1;                                           /* Channel 3 bit */
    byte channel4  : 1;                                           /* channel 4 bit */
    byte channel5  : 1;                                           /* channel 5 bit */
  } TChannels;                         /* Structure contains bit informations for 6 channels, one bit for each channel. */
#endif

#ifndef __BWUserType_TChannelPairs
#define __BWUserType_TChannelPairs
/**
 * 3-pair bit information structure.
 */
  typedef struct {
    byte pair0 : 1;                                               /* PWM pair 0 */
    byte pair1 : 1;                                               /* PWM pair 1 */
    byte pair2 : 1;                                               /* PWM pair 2 */
  } TChannelPairs;                     /* 3 channel pairs represented by bit in structure. */
#endif


void PWMC0_init (void);
byte PWMC0_setPeriod (word period);
byte PWMC0_setDuty (byte channel, int duty);
byte PWMC0_setOutput (bool OutCTL, TChannels Outputs);
byte PWMC0_setPrescaler (byte presc);
#define PWMC0_load() setRegBit(PWMA_PMCTL, LDOK)
byte PWMC0_setDutyPercent (byte channel, byte duty);
//#define PWMC0_outputPadEnable() setRegBit(PWMA_PMOUT, PAD_EN)
byte PWMC0_outputPadEnable (void);
#define PWMC0_outputPadDisable() clrRegBit(PWMA_PMOUT, PAD_EN)


#endif /* ifndef __pwmc1h__ */
