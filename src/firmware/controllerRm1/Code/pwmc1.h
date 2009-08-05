/*
 * pwmc1.h
 *	the pwm generation interface (channel A).
 */

#ifndef __pwmc1h__
#define __pwmc1h__

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


void PWMC1_init (void);
byte PWMC1_setPeriod (word period);
byte PWMC1_setDuty (byte channel, int duty);
byte PWMC1_setOutput (bool OutCTL, TChannels Outputs);
byte PWMC1_setPrescaler (byte presc);
#define PWMC1_load() setRegBit(PWMB_PMCTL, LDOK)
byte PWMC1_setDutyPercent (byte channel, byte duty);
//#define PWMC1_outputPadEnable() setRegBit(PWMB_PMOUT, PAD_EN)
byte PWMC1_outputPadEnable (void);
#define PWMC1_outputPadDisable() clrRegBit(PWMB_PMOUT, PAD_EN)


#endif /* ifndef __pwmc1h__ */
