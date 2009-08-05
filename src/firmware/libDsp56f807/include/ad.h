/*
 * ad.h
 * 	AD sampling interface.
 */

#ifndef __adh__
#define __adh__

/**
 * \file ad.h 
 * ad.h contains the interface definition to the AD converter.
 * Current implementation is made to acquire from a single channel only.
 */

#include "dsp56f807.h"

/**************************************************************************************/
/**
 * This method performs one measurement on channel A
 *
 * @param wait waits for result to be ready.
 * @return ERR_OK after a successful sampling, ERR_BUSY if the device is
 * already running a conversion.
 *
 **************************************************************************************/
byte AD_measureA(bool wait);

/**************************************************************************************/
/**
 * This method performs one measurement on channel B
 *
 * @param wait waits for result to be ready.
 * @return ERR_OK after a successful sampling, ERR_BUSY if the device is
 * already running a conversion.
 *
 **************************************************************************************/
byte AD_measureB(bool wait);

/**************************************************************************************/
/**
 * enables triggered sequential mode synchronous with the
 * PWM generation signal for Channel A.
 *
 **************************************************************************************/
byte AD_enableIntTriggerA(void);

/**************************************************************************************/
/**
 * enables triggered sequential mode synchronous with the
 * PWM generation signal for channel B.
 *
 **************************************************************************************/
byte AD_enableIntTriggerB(void);

/**************************************************************************************/
/**
 * stops the acquisition on Channel A, disables interrupts.
 * use init to start acquisition again.
 *
 **************************************************************************************/
byte AD_stopAcquisitionA(void);

/**************************************************************************************/
/**
 * stops the acquisition on channel B, disables interrupts.
 * use init to start acquisition again.
 *
 **************************************************************************************/
byte AD_stopAcquisitionB(void);

/**************************************************************************************/
/**
 * gets the sampled values if available.
 * @param i is the index of the channel to read 0-2
 * @param values is a pointer to a 16 bit value.
 * @return ERR_OK if values are available, ERR_NOTAVAIL otherwise.
 *
 **************************************************************************************/
byte AD_getChannel16A(byte i, word *value);

/**************************************************************************************/
/**
 * gets the sampled values if available.
 * @param i is the index of the channel to read 0-2
 * @param values is a pointer to a 16 bit value.
 * @return ERR_OK if values are available, ERR_NOTAVAIL otherwise.
 *
 **************************************************************************************/
byte AD_getChannel16B(byte i, word *value);

/**************************************************************************************/
/**
 * initializes the AD conversion module.
 *
 **************************************************************************************/
void AD_init(void);


#endif /* ifndef __adh__ */
