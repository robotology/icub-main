#ifndef __qd0h__
#define __qd0h__

#include "dsp56f807.h"

/**
 * sets the init registers of the quadrature decoder.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD0_setResetPosition (dword Position);

/**
 * sets the encoder position by using the init register method.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD0_setPosition (dword Position);

/**
 * gets the encoder position value as a 32 bit integer.
 * @param Position is the pointer to the variable holding the value.
 * @return ERR_OK always.
 */
byte QD0_getPosition (dword *Position);

/**
 * intiializes the position by setting the SWIP bit.
 * @return ERR_OK always.
 */
byte QD0_ResetPosition (void);

/**
 * gets the signals from the quadrature decoder channels.
 * @param Filtered, if TRUE gets the filtered signals, otherwise the raw ones.
 * @param Signals is the pointer to an array of raw/filtered signals containing 
 * home, index, B, and A.
 */
byte QD0_getSignals (bool Filtered, word *Signals);

/**
 * initializes the quadrature decoder circuitry.
 */
void QD0_init (void);

#endif 
