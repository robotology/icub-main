/**
 * qd0.c
 * implementation of the quadrature decoder interface.
 *
 */

#include "qd0.h"

/**
 * sets the init registers of the quadrature decoder.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD0_setPositionInit (dword Position)
{
	setReg (QD0_UIR, (word)(Position>>16));
	setReg (QD0_LIR, (word)(Position));
	return ERR_OK;
}

/**
 * sets the encoder position by using the init register method.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD0_setPosition (dword Position)
{
	setReg(QD0_UIR, (word)(Position>>16));
	setReg(QD0_LIR, (word)(Position));
	setRegBit(QD0_DECCR, SWIP);
	return ERR_OK;
}

/**
 * gets the encoder position value as a 32 bit integer.
 * @param Position is the pointer to the variable holding the value.
 * @return ERR_OK always.
 */
byte QD0_getPosition (dword *Position)
{
	*Position = getReg(QD0_LPOS) | ((dword)getReg(QD0_UPOS)<<16);
	return ERR_OK;
}

/**
 * initializes the quadrature decoder circuitry.
 */
void QD0_init (void)
{
	/* QD0_DECCR: HIRQ=0,HIE=0,HIP=0,HNE=0,SWIP=0,REV=0,PH1=0,XIRQ=0,XIE=0,XIP=0,XNE=0,DIRQ=0,DIE=0,WDE=0,MODE=0 */
	
	/* not needed since it's cleared later on.
	clrRegBit (QD0_DECCR, MODE0);
	clrRegBit (QD0_DECCR, MODE1);
	*/
	setReg (QD0_FIR, 10);
	
	setRegBits (QD0_DECCR, 0);
}

/**
 * intiializes the position by setting the SWIP bit.
 * @return ERR_OK always.
 */
byte QD0_initPosition (void)
{
	setRegBit (QD0_DECCR, SWIP);
	return ERR_OK;
}

/**
 * gets the signals from the quadrature decoder channels.
 * @param Filtered, whether to filter the signals (TRUE means filter is on).
 * @param Signals is the pointer to an array of signals containing 
 * home, index, B, and A.
 */
byte QD0_getSignals (bool Filtered, word *Signals)
{
	if (Filtered)
		*Signals = getReg(QD0_IMR) >> 4;
	else
		*Signals = getReg(QD0_IMR) & 15;
	return ERR_OK;
}
