/**
 * QD1.c
 * implementation of the quadrature decoder interface.
 *
 */

#include "qd1.h"

/**
 * sets the init registers of the quadrature decoder.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD1_setResetPosition (dword Position)
{
	setReg (QD1_UIR, (word)(Position>>16));
	setReg (QD1_LIR, (word)(Position));
	return ERR_OK;
}

/**
 * sets the encoder position by using the init register method.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD1_setPosition (dword Position)
{
	setReg(QD1_UIR, (word)(Position>>16));
	setReg(QD1_LIR, (word)(Position));
	setRegBit(QD1_DECCR, SWIP);
	return ERR_OK;
}

/**
 * gets the encoder position value as a 32 bit integer.
 * @param Position is the pointer to the variable holding the value.
 * @return ERR_OK always.
 */
byte QD1_getPosition (dword *Position)
{
	*Position = getReg(QD1_LPOS) | ((dword)getReg(QD1_UPOS)<<16);
	return ERR_OK;
}

/**
 * initializes the quadrature decoder circuitry.
 */
void QD1_init (void)
{
	/* QD1_DECCR: HIRQ=0,HIE=0,HIP=0,HNE=0,SWIP=0,REV=0,PH1=0,XIRQ=0,XIE=0,XIP=0,XNE=0,DIRQ=0,DIE=0,WDE=0,MODE=0 */
	
	/* not needed since it's cleared later on.
	clrRegBit (QD1_DECCR, MODE0);
	clrRegBit (QD1_DECCR, MODE1);
	*/
	// The FIR has a frequency of 156 KHz (255) instead of 4MHz (10)
	
	setReg (QD1_FIR, 10);
	
	setRegBits (QD1_DECCR, 0);
}

/**
 * intiializes the position by setting the SWIP bit.
 * @return ERR_OK always.
 */
byte QD1_ResetPosition (void)
{
	setRegBit (QD1_DECCR, SWIP);
	return ERR_OK;
}

/**
 * gets the signals from the quadrature decoder channels.
 * @param Filtered, if TRUE gets the filtered signals, otherwise the raw ones.
 * @param Signals is the pointer to an array of raw/filtered signals containing 
 * home, index, B, and A.
 */
byte QD1_getSignals (bool Filtered, word *Signals)
{
	if (Filtered)
		*Signals = getReg(QD1_IMR) >> 4;
	else
		*Signals = getReg(QD1_IMR) & 15;
	return ERR_OK;
}
