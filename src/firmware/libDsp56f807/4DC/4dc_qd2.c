#include "qd2.h"

dword qd2_reset_position=0;
//volatile dword qd2_position=0;
dword qd2_position=0;
/** 
 * Local Prototypes
 */
void QD2_decode (void);

/**
 * sets the init registers of the quadrature decoder.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD2_setResetPosition (dword Position)
{
	qd2_reset_position=Position;
	return ERR_OK;
}

/**
 * sets the encoder position by using the init register method.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD2_setPosition (dword Position)
{
	qd2_position=qd2_reset_position=Position;
	return ERR_OK;
}

/**
 * gets the encoder position value as a 32 bit integer.
 * @param Position is the pointer to the variable holding the value.
 * @return ERR_OK always.
 */
byte QD2_getPosition (dword *Position)
{ 
	dword TimerValue=getReg(TMRC0_CNTR);
	if (TimerValue>=32000)
	{
		qd2_position=qd2_position+ (TimerValue-32000); 	
	}
	else
	{
		qd2_position =qd2_position-(32000 - TimerValue);
	}
	*Position=qd2_position;
	setReg(TMRC0_CNTR,32000);
	setRegBits(TMRC0_CTRL,0x8000);     /* Run counter */
	return ERR_OK;
}

/**
 * initializes the timers C0(PHA), C1(PHB), D2(INDEX) for quadrature decoding.
 */
void QD2_init (void)
{
  /* TMRC0_CTRL: CM=0,PCS=0,SCS=1,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 */
  setReg(TMRC0_CTRL,128);              /* Set up mode */
  /* TMRC0_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=0,IPS=0,INPUT=0,Capture_Mode=0,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 */
  setReg(TMRC0_SCR,0);
  setReg(TMRC0_CNTR,32000);                /* Reset counter register */
  setReg(TMRC0_LOAD,0);            /* Reset load register */
  setReg(TMRC0_CMP1,65535);            /* Set up compare 1 register */
  setReg(TMRC0_CMP2,0);                /* Set up compare 2 register */
  setRegBits(TMRC0_CTRL,0x8000);     /* Run counter */ 	
}

/**
 * intializes the position by setting the SWIP bit.
 * @return ERR_OK always.
 */
byte QD2_ResetPosition (void)
{
	qd2_position=qd2_reset_position;
	return ERR_OK;
}

/**
 * gets the signals from the quadrature decoder channels.
 * @param Filtered, if TRUE gets the filtered signals, otherwise the raw ones.
 * @param Signals is the pointer to an array of raw/filtered signals containing 
 * home, index, B, and A.
 */
byte QD2_getSignals (bool Filtered, word *Signals)
{
	return ERR_OK;
}

