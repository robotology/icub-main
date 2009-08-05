#include "qd3.h"

dword qd3_reset_position=0;
volatile dword qd3_position=0;
volatile byte  qd3_status     = 0;
volatile byte  qd3_status_old = 0;

#define COUNT_UP \
{ \
	qd3_position++; \
}

#define COUNT_DOWN \
{ \
	qd3_position--; \
}

/**
 * sets the init registers of the quadrature decoder.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD3_setResetPosition (dword Position)
{
	qd3_reset_position=Position;
	return ERR_OK;
}

/**
 * sets the encoder position by using the init register method.
 * @param Position is the 32bit position value.
 * @return ERR_OK always.
 */
byte QD3_setPosition (dword Position)
{
	qd3_position=qd3_reset_position=Position;
	return ERR_OK;
}

/**
 * gets the encoder position value as a 32 bit integer.
 * @param Position is the pointer to the variable holding the value.
 * @return ERR_OK always.
 */
byte QD3_getPosition (dword *Position)
{
	*Position=qd3_position;
	return ERR_OK;
}

/**
 * initializes the timers D0(PHA), D1(PHB), D3(INDEX) for quadrature decoding.
 */
void QD3_init (void)
{
  // - - - Connects the timer C0 to the PHA pin - - -
  // TMRC0_CTRL: CM=1,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRD0_CTRL, 0b0011000000000000);              
  // TMRC0_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=3,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRD0_SCR,  0b0000010011000000);
  setReg (TMRD0_CNTR,0);                // Reset counter register 
  setReg (TMRD0_LOAD,0);                // Reset load register 
  setReg (TMRD0_CAP,0);                 // Reset capture register 	

  // - - - Connects the timer C1 to the PHB pin - - -
  // TMRC1_CTRL: CM=1,PCS=8,SCS=1,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRD1_CTRL, 0b0011000010000000);              
  // TMRC1_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=3,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRD1_SCR,  0b0000010011000000);
  setReg (TMRD1_CNTR,0);                // Reset counter register 
  setReg (TMRD1_LOAD,0);                // Reset load register 
  setReg (TMRD1_CAP,0);                 // Reset capture register 	

  // - - - Connects the timer D2 to the INDEX pin - - -
  // TMRD2_CTRL: CM=1,PCS=8,SCS=2,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRD3_CTRL, 0b0011000100000000);              
  // TMRD2_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=1,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP  
  setReg (TMRD3_SCR,  0b0000010001000000);
  setReg (TMRD3_CNTR,0);                // Reset counter register 
  setReg (TMRD3_LOAD,0);                // Reset load register 
  setReg( TMRD3_CAP,0);                 // Reset capture register 
}

/**
 * intiializes the position by setting the SWIP bit.
 * @return ERR_OK always.
 */
byte QD3_ResetPosition (void)
{
	qd3_position=qd3_reset_position;
	return ERR_OK;
}

/**
 * gets the signals from the quadrature decoder channels.
 * @param Filtered, if TRUE gets the filtered signals, otherwise the raw ones.
 * @param Signals is the pointer to an array of raw/filtered signals containing 
 * home, index, B, and A.
 */
byte QD3_getSignals (bool Filtered, word *Signals)
{
	return ERR_OK;
}

#pragma interrupt called
void QD3_decode (void)
{
	switch (qd3_status_old)
	{
		case 0b10:
			if      (qd3_status == 0b11) COUNT_DOWN
			else if (qd3_status == 0b00) COUNT_UP
		break;
		case 0b11:
			if      (qd3_status == 0b01) COUNT_DOWN
			else if (qd3_status == 0b10) COUNT_UP
		break;
		case 0b01:
			if      (qd3_status == 0b00) COUNT_DOWN
			else if (qd3_status == 0b11) COUNT_UP
		break;
		case 0b00:
			if      (qd3_status == 0b10) COUNT_DOWN
			else if (qd3_status == 0b01) COUNT_UP
		break;
	}
}

#define TCF_BIT 0x8000
#define TOF_BIT 0x2000
#define IEF_BIT 0x800
#define SCR_INPUT 0x100

//*********************************************************
#pragma interrupt saveall
void TD0_Interrupt(void)
{
 Int16 timer_status = getReg (TMRD0_SCR);             
 
 //----------------------TIMER A0---------------------	
  //timer capture check
 //IEF bit (11)
 if (timer_status & IEF_BIT)
 	{
  	 // clear interrupt flag
 	 clrRegBits(TMRD0_SCR, IEF_BIT);
 	
 	 //qd2_status_old = qd2_status;
     // received interrupt from the motor hall effect sensor
 	 qd3_status = getRegBits(TMRD0_SCR,SCR_INPUT) | getRegBits(TMRD1_SCR,SCR_INPUT)<<1;
 	 QD3_decode ();
 	 qd3_status_old = qd3_status;
	 }
}

//*********************************************************
#pragma interrupt saveall
void TD1_Interrupt(void)
{
 Int16 timer_status = getReg (TMRD1_SCR);             
 
 //----------------------TIMER A0---------------------	
  //timer capture check
 //IEF bit (11)
 if (timer_status & IEF_BIT)
 	{
  	 // clear interrupt flag
 	 clrRegBits(TMRD1_SCR, IEF_BIT);
 	
 	 //qd2_status_old = qd2_status;
     // received interrupt from the motor hall effect sensor
 	 qd3_status = getRegBits(TMRD0_SCR,SCR_INPUT) | getRegBits(TMRD1_SCR,SCR_INPUT)<<1;
 	 QD3_decode ();
 	 qd3_status_old = qd3_status;
	 }
}

