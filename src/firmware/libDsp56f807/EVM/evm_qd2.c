#include "qd2.h"

dword qd2_reset_position=0;
volatile dword qd2_position=0;
volatile byte  qd2_status     = 0;
volatile byte  qd2_status_old = 0;

#define COUNT_UP \
{ \
	qd2_position++; \
}

#define COUNT_DOWN \
{ \
	qd2_position--; \
}

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
	*Position=qd2_position;
	return ERR_OK;
}

/**
 * initializes the timers C0(PHA), C1(PHB), D2(INDEX) for quadrature decoding.
 */
void QD2_init (void)
{
  // - - - Connects the timer C0 to the PHA pin - - -
  // TMRC0_CTRL: CM=1,PCS=8,SCS=0,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRC0_CTRL, 0b0011000000000000);              
  // TMRC0_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=3,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRC0_SCR,  0b0000010011000000);
  setReg (TMRC0_CNTR,0);                // Reset counter register 
  setReg (TMRC0_LOAD,0);                // Reset load register 
  setReg (TMRC0_CAP,0);                 // Reset capture register 	

  // - - - Connects the timer C1 to the PHB pin - - -
  // TMRC1_CTRL: CM=1,PCS=8,SCS=1,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRC1_CTRL, 0b0011000010000000);              
  // TMRC1_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=3,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP
  setReg (TMRC1_SCR,  0b0000010011000000);
  setReg (TMRC1_CNTR,0);                // Reset counter register 
  setReg (TMRC1_LOAD,0);                // Reset load register 
  setReg (TMRC1_CAP,0);                 // Reset capture register 	

  // - - - Connects the timer D2 to the INDEX pin - - -
  // TMRD2_CTRL: CM=1,PCS=8,SCS=2,ONCE=0,LENGTH=0,DIR=0,Co_INIT=0,OM=0 
  //                    CCCPPPPSS
  setReg (TMRD2_CTRL, 0b0011000100000000);              
  // TMRD2_SCR: TCF=0,TCFIE=0,TOF=0,TOFIE=0,IEF=0,IEFIE=1,IPS=0,INPUT=0,Capture_Mode=1,MSTR=0,EEOF=0,VAL=0,FORCE=0,OPS=0,OEN=0 
  //                    CCOOEEIIPP  
  setReg (TMRD2_SCR,  0b0000010001000000);
  setReg (TMRD2_CNTR,0);                // Reset counter register 
  setReg (TMRD2_LOAD,0);                // Reset load register 
  setReg( TMRD2_CAP,0);                 // Reset capture register 		
}

/**
 * intiializes the position by setting the SWIP bit.
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

#pragma interrupt called
void QD2_decode (void)
{
	switch (qd2_status_old)
	{
		case 0b10:
			if      (qd2_status == 0b11) COUNT_DOWN
			else if (qd2_status == 0b00) COUNT_UP
		break;
		case 0b11:
			if      (qd2_status == 0b01) COUNT_DOWN
			else if (qd2_status == 0b10) COUNT_UP
		break;
		case 0b01:
			if      (qd2_status == 0b00) COUNT_DOWN
			else if (qd2_status == 0b11) COUNT_UP
		break;
		case 0b00:
			if      (qd2_status == 0b10) COUNT_DOWN
			else if (qd2_status == 0b01) COUNT_UP
		break;
	}
}

#define TCF_BIT 0x8000
#define TOF_BIT 0x2000
#define IEF_BIT 0x800
#define SCR_INPUT 0x100

//*********************************************************
#pragma interrupt saveall
void TC0_Interrupt(void)
{
 Int16 timer_status = getReg (TMRC0_SCR);             
 
 //----------------------TIMER A0---------------------	
  //timer capture check
 //IEF bit (11)
 if (timer_status & IEF_BIT)
 	{
  	 // clear interrupt flag
 	 clrRegBits(TMRC0_SCR, IEF_BIT);
 	
 	 //qd2_status_old = qd2_status;
     // received interrupt from the motor hall effect sensor
 	 qd2_status = getRegBits(TMRC0_SCR,SCR_INPUT) | getRegBits(TMRC1_SCR,SCR_INPUT)<<1;
 	 QD2_decode ();
 	 qd2_status_old = qd2_status;
	 }
}

//*********************************************************
#pragma interrupt saveall
void TC1_Interrupt(void)
{
 Int16 timer_status = getReg (TMRC1_SCR);             
 
 //----------------------TIMER A0---------------------	
  //timer capture check
 //IEF bit (11)
 if (timer_status & IEF_BIT)
 	{
  	 // clear interrupt flag
 	 clrRegBits(TMRC1_SCR, IEF_BIT);
 	
 	 //qd2_status_old = qd2_status;
     // received interrupt from the motor hall effect sensor
 	 qd2_status = getRegBits(TMRC0_SCR,SCR_INPUT) | getRegBits(TMRC1_SCR,SCR_INPUT)<<1;
 	 QD2_decode ();
 	 qd2_status_old = qd2_status;
	 }
}
