/*
 * pwmc0.c
 * 	implementation of the PWM generation interface.
 *
 */

/**
 * \file pwmc0.c
 *	the implementation of the PWM channel A interface.
 */
 
#include "pwmc0.h"
#include "controller.h"

/**
 * initializes the PWM module w/ 30KHz complementary mode and 8 clock tick dead time.
 */
void PWMC0_init(void)
{
	/* PWMA_PMCTL: LDFQ=0,HALF=0,IPOL2=0,IPOL1=0,IPOL0=0,PRSC=0,PWMRIE=0,PWMF=0,ISENS=0,LDOK=0,PWMEN=0 */
	setReg (PWMA_PMCTL, 0);
	            
	/* PWMA_PMFCTL: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,FIE3=0,FMODE3=1,FIE2=0,FMODE2=1,FIE1=0,FMODE1=1,FIE0=0,FMODE0=1 */
	/* PWMA_PMDISMAP1: DISMAP=0 */
	/* PWMA_PMDISMAP2: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,DISMAP=0 */
#ifndef EMERGENCY_DISABLED
	setReg (PWMA_PMFCTL, 0x00); /* manual fault reset */
#if VERSION == 0x0111 || VERSION == 0x0114
	setReg (PWMA_PMDISMAP1, 0x3333);
#else
	setReg (PWMA_PMDISMAP1, 0x2222);
#endif	
	setReg (PWMA_PMDISMAP2, 0x00);          
#else	
	setReg (PWMA_PMFCTL, 0x55);
	setReg (PWMA_PMDISMAP1, 0);          
	setReg (PWMA_PMDISMAP2, 0);          
#endif

	/* PWMA_PMOUT: PAD_EN=0,??=0,OUTCTL=0,??=0,??=0,OUT=0 */
	setReg (PWMA_PMOUT, 0);

	/* PWMA_PMCCR: ENHA=0,??=0,MSK=0,??=0,??=0,VLMODE=0,??=0,SWP45=0,SWP23=0,SWP01=0 */
	setReg (PWMA_PMCCR, 0);
	            
	/* PWMA_PMCFG: ??=0,??=0,??=0,EDG=1,??=0,TOPNEG45=0,TOPNEG23=0,TOPNEG01=0,??=0,BOTNEG45=0,BOTNEG23=0,BOTNEG01=0,INDEP45=0,INDEP23=0,INDEP01=0,WP=0 */
	setReg (PWMA_PMCFG, 0x1000);           

	/* PWMA_PMDEADTM: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PWMDT=40 */
	setReg (PWMA_PMDEADTM, 0x28);          

	/* PWMA_PWMVAL0: PWMVAL=0 */
	setReg (PWMA_PWMVAL0, 0x0);            

	/* PWMA_PWMVAL1: PWMVAL=1333 */
	setReg (PWMA_PWMVAL1, 0x535);         

	/* PWMA_PWMVAL2: PWMVAL=0 */
	setReg (PWMA_PWMVAL2, 0);            

	/* PWMA_PWMVAL3: PWMVAL=1333 */
	setReg (PWMA_PWMVAL3, 0x535);         

	/* PWMA_PWMVAL4: PWMVAL=0 */
	setReg (PWMA_PWMVAL4, 0);            

	/* PWMA_PWMVAL5: PWMVAL=1333 */
	setReg (PWMA_PWMVAL5, 0x535);         

	/* PWMA_PWMCM: ??=0,PWMCM=1333 i.e. 30KHz*/
	setReg (PWMA_PWMCM, 0x535);           

	/* PWMA_PMCTL: LDOK=1,PWMEN=1 */
	setRegBits (PWMA_PMCTL, 3);     
	
	/* write protect on */
	setReg (PWMA_PMCFG, 0x1001);            
}


/**
 * Enables the PWM pad and clears fault pins.
 * @return ERR_OK always.
 */
byte PWMC0_outputPadEnable (void)
{
	setRegBit (PWMA_PMOUT, PAD_EN);
#ifndef EMERGENCY_DISABLED
	setReg (PWMA_PMFSA, 0x55);
#endif
	
	return ERR_OK;
}


/**
 * sets the period of the PWM signal.
 * @param period is the period of the PWM in the 15 bit range. This is the
 * modulo of the counter.
 * @return ERR_OK or ERR_RANGE.
 */
byte PWMC0_setPeriod (word period)
{
	if (period < 32768)
		setReg (PWMA_PWMCM, period);
	else
		return ERR_RANGE;
		
	return ERR_OK;
}

/**
 * sets the PWM duty cycle.
 * @param channel is the channel number between 0 and 5.
 * @param val determines the duty cycle which is given by
 * the expression duty = val/PWMCM * 100. 0 means off, 
 * greater than 0x7fff will cause the pwm to be on the 
 * whole period.
 * @return ERR_OK if successful.
 */
byte PWMC0_setDuty (byte channel, int val)
{
	switch (channel) 
	{
	case 0 :
		setReg (PWMA_PWMVAL0, val);
		break;
	case 1 :
		setReg (PWMA_PWMVAL1, val);
		break;
	case 2 :
		setReg (PWMA_PWMVAL2, val);
		break;
	case 3 :
		setReg (PWMA_PWMVAL3, val);
		break;
	case 4 :
		setReg (PWMA_PWMVAL4, val);
		break;
	case 5 :
		setReg (PWMA_PWMVAL5, val);
		break;
	default: 
		return ERR_RANGE;
	}
	return ERR_OK;
}

/**
 * sets the duty cycle as a percentage.
 * @param channel is the PWM channel to control (0-5).
 * @param duty is the duty cycle (0-100).
 * @return ERR_OK if successful.
 */
byte PWMC0_setDutyPercent(byte channel,byte duty)
{
	register word dutyreg;

	if (duty>100)
		return ERR_RANGE;
		
	dutyreg = (word)((dword)getReg (PWMA_PWMCM) * duty / 100);
	switch (channel) 
	{
	case 0 :
		setReg(PWMA_PWMVAL0,duty);
		break;
	case 1 :
		setReg(PWMA_PWMVAL1,duty);
		break;
	case 2 :
		setReg(PWMA_PWMVAL2,duty);
		break;
	case 3 :
		setReg(PWMA_PWMVAL3,duty);
		break;
	case 4 :
		setReg(PWMA_PWMVAL4,duty);
		break;
	case 5 :
		setReg(PWMA_PWMVAL5,duty);
		break;
	default: 
		return ERR_RANGE;
	}
	
	return ERR_OK;
}

/**
 * sets output.
 * @param OutCTL, if TRUE sets the output channels pad (on/off).
 * @param Outputs is the channel structure (see above).
 * @return ERR_OK always.
 */
byte PWMC0_setOutput(bool OutCTL, TChannels Outputs)
{
	if (OutCTL)
		setReg (PWMA_PMOUT, ((*(byte*)&Outputs) & 63) | 48896);
	else
		setReg (PWMA_PMOUT, 32768);
	
	return ERR_OK;
}

/**
 * sets the clock prescaler.
 * @param presc is the prescaler value in range 0-3 that mean divisors 1 to 8.
 * @return ERR_OK if successful.
 */
byte PWMC0_setPrescaler(byte presc)
{
	if (presc < 4) 
	{
		clrRegBits (PWMA_PMCTL, 0x00c0);
		setRegBits (PWMA_PMCTL, (presc << 6));
		///setRegBitGroup(PWMA_PMCTL, PRSC, presc);
		return ERR_OK;
	}
	else
		return ERR_RANGE;
}