/*
 * pwmc1.c
 * 	implementation of the PWM generation interface.
 *
 */

/**
 * \file pwmc1.c
 *	the implementation of the PWM channel A interface.
 */
 
#include "pwmc1.h"
#include "controller.h"
#include "asc.h"


/**
 * initializes the PWM module w/ 30KHz complementary mode and 8 clock tick dead time.
 */
void PWMC1_init(void)
{
	/* PWMB_PMCTL: LDFQ=0,HALF=0,IPOL2=0,IPOL1=0,IPOL0=0,PRSC=0,PWMRIE=0,PWMF=0,ISENS=0,LDOK=0,PWMEN=0 */
	setReg (PWMB_PMCTL, 0);
	            
	/* PWMB_PMFCTL: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,FIE3=0,FMODE3=1,FIE2=0,FMODE2=1,FIE1=0,FMODE1=1,FIE0=0,FMODE0=1 */
	/* PWMB_PMDISMAP1: DISMAP=0 */
	/* PWMB_PMDISMAP2: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,DISMAP=0 */
#ifndef EMERGENCY_DISABLED	
	setReg (PWMB_PMFCTL, 0x00); 
#if VERSION == 0x0111 || VERSION == 0x0114
	setReg (PWMB_PMDISMAP1, 0x3333);
#else
	setReg (PWMB_PMDISMAP1, 0x2222);
#endif	          
	setReg (PWMB_PMDISMAP2, 0x00);          
#else
	setReg (PWMB_PMFCTL, 0x55);            
	setReg (PWMB_PMDISMAP1, 0);          
	setReg (PWMB_PMDISMAP2, 0);          
#endif

	/* PWMB_PMOUT: PAD_EN=0,??=0,OUTCTL=0,??=0,??=0,OUT=0 */
	setReg (PWMB_PMOUT, 0);

	/* PWMB_PMCCR: ENHA=0,??=0,MSK=0,??=0,??=0,VLMODE=0,??=0,SWP45=0,SWP23=0,SWP01=0 */
	setReg (PWMB_PMCCR, 0);
	            
	/* PWMB_PMCFG: ??=0,??=0,??=0,EDG=1,??=0,TOPNEG45=0,TOPNEG23=0,TOPNEG01=0,??=0,BOTNEG45=0,BOTNEG23=0,BOTNEG01=0,INDEP45=0,INDEP23=0,INDEP01=0,WP=0 */
	setReg (PWMB_PMCFG, 0x1000);           

	/* PWMB_PMDEADTM: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PWMDT=40 */
	setReg (PWMB_PMDEADTM, 0x28);          

	/* PWMB_PWMVAL0: PWMVAL=0 */
	setReg (PWMB_PWMVAL0, 0);            

	/* PWMB_PWMVAL1: PWMVAL=1333 */
	setReg (PWMB_PWMVAL1, 0x0535);         

	/* PWMB_PWMVAL2: PWMVAL=0 */
	setReg (PWMB_PWMVAL2, 0);            

	/* PWMB_PWMVAL3: PWMVAL=1333 */
	setReg (PWMB_PWMVAL3, 0x0535);         

	/* PWMB_PWMVAL4: PWMVAL=0 */
	setReg (PWMB_PWMVAL4, 0);            

	/* PWMB_PWMVAL5: PWMVAL=1333 */
	setReg (PWMB_PWMVAL5, 0x0535);         

	/* PWMB_PWMCM: ??=0,PWMCM=1333 i.e. 30KHz*/
	setReg (PWMB_PWMCM, 0x0535);           

	/* PWMB_PMCTL: LDOK=1,PWMEN=1 */
	setRegBits (PWMB_PMCTL, 3);         

	/* write protect on */
	setReg (PWMB_PMCFG, 0x1001);           
}


/**
 * Enables the PWM pad and clears fault pins.
 * @return ERR_OK always.
 */
byte PWMC1_outputPadEnable (void)
{
	setRegBit (PWMB_PMOUT, PAD_EN);
#ifndef EMERGENCY_DISABLED
	setReg (PWMB_PMFSA, 0x55);
#endif	
	return ERR_OK;
}


/**
 * sets the period of the PWM signal.
 * @param period is the period of the PWM in the 15 bit range. This is the
 * modulo of the counter.
 * @return ERR_OK or ERR_RANGE.
 */
byte PWMC1_setPeriod (word period)
{
	if (period < 32768)
		setReg (PWMB_PWMCM, period);
	else
		return ERR_RANGE;
		
	return ERR_OK;
}

/**
 * sets the PWM duty cycle.
 * @param channel is the channel number between 0 and 5.
 * @param duty is the duty cycle. 0 means off, greater than 0x7fff
 * will cause the pwm to be on the whole period.
 * @return ERR_OK if successful.
 */
byte PWMC1_setDuty (byte channel, int duty)
{
	switch (channel) 
	{
	case 0 :
		setReg (PWMB_PWMVAL0, duty);
		break;
	case 1 :
		setReg (PWMB_PWMVAL1, duty);
		break;
	case 2 :
		setReg (PWMB_PWMVAL2, duty);
		break;
	case 3 :
		setReg (PWMB_PWMVAL3, duty);
		break;
	case 4 :
		setReg (PWMB_PWMVAL4, duty);
		break;
	case 5 :
		setReg (PWMB_PWMVAL5, duty);
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
byte PWMC1_setDutyPercent(byte channel,byte duty)
{
	register word dutyreg;

	if (duty>100)
		return ERR_RANGE;
		
	dutyreg = (word)((dword)getReg (PWMB_PWMCM) * duty / 100);
	switch (channel) 
	{
	case 0 :
		setReg(PWMB_PWMVAL0,dutyreg);
		break;
	case 1 :
		setReg(PWMB_PWMVAL1,dutyreg);
		break;
	case 2 :
		setReg(PWMB_PWMVAL2,dutyreg);
		break;
	case 3 :
		setReg(PWMB_PWMVAL3,dutyreg);
		break;
	case 4 :
		setReg(PWMB_PWMVAL4,dutyreg);
		break;
	case 5 :
		setReg(PWMB_PWMVAL5,dutyreg);
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
byte PWMC1_setOutput(bool OutCTL, TChannels Outputs)
{
	if (OutCTL)
		setReg (PWMB_PMOUT, ((*(byte*)&Outputs) & 63) | 48896);
	else
		setReg (PWMB_PMOUT, 32768);
	
	return ERR_OK;
}

/**
 * sets the clock prescaler.
 * @param presc is the prescaler value in range 0-3 that mean divisors 1 to 8.
 * @return ERR_OK if successful.
 */
byte PWMC1_setPrescaler(byte presc)
{
	if (presc < 4) 
	{
		clrRegBits (PWMB_PMCTL, 0x00c0);
		setRegBits (PWMB_PMCTL, (presc << 6));
		///setRegBitGroup(PWMB_PMCTL, PRSC, presc);
		return ERR_OK;
	}
	else
		return ERR_RANGE;
}