/**
 * \file pwm_a.c
 *	the implementation of the PWM channel A interface.
 */
 
#include "pwm_a.h"
#include "faults_interface.h"

/**************************************************************************************/
/**
 * initializes the PWM module w/ 30KHz complementary mode and 8 clock tick dead time.
 *
 **************************************************************************************/
void PWM_A_init(void)
{
	/* PWMA_PMCTL: LDFQ=0,HALF=0,IPOL2=0,IPOL1=0,IPOL0=0,PRSC=0,PWMRIE=0,PWMF=0,ISENS=0,LDOK=0,PWMEN=0 */
	setReg (PWMA_PMCTL, 0);
	            
	/* PWMA_PMOUT: PAD_EN=1,??=0,OUTCTL=111111,??=0,??=0,OUT=0 */
	setReg (PWMA_PMOUT, 0b1011111100000000);

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
	//setRegBits (PWMA_PMCFG, PWMA_PMCFG_WP_MASK);            
}

/**************************************************************************************/
/**
 * Enables the PWM pad and clears fault pins.
 * @return ERR_OK always.
 */
/**************************************************************************************/
void PWM_A_outputPadEnable (word mask)
{
    mask &= 0b0011111100000000;
	clrRegBits(PWMA_PMOUT, mask);
	reset_faults_PWMA();
}

/**************************************************************************************/
void PWM_A_outputPadDisable (word mask)
{
     mask &= 0b0011111100000000;
	 setRegBits(PWMA_PMOUT, mask);
}

/**************************************************************************************/
/**
 * sets the PWM duty cycle.
 * @param channel is the channel number between 0 and 5.
 * @param val determines the duty cycle which is given by
 * the expression duty = val/PWMCM * 100. 0 means off, 
 * greater than 0x7fff will cause the pwm to be on the 
 * whole period.
 * @return ERR_OK if successful.
 */
/**************************************************************************************/
byte PWM_A_setDuty (byte channel, int val)
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

/**************************************************************************************/
/**
 * sets the duty cycle as a percentage.
 * @param channel is the PWM channel to control (0-5).
 * @param duty is the duty cycle (0-100).
 * @return ERR_OK if successful.
 */
/**************************************************************************************/
byte PWM_A_setDutyPercent(byte channel,byte duty)
{
	register word dutyreg;

	if (duty>100)
		return ERR_RANGE;
		
	dutyreg = (word)((dword)getReg (PWMA_PWMCM) * duty / 100);
	switch (channel) 
	{
	case 0 :
		setReg(PWMA_PWMVAL0,dutyreg);
		break;
	case 1 :
		setReg(PWMA_PWMVAL1,dutyreg);
		break;
	case 2 :
		setReg(PWMA_PWMVAL2,dutyreg);
		break;
	case 3 :
		setReg(PWMA_PWMVAL3,dutyreg);
		break;
	case 4 :
		setReg(PWMA_PWMVAL4,dutyreg);
		break;
	case 5 :
		setReg(PWMA_PWMVAL5,dutyreg);
		break;
	default: 
		return ERR_RANGE;
	}
	
	return ERR_OK;
}
