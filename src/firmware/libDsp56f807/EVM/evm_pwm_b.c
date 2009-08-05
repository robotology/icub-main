 
#include "pwm_b.h"
#include "faults_interface.h"

/**************************************************************************************/
/**
 * initializes the PWM module w/ 30KHz indipendent mode.
 *
 **************************************************************************************/
void PWM_B_init(void)
{
	/* PWMB_PMCTL: LDFQ=0,HALF=0,IPOL2=0,IPOL1=0,IPOL0=0,PRSC=0,PWMRIE=0,PWMF=0,ISENS=0,LDOK=0,PWMEN=0 */
	setReg (PWMB_PMCTL, 0);
	            
	/* PWMB_PMOUT: PAD_EN=1,??=0,OUTCTL=111111,??=0,??=0,OUT=0 */
	setReg (PWMB_PMOUT, 0b1011111100000000);

	/* PWMB_PMCCR: ENHA=0,??=0,MSK=0,??=0,??=0,VLMODE=0,??=0,SWP45=0,SWP23=0,SWP01=0 */
	setReg (PWMB_PMCCR, 0);
	            
	/* PWMB_PMCFG: ??=0,??=0,??=0,EDG=0,??=0,TOPNEG45=0,TOPNEG23=0,TOPNEG01=0,??=0,BOTNEG45=0,BOTNEG23=0,BOTNEG01=0,INDEP45=1,INDEP23=1,INDEP01=1,WP=0 */
	setReg (PWMB_PMCFG, 0x0E);           

	/* PWMB_PMDEADTM: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PWMDT=0 */
	setReg (PWMB_PMDEADTM, 0);          

	/* PWMB_PWMVAL0: PWMVAL=0 */
	setReg (PWMB_PWMVAL0, 0x0);            

	/* PWMB_PWMVAL1: PWMVAL=0 */
	setReg (PWMB_PWMVAL1, 0x0);         

	/* PWMB_PWMVAL2: PWMVAL=0 */
	setReg (PWMB_PWMVAL2, 0);            

	/* PWMB_PWMVAL3: PWMVAL=0 */
	setReg (PWMB_PWMVAL3, 0x0);         

	/* PWMB_PWMVAL4: PWMVAL=0 */
	setReg (PWMB_PWMVAL4, 0);            

	/* PWMB_PWMVAL5: PWMVAL=0 */
	setReg (PWMB_PWMVAL5, 0x0);         

	/* PWMB_PWMCM: ??=0,PWMCM=1333 i.e. 30KHz*/
	setReg (PWMB_PWMCM, 0x535);           

	/* PWMB_PMCTL: LDOK=1,PWMEN=1 */
	setRegBits (PWMB_PMCTL, 3);     
	
	/* write protect on */
	//setRegBits (PWMB_PMCFG, PWMB_PMCFG_WP_MASK);   	
}

/**
 * Enables the PWM pad and clears fault pins.
 * @return ERR_OK always.
 */
void PWM_B_outputPadEnable (word mask)
{
    mask &= 0b0011111100000000;
	clrRegBits(PWMB_PMOUT, mask);
	reset_faults_PWMB();
}

void PWM_B_outputPadDisable (word mask)
{
     mask &= 0b0011111100000000;
	 setRegBits(PWMB_PMOUT, mask);
}

/**
 * sets the period of the PWM signal.
 * @param period is the period of the PWM in the 15 bit range. This is the
 * modulo of the counter.
 * @return ERR_OK or ERR_RANGE.
 */
byte PWM_B_setPeriod (word period)
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
byte PWM_B_setDuty (byte channel, int duty)
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
byte PWM_B_setDutyPercent(byte channel,byte duty)
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
 * sets the clock prescaler.
 * @param presc is the prescaler value in range 0-3 that mean divisors 1 to 8.
 * @return ERR_OK if successful.
 */
byte PWM_B_setPrescaler(byte presc)
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