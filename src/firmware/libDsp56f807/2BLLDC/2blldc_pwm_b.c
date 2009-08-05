 
#include "pwm_b.h"
#include "faults_interface.h"
#include "brushess_comm.h"
// the PWM limit are MIN_DUTY= 2%   and MAX_DUTY=98%
//#define PWMCM    1333   //it correspond to 30KHz PWM
//#define MIN_DUTY 58 // 2% of  1333 + deadtime*2
//#define MAX_DUTY 1306   // 98% of 1333

extern sDutyControlBL DutyCycle[2];
extern sDutyControlBL DutyCycleReq[2];

void PWM_B_Write_Protect();
/**************************************************************************************/
/**
 * initializes the PWM module w/ 30KHz indipendent mode.
 *
 **************************************************************************************/
void PWM_B_init(void)
{

	/* PWMB_PMCTL: LDFQ=0,HALF=0,IPOL2=0,IPOL1=0,IPOL0=0,PRSC=0,PWMRIE=0,PWMF=0,ISENS=0,LDOK=0,PWMEN=0 */
	setReg (PWMB_PMCTL, 0);
	            
	/* PWMB_PMOUT: PAD_EN=0,??=0,OUTCTL=001111,??=0,??=0,OUT=0 */
	setReg (PWMB_PMOUT, 0x0F);

	// PWMB_PMCCR: ENHA=1,??=0,MSK=0,??=0,??=0,VLMODE=0,??=0,SWP45=0,SWP23=0,SWP01=0 
	setReg(PWMB_PMCCR, 0x8000); 
	          
	/* PWMB_PMCFG: ??=0,??=0,??=0,EDG=0,??=0,TOPNEG45=0,TOPNEG23=0,TOPNEG01=0,??=0,BOTNEG45=0,BOTNEG23=0,BOTNEG01=0,INDEP45=1,INDEP23=0,INDEP01=0,WP=0 */
	setReg (PWMB_PMCFG, 0x1008);           

	/* PWMB_PMDEADTM: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PWMDT=40 */
	setReg (PWMB_PMDEADTM, DEAD_TIME);          
	
	/* PWMB_PWMVAL0: PWMVAL=0 */
	setReg (PWMB_PWMVAL0, MIN_DUTY);            

	/* PWMB_PWMVAL1: PWMVAL=1333 */
//	setReg (PWMB_PWMVAL1, MAX_DUTY);         

	/* PWMB_PWMVAL2: PWMVAL=0 */
	setReg (PWMB_PWMVAL2, MIN_DUTY);            

	/* PWMB_PWMVAL3: PWMVAL=1333 */
//	setReg (PWMB_PWMVAL3, MAX_DUTY);         

	/* PWMB_PWMVAL4: PWMVAL=0 */
//	setReg (PWMB_PWMVAL4, 0);            

	/* PWMB_PWMVAL5: PWMVAL=1333 */
//	setReg (PWMB_PWMVAL5, 0);         

	/* PWMB_PWMCM: ??=0,PWMCM=1333 i.e. 30KHz*/
	setReg (PWMB_PWMCM, PWMCM);           

	/* PWMB_PMCTL: LDOK=1,PWMEN=0 */
	setRegBits (PWMB_PMCTL, 2);         

	/* PWMB_PMCTL: LDOK=1,PWMEN=1 */
	setRegBits (PWMB_PMCTL,3);   

	/* write protect on */
	//setRegBits (PWMB_PMCFG, PWMB_PMCFG_WP_MASK);   
}

/**
 * Enables the PWM pad and clears fault pins.
 * @return ERR_OK always.
 */
 
void PWM_B_outputPadEnable (word mask)
{
 	DutyCycle[1].Duty = MIN_DUTY;
 	DutyCycle[1].Dir=0;
	DutyCycleReq[1].Duty = MIN_DUTY;
	DutyCycleReq[1].Dir=0;
	PWM_generate_DC(1,DutyCycle[1].Duty,DutyCycle[1].Dir);
	setRegBit(PWMB_PMOUT,PAD_EN);
	reset_faults_PWMB();
}

void PWM_B_outputPadDisable (word mask)
{
	setRegBits (PWMB_PMCFG, PWMB_PMCFG_WP_MASK); 
}
	
void PWM_B_Write_Protect()
{
	// write protect on 
	setRegBits (PWMB_PMCFG, PWMB_PMCFG_WP_MASK); 
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
	if (duty>MAX_DUTY) duty=MAX_DUTY;
	if (duty<MIN_DUTY) duty=MIN_DUTY;
	
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

	if (duty>98 && duty<2)
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