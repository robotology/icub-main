 
#include "pwm_b.h"
#include "faults_interface.h"
#include "brushless_comm.h"

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
	// PWMB_PMCTL: LDFQ=0,HALF=0,IPOL2=0,IPOL1=0,IPOL0=0,PRSC=0,PWMRIE=0,PWMF=0,ISENS=0,LDOK=0,PWMEN=0 
	setReg (PWMB_PMCTL, 0);
	          
  	// PWMB_PMOUT: PAD_EN=0,??=0,OUTCTL=0,??=0,??=0,OUT=0 
 	setReg(PWMB_PMOUT, 0);     

	// PWMB_PMCCR: ENHA=1,??=0,MSK=0,??=0,??=0,VLMODE=0,??=0,SWP45=0,SWP23=0,SWP01=0 
	setReg(PWMB_PMCCR, 32768); 
	            
	// PWMB_PMCFG: ??=0,??=0,??=0,EDG=1,??=0,TOPNEG45=0,TOPNEG23=0,TOPNEG01=0,??=0,BOTNEG45=0,BOTNEG23=0,BOTNEG01=0,INDEP45=0,INDEP23=0,INDEP01=0,WP=0 
	setReg(PWMB_PMCFG, 0x1000);           

	// PWMB_PMDEADTM: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PWMDT=4 
	setReg (PWMB_PMDEADTM, DEAD_TIME);          

           
  	// PWMB_PWMVAL0: PWMVAL=MIN_DUTY 
  	setReg(PWMB_PWMVAL0, MIN_DUTY);           
  	// PWMB_PWMVAL1: PWMVAL=MAX_DUTY 
  	setReg(PWMB_PWMVAL1, MAX_DUTY);          
  	// PWMB_PWMVAL2: PWMVAL=MIN_DUTY 
  	setReg(PWMB_PWMVAL2, MIN_DUTY);           
    // PWMB_PWMVAL3: PWMVAL=MAX_DUTY 
 	setReg(PWMB_PWMVAL3, MAX_DUTY);          
 	// PWMB_PWMVAL4: PWMVAL=MIN_DUTY 
	setReg(PWMB_PWMVAL4, MIN_DUTY);           
 	// PWMB_PWMVAL5: PWMVAL=MAX_DUTY 
	setReg(PWMB_PWMVAL5, MAX_DUTY);      
      
 	// PWMB_PWMCM: ??=0,PWMCM=PWMFREQ
  	setReg(PWMB_PWMCM, PWMFREQ);     
  
  
	// PWMB_PMCTL: LDOK=1,PWMEN=1 
	setRegBits (PWMB_PMCTL, 3);     
	
	// write protect on 
	setRegBits (PWMB_PMCFG, PWMB_PMCFG_WP_MASK);  
}

/**
 * Enables the PWM pad and clears fault pins.
 * @return ERR_OK always.
 */
 
void PWM_B_outputPadEnable (word mask)
{
	DutyCycle[1].Duty = MIN_DUTY;
	DutyCycleReq[1].Duty = MIN_DUTY;
	PWM_generate_BLL(1,DutyCycle[1].Duty);
	setRegBit(PWMB_PMOUT,PAD_EN);
		reset_faults_PWMB();
	
}


void PWM_B_outputPadDisable (word mask)
{
	clrRegBit(PWMB_PMOUT,PAD_EN);
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