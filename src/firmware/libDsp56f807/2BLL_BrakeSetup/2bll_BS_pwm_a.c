/**
 * \file pwm_a.c
 *	the implementation of the PWM channel A interface.
 */
 
#include "pwm_a.h"
#include "faults_interface.h"
#include "brushless_comm.h"

extern sDutyControlBL DutyCycle[2];
extern sDutyControlBL DutyCycleReq[2];


void PWM_A_Write_Protect();
/**************************************************************************************/
/**
 * initializes the PWM module w/ 200KHz indipendent mode.
 *
 **************************************************************************************/
void PWM_A_init(void)
{
	
	
		
	
	// PWMA_PMCTL: LDFQ=0,HALF=0,IPOL2=0,IPOL1=0,IPOL0=0,PRSC=0,PWMRIE=0,PWMF=0,ISENS=0,LDOK=0,PWMEN=0 
	setReg (PWMA_PMCTL, 0);
	          
  	// PWMA_PMOUT: PAD_EN=0,??=0,OUTCTL=0,??=0,??=0,OUT=0 
 	setReg(PWMA_PMOUT, 0);     

	// PWMA_PMCCR: ENHA=1,??=0,MSK=0,??=0,??=0,VLMODE=0,??=0,SWP45=0,SWP23=0,SWP01=0 
	setReg(PWMA_PMCCR, 32768); 
	            
	// PWMA_PMCFG: ??=0,??=0,??=0,EDG=1,??=0,TOPNEG45=0,TOPNEG23=0,TOPNEG01=0,??=0,BOTNEG45=0,BOTNEG23=0,BOTNEG01=0,INDEP45=0,INDEP23=0,INDEP01=0,WP=0 
	setReg(PWMA_PMCFG, 0x1000);           

	// PWMA_PMDEADTM: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PWMDT=4 
	setReg (PWMA_PMDEADTM, 4);          


		
           
  	// PWMA_PWMVAL0: PWMVAL=MIN_DUTY 
  	setReg(PWMA_PWMVAL0, MIN_DUTY);           
  	// PWMA_PWMVAL1: PWMVAL=MAX_DUTY 
  	setReg(PWMA_PWMVAL1, MAX_DUTY);          
  	// PWMA_PWMVAL2: PWMVAL=MIN_DUTY 
  	setReg(PWMA_PWMVAL2, MIN_DUTY);           
    // PWMA_PWMVAL3: PWMVAL=MAX_DUTY 
 	setReg(PWMA_PWMVAL3, MAX_DUTY);          
 	// PWMA_PWMVAL4: PWMVAL=MIN_DUTY 
	setReg(PWMA_PWMVAL4, MIN_DUTY);           
 	// PWMA_PWMVAL5: PWMVAL=MAX_DUTY 
	setReg(PWMA_PWMVAL5, MAX_DUTY);      
      
 	// PWMA_PWMCM: ??=0,PWMCM=PWMFREQ
  	setReg(PWMA_PWMCM, PWMFREQ);     
  
  
	// PWMA_PMCTL: LDOK=1,PWMEN=1 
	setRegBits (PWMA_PMCTL, 3);     
	
	// write protect on 
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

	DutyCycle[0].Duty = MIN_DUTY;
	DutyCycleReq[0].Duty = MIN_DUTY;
	PWM_generate_BLL(0,DutyCycle[0].Duty);
	setRegBit(PWMA_PMOUT,PAD_EN);	
	reset_faults_PWMA();
}

/**************************************************************************************/

void PWM_A_outputPadDisable (word mask)
{
	clrRegBit(PWMA_PMOUT,PAD_EN);
}
void PWM_A_Write_Protect()
{
	// write protect on 
	setRegBits (PWMA_PMCFG, PWMA_PMCFG_WP_MASK); 
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

