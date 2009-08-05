
#include "faults_interface.h"
#include "pwm_interface.h"
#include "asc.h"
#include "leds_interface.h"
#include "can1.h"

/**************************************************************************************/
/**
 * 
 *
 **************************************************************************************/

void init_faults(bool internal_fault_enable,  bool external_fault_enable, bool interrupts_enable)
{	      
	setReg (PWMA_PMFCTL, 	0); 
	setReg (PWMA_PMDISMAP1, 0);
	setReg (PWMA_PMDISMAP2, 0);  
	setReg (PWMB_PMFCTL, 	0); 
	setReg (PWMB_PMDISMAP1, 0);
	setReg (PWMB_PMDISMAP2, 0); 	   
	if (internal_fault_enable==true)
	{
		// manual fault reset 
		// UVLO      signal disables PWMx channels 012345
		
		// OVL1      signal disables PWMA channels 012345
		// EXTFAULT1 signal disables PWMA channels 012345
		
		// OVL2      signal disables PWMB channels 012345
		// EXTFAULT2 signal disables PWMB channels 012345
		setRegBits (PWMA_PMDISMAP1, 0x3333);		
		setRegBits (PWMA_PMDISMAP2, 0x33);   
		setRegBits (PWMB_PMDISMAP1, 0x3333);
		setRegBits (PWMB_PMDISMAP2,	0x33);   
		
		if (interrupts_enable==true)
		{
			setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK |
									PWMA_PMFCTL_FIE1_MASK |
									//PWMA_PMFCTL_FIE2_MASK |
									PWMA_PMFCTL_FIE3_MASK );
			setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK |
									PWMB_PMFCTL_FIE1_MASK |
									//PWMB_PMFCTL_FIE2_MASK |
									PWMB_PMFCTL_FIE3_MASK );
		}  	
	}
	if (external_fault_enable==true)
	{
		setRegBits (PWMA_PMDISMAP1, 0x8888);
		setRegBits (PWMA_PMDISMAP2, 0);
		setRegBits (PWMB_PMDISMAP1, 0x8888);
		setRegBits (PWMB_PMDISMAP2, 0); 
	}
}

void FaultA_Interrupt(void);
void FaultB_Interrupt(void);

/******************************************************************************/
/**
 * 
 ******************************************************************************/
#pragma interrupt saveall
void FaultA_Interrupt(void)
{
	unsigned int FA=getReg (PWMA_PMFSA);

#ifdef DEBUG_CAN_MSG
	can_printf("FAULT 0: %d", FA);  	
#endif
 	
 	PWM_outputPadDisable(0); 	
	led1_on
	// Auto Reset of the fault
//	setRegBits(PWMA_PMFSA,	PWMA_PMFSA_FTACK3_MASK);
	//setRegBits(PWMA_PMFSA,	PWMA_PMFSA_FTACK2_MASK);

 	

}

/******************************************************************************/
/**
 * 
 ******************************************************************************/
#pragma interrupt saveall
void FaultB_Interrupt(void)
{
	unsigned int FB=getReg (PWMB_PMFSA);
#ifdef DEBUG_CAN_MSG
	can_printf("FAULT 1: %d", FB);
#endif
	
	PWM_outputPadDisable(1);
	led3_on
	// Auto Reset of the fault
//	setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK3_MASK);
	//setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK2_MASK);
}
/******************************************************************************/
/**
 * 
 ******************************************************************************/
void FaultInterruptDisable(byte axis)
{
	if 		(axis==0) 
	{
		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK);	
		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE1_MASK);	
		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE2_MASK);	
		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE3_MASK);	
	}
					  	
	else if (axis==1) 
	{
		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK);	
		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE1_MASK);	
		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE2_MASK);	
		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE3_MASK);	
	}
}

/******************************************************************************/
/**
 * 
 ******************************************************************************/
void FaultInterruptEnable(byte axis)
{
	if (axis==0) 
	{
		setRegBits(PWMA_PMFSA, 	PWMA_PMFSA_FTACK0_MASK);
		setRegBits(PWMA_PMFSA, 	PWMA_PMFSA_FTACK1_MASK);
		setRegBits(PWMA_PMFSA, 	PWMA_PMFSA_FTACK3_MASK);
		
		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK);
		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE1_MASK);
		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE2_MASK);
		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE3_MASK);
		
	}
	else if (axis==1) 
	{
		setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK0_MASK);
		setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK1_MASK);
		setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK3_MASK);
		
		setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK);
		setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE1_MASK);
		setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE2_MASK);
		setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE3_MASK);
	}
}