#include "dsp56f807.h"

#include "faults_interface.h"
#include "pwm_interface.h"
#include "asc.h"
#include "can1.h"

void FaultA_Interrupt(void);
void FaultB_Interrupt(void);

/******************************************************************************/
/**
 * 
 ******************************************************************************/
void init_faults(bool internal_fault_enable,  bool external_fault_enable, bool interrupts_enable)
{

	#warning //so far we are not using the DRV1_2_3_4FAULT
	  
	
	setReg (PWMA_PMFCTL, 	0); 
	setReg (PWMA_PMDISMAP1, 0);
	setReg (PWMA_PMDISMAP2, 0);  
	setReg (PWMB_PMFCTL, 	0); 
	setReg (PWMB_PMDISMAP1, 0);
	setReg (PWMB_PMDISMAP2, 0);  
	if (internal_fault_enable==true)
	{
		// manual fault reset
		// FAULT0    signal disables PWMx channels 0123
		// FAULT1    signal disables PWMx channels 0123		 
		setRegBits (PWMA_PMDISMAP1, 0x1111);//setRegBits (PWMA_PMDISMAP1, 0b0001000100010001);		
		setRegBits (PWMA_PMDISMAP2, 0x00);   
		setRegBits (PWMB_PMDISMAP1, 0x1111);
		setRegBits (PWMB_PMDISMAP2, 0x00);  
		if (interrupts_enable==true)
		{
			setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK |
									PWMA_PMFCTL_FIE3_MASK);
			setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK |					
									PWMB_PMFCTL_FIE3_MASK);
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


/******************************************************************************/
/**
 * 
 ******************************************************************************/
#pragma interrupt called
void FaultInterruptDisable(byte axis)
{
	if 		(axis<=1)
	{
		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK);	
//		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE1_MASK);	
//		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE2_MASK);	
		clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE3_MASK);	
	}
	else
	{
		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK);	
//		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE1_MASK);	
//		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE2_MASK);	
		clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE3_MASK);		
	}

}

/******************************************************************************/
/**
 * 
 ******************************************************************************/
void FaultInterruptEnable(byte axis)
{

				
	if 		(axis<=1)
	{
		setRegBits(PWMA_PMFSA, 	PWMA_PMFSA_FTACK0_MASK);
		setRegBits(PWMA_PMFSA, 	PWMA_PMFSA_FTACK3_MASK);
		
		
		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK);	
//		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE1_MASK);	
//		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE2_MASK);	
		setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE3_MASK);	
	}
	else if (axis>1)
	{
		setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK0_MASK);
		setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK3_MASK);
	
		setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK);	
//		setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE1_MASK);	
//		setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE2_MASK);	
	    setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE3_MASK);		
	}

}

/******************************************************************************/
/**
 * 
 ******************************************************************************/	
#pragma interrupt saveall
void FaultA_Interrupt(void)
{

	int status=0;

	status = getReg (PWMA_PMFSA);

	if (status & PWMB_PMFSA_FFLAG0_MASK) 
	{
	PWM_outputPadDisable(0);
	PWM_outputPadDisable(1);	
	}	
	if (status & PWMB_PMFSA_FFLAG1_MASK) 
	{
	PWM_outputPadDisable(0);	
	}
	if (status & PWMB_PMFSA_FFLAG2_MASK) 
	{
	PWM_outputPadDisable(1);	
	}
	if (status & PWMB_PMFSA_FFLAG3_MASK) 
	{
	PWM_outputPadDisable(0);
	PWM_outputPadDisable(1);	
	}
		
//	setRegBits(PWMA_PMFSA, 	PWMA_PMFSA_FTACK0_MASK);
//	setRegBits(PWMA_PMFSA, 	PWMA_PMFSA_FTACK3_MASK);
//	#ifdef DEBUG_CAN_MSG
	can_printf("FAULT AXIS: 0 & 1 %d",status);
//   #endif
}

/******************************************************************************/
/**
 * 
 ******************************************************************************/
#pragma interrupt saveall
void FaultB_Interrupt(void)
{
	int status=0;

	status = getReg (PWMB_PMFSA);
	
	if (status & PWMB_PMFSA_FFLAG0_MASK) 
	{
	PWM_outputPadDisable(2);
	PWM_outputPadDisable(3);	
	}
	if (status & PWMB_PMFSA_FFLAG1_MASK) 
	{
	PWM_outputPadDisable(2);	
	}
	if (status & PWMB_PMFSA_FFLAG2_MASK) 
	{
	PWM_outputPadDisable(3);	
	}
	if (status & PWMB_PMFSA_FFLAG3_MASK) 
	{
	PWM_outputPadDisable(2);
	PWM_outputPadDisable(3);	
	}	
	
//	setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK0_MASK);
//	setRegBits(PWMB_PMFSA, 	PWMB_PMFSA_FTACK3_MASK);
//	#ifdef DEBUG_CAN_MSG
 //   can_printf("FAULT AXIS: 2 & 3 %d",status);
 //   #endif
	//
}