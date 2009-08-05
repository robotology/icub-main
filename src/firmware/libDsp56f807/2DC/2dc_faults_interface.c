
#include "faults_interface.h"
#include "asc.h"
#include "pwm_interface.h"

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
		// FAULT0    signal disables PWMx channels 0123
		// FAULT1    signal disables PWMx channels 0123		 
		setRegBits (PWMA_PMDISMAP1, 0b0001000100010001);		
		setRegBits (PWMA_PMDISMAP2, 0b0000000000000000);   
		setRegBits (PWMB_PMDISMAP1, 0b0001000100010001);
		setRegBits (PWMB_PMDISMAP2, 0b0000000000000000);  
		if (interrupts_enable==true)
		{
			setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK |
									PWMA_PMFCTL_FIE1_MASK );
			setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK |
									PWMB_PMFCTL_FIE1_MASK );
		}
	}	
	
	if (external_fault_enable==true)
	{
		setRegBits (PWMA_PMDISMAP1, 0b0010001000100010);
		setRegBits (PWMA_PMDISMAP2, 0);
		setRegBits (PWMB_PMDISMAP1, 0b0010001000100010);
		setRegBits (PWMB_PMDISMAP2, 0); 
	}
}
		
void FaultA_Interrupt(void);
void FaultB_Interrupt(void);

#pragma interrupt saveall
void FaultA_Interrupt(void)
{
	PWM_outputPadDisable(0);
	AS1_printStringEx ("Fault PWMA!");
	AS1_printStringEx ("\r\n");	
	AS1_printStringEx ("PWMA_PFMFSA = ");
	AS1_printUWord16AsChars (getReg (PWMA_PMFSA));
	AS1_printStringEx ("\r\n");
	setRegBits(PWMA_PMFSA, 0b0000000001010101);
	AS1_printUWord16AsChars (getReg (PWMA_PMFSA));
}

#pragma interrupt saveall
void FaultB_Interrupt(void)
{
	PWM_outputPadDisable(1);
	AS1_printStringEx ("Fault PWMB!");
	AS1_printStringEx ("\r\n");
	AS1_printStringEx ("PWMB_PFMFSA = ");
	AS1_printUWord16AsChars (getReg (PWMB_PMFSA));
	AS1_printStringEx ("\r\n");
	setRegBits(PWMB_PMFSA, 0b0000000001010101);
	AS1_printUWord16AsChars (getReg (PWMB_PMFSA));		
}
