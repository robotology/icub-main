#include "dsp56f807.h"

#include "faults_interface.h"
#include "pwm_interface.h"
#include "asc.h"

/******************************************************************************/
/**
 * 
 ******************************************************************************/
void init_faults(bool enable, bool interrupts_enable)
{	            
	if (enable==true)
	{
		// manual fault reset 
		// UVLO       signal disables PWMx channels 0123
		
		// DRV1FAULT  signal disbales PWMA channels 01
		// DRV2FAULT  signal disbales PWMA channels 23
		// EXTFAULT12 signal disbales PWMA channels 0123
		
		// DRV3FAULT  signal disbales PWMB channels 01
		// DRV4FAULT  signal disbales PWMB channels 23
		// EXTFAULT34 signal disbales PWMB channels 0123
		/*		
		setReg (PWMA_PMFCTL, 	0x00); 
		setReg (PWMA_PMDISMAP1, 0b1101110110111011);
		setReg (PWMA_PMDISMAP2, 0b0000000000000000);  
		setReg (PWMB_PMFCTL, 	0x00); 
		setReg (PWMB_PMDISMAP1, 0b1101110110111011);
		setReg (PWMB_PMDISMAP2, 0b0000000000000000); 
		*/
	//---	setReg (PWMA_PMFCTL, 	0x00); 
		setReg (PWMA_PMFCTL, 	0b01010101);
		setReg (PWMA_PMDISMAP1, 0b0101010100110011);
		setReg (PWMA_PMDISMAP2, 0b0000000000000000);  
	//---		setReg (PWMB_PMFCTL, 	0x00); 
		setReg (PWMB_PMFCTL, 	0b01010101);
		setReg (PWMB_PMDISMAP1, 0b0101010100110011);
		setReg (PWMB_PMDISMAP2, 0b0000000000000000);
		if (interrupts_enable==true)
		{
			setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE0_MASK |
									PWMA_PMFCTL_FIE1_MASK |
									PWMA_PMFCTL_FIE2_MASK |
									PWMA_PMFCTL_FIE3_MASK );
			setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE0_MASK |
									PWMB_PMFCTL_FIE1_MASK |
									PWMB_PMFCTL_FIE2_MASK |
									PWMB_PMFCTL_FIE3_MASK );
		}
	}
	else
	{	
		// fault disable
		setReg (PWMA_PMFCTL, 	0b01010101);
		setReg (PWMA_PMDISMAP1, 0);          
		setReg (PWMA_PMDISMAP2, 0); 
		setReg (PWMB_PMFCTL, 	0b01010101);
		setReg (PWMB_PMDISMAP1, 0);          
		setReg (PWMB_PMDISMAP2, 0);     		
	}
	
	clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE3_MASK);
	clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE3_MASK);
}

void FaultA_Interrupt(void);
void FaultB_Interrupt(void);

/******************************************************************************/
/**
 * 
 ******************************************************************************/
void FaultInterruptDisable(byte axis)
{
	if 		(axis==0) clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE1_MASK);
	else if (axis==1) clrRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE2_MASK);
	else if (axis==2) clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE1_MASK);
	else if (axis==3) clrRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE2_MASK);	
}

/******************************************************************************/
/**
 * 
 ******************************************************************************/
void FaultInterruptEnable(byte axis)
{
	if 		(axis==0) setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE1_MASK);
	else if (axis==1) setRegBits (PWMA_PMFCTL,PWMA_PMFCTL_FIE2_MASK);
	else if (axis==2) setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE1_MASK);
	else if (axis==3) setRegBits (PWMB_PMFCTL,PWMB_PMFCTL_FIE2_MASK);	
}

/******************************************************************************/
/**
 * 
 ******************************************************************************/	
#pragma interrupt saveall
void FaultA_Interrupt(void)
{
	int status=0;
/*	
	AS1_printStringEx ("Fault PWMA!");
	AS1_printStringEx ("\r\n");	
	AS1_printStringEx ("PWMA_PFMFSA = ");
	AS1_printUWord16AsChars (getReg (PWMA_PMFSA));
	AS1_printStringEx ("\r\n");	
*/	
	status = getReg (PWMA_PMFSA);

	if (status & PWMA_PMFSA_FPIN0_MASK) 
	{
	PWM_outputPadDisable(0);
	PWM_outputPadDisable(1);	
	}	
	if (status & PWMA_PMFSA_FPIN1_MASK) 
	{
	PWM_outputPadDisable(0);	
	}
	if (status & PWMA_PMFSA_FPIN2_MASK) 
	{
	PWM_outputPadDisable(1);	
	}
	if (status & PWMA_PMFSA_FPIN3_MASK) 
	{
	PWM_outputPadDisable(0);
	PWM_outputPadDisable(1);	
	}	
}

/******************************************************************************/
/**
 * 
 ******************************************************************************/
#pragma interrupt saveall
void FaultB_Interrupt(void)
{
	int status=0;
/*	
	AS1_printStringEx ("Fault PWMB!");
	AS1_printStringEx ("\r\n");
	AS1_printStringEx ("PWMB_PFMFSA = ");
	AS1_printUWord16AsChars (getReg (PWMA_PMFSA));
	AS1_printStringEx ("\r\n");		
*/	
	status = getReg (PWMA_PMFSA);
	
	if (status & PWMA_PMFSA_FPIN0_MASK) 
	{
	PWM_outputPadDisable(2);
	PWM_outputPadDisable(3);	
	}
	if (status & PWMB_PMFSA_FPIN1_MASK) 
	{
	PWM_outputPadDisable(2);	
	}
	if (status & PWMB_PMFSA_FPIN2_MASK) 
	{
	PWM_outputPadDisable(3);	
	}
	if (status & PWMA_PMFSA_FPIN3_MASK) 
	{
	PWM_outputPadDisable(2);
	PWM_outputPadDisable(3);	
	}	
}