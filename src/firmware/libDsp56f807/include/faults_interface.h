#ifndef __faults_interface_h__
#define __faults_interface_h__

#include "dsp56f807.h"

void init_faults(bool internal_fault_enable,  bool external_fault_enable, bool interrupts_enable);

inline void reset_faults_PWMA(void)
{
	setReg (PWMA_PMFSA, 0x55);	
}

inline void reset_faults_PWMB(void)
{
	setReg (PWMB_PMFSA, 0x55);	
}

void FaultInterruptDisable (byte axis);
void FaultInterruptEnable  (byte axis);

#endif 
