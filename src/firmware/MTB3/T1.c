#include<p30f4011.h>
#include "T1.h"
#include "timer.h"
void T1_Init(unsigned int match_value)
{
	#warning "it has to be changed"
	// int match_value=0xFFFF;//0X9FFF; //value of the timer it correspond to 20 ms
	//==================================== INIT TIMER ================================
	ConfigIntTimer1(T1_INT_PRIOR_1 & T1_INT_ON);
	WriteTimer1(0);
	OpenTimer1(T1_ON & T1_GATE_OFF & T1_IDLE_STOP & T1_PS_1_64 & T1_SYNC_EXT_OFF & T1_SOURCE_INT, match_value);
	
}
