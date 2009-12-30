#include<p30f4011.h>
#include "LED.h"

extern void Wait(unsigned int value);

void LED_Init()
{
	TRISFbits.TRISF5=0;
	led0=0;	
}

void LED_test()
{
	led0=0;
	Wait(100);
	led0=1;
	Wait(100);
}
