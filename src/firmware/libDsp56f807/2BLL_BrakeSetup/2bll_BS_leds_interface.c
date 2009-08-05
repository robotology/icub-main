
#include "leds_interface.h"



/***************************************************************************/
/**
 * This method inits the LED interface for the brushless board
 ***************************************************************************/
void init_leds(void)
{
	setRegBits(GPIO_A_DDR,0xF0);   
	clrRegBits(GPIO_A_PER,0xF0); 
	clrRegBits(GPIO_A_DR, 0xF0);		
}

/***************************************************************************/
/**
 * This method turns on the specified LED
 ***************************************************************************/
//#pragma interrupt called
void turn_led_on(byte number)
{
	if (number>=0 && number<4)	setRegBits(GPIO_A_DR,(0x10 << number));
}



/***************************************************************************/
/**
 * This method turns off the specified LED
 ***************************************************************************/

void turn_led_off(byte number)
{
	if (number>=0 && number<4)	clrRegBits(GPIO_A_DR,(0x10 << number));
}
