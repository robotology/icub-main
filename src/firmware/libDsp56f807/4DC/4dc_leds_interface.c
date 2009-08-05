
#include "leds_interface.h"

/***************************************************************************/
/**
 * This method inits the LED interface for the 4DC motor board
 ***************************************************************************/
void init_leds(void)
{
	setRegBits(GPIO_A_DDR,0xF0);   
	clrRegBits(GPIO_A_PER,0xF0); 
	setRegBits(GPIO_A_DR, 0xF0);	
}


/***************************************************************************/
/**
 * This method turns on the specified LED
 ***************************************************************************/
void turn_led_on(byte number)
{
	if (number==0)
		clrRegBits(GPIO_A_DR,0x30);
	else
		clrRegBits(GPIO_A_DR,0xC0);
}

/***************************************************************************/
/**
 * This method turns off the specified LED
 ***************************************************************************/
void turn_led_off(byte number)
{
	if (number==0)
		setRegBits(GPIO_A_DR,0x30);
	else
		setRegBits(GPIO_A_DR,0xC0);	
}
