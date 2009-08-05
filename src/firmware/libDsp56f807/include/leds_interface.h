#ifndef __leds_h__
#define __leds_h__

#include "dsp56f807.h"

#define led0_on setRegBits(GPIO_A_DR,0x10);
#define led1_on setRegBits(GPIO_A_DR,0x20);
#define led2_on setRegBits(GPIO_A_DR,0x40);
#define led3_on setRegBits(GPIO_A_DR,0x80);


#define led0_off clrRegBits(GPIO_A_DR,0x10);
#define led1_off clrRegBits(GPIO_A_DR,0x20);
#define led2_off clrRegBits(GPIO_A_DR,0x40);
#define led3_off clrRegBits(GPIO_A_DR,0x80);

#define led0_status getRegBits(GPIO_A_DR,0x10);
#define led1_status getRegBits(GPIO_A_DR,0x20);
#define led2_status getRegBits(GPIO_A_DR,0x40);
#define led3_status getRegBits(GPIO_A_DR,0x80);
void init_leds(void);

void turn_led_on(byte number);
void turn_led_off(byte number);

#endif 
