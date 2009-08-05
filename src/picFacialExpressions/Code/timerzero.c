#include <p18cxxx.h>
#include "timerzero.h"

unsigned char TMR0fim;

void espera_x10ms(unsigned char x10ms)
{
	unsigned int t;
	TMR0fim = 0;
	t = 0x10000 - (157 * x10ms);
	TMR0H = (unsigned char) (t >> 8) & 0xFF;
	TMR0L = (unsigned char) t & 0xFF;
	T0CONbits.TMR0ON = 1;
}
