#include <p18f2550.h>
#include <usart.h>
#include <string.h>
#include "usart_int.h"

unsigned char buffUSARTenvio[USART_BUFFER_SIZE];
unsigned char buffUSARTrecep[USART_BUFFER_SIZE];
unsigned char counUSARTenvio;
unsigned char posiUSARTenvio;
unsigned char counUSARTrecep;
unsigned char posiUSARTrecep;
unsigned char baudrateUSART;

unsigned char lastcounUSART_recep;


void USARTopen(unsigned char bd)
{
	char i;
	unsigned char baud, bconf1, bconf2;

	CloseUSART();

	for (i=0;i<USART_BUFFER_SIZE;i++) 
	{
		buffUSARTenvio[i] = 0;
		buffUSARTrecep[i] = 0;
	}
	counUSARTenvio = 0;
	counUSARTrecep = 0;
	posiUSARTenvio = 0;
	posiUSARTrecep = 0;

	if (bd == BAUD_9600)
	{
		baudrateUSART = BAUD_9600;
		bconf1 = USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT \
				& USART_CONT_RX & USART_BRGH_LOW;
		bconf2 = 0;
		baud = 0x68;
	}
	else /* BAUD_115200 */
	{
		baudrateUSART = BAUD_115200;
		bconf1 = USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & USART_EIGHT_BIT \
				& USART_CONT_RX & USART_BRGH_LOW;
		bconf2 = 0;
		baud = 51; //51; // 34;
	}
	/* USART a 9600bauds @ 4MHz = 0x68 */
	/* USART a 9600bauds @16MHz c/BRGH LOW = 0x68*/
	OpenUSART( bconf1 , baud);
	baudUSART (BAUD_16_BIT_RATE & BAUD_WAKEUP_OFF & BAUD_AUTO_OFF);	
	
	PIE1bits.RCIE	= 0;
	PIE1bits.TXIE	= 0;
	PIR1bits.RCIF = 0;
	PIR1bits.TXIF = 0;
	PIE1bits.RCIE	= 1;
	PIE1bits.TXIE	= 1;
}

char USARTwriteNbytesFromRAM (char * st, char count)
{
	if ((counUSARTenvio > 0) || (count > USART_BUFFER_SIZE)) return USART_NOTOK;
	else
	{
		memcpy((void *)buffUSARTenvio, (void *)st, count);
		counUSARTenvio 	= count - 1;
		posiUSARTenvio  = 1;
		TXREG = buffUSARTenvio [0];
		PIE1bits.TXIE = 1;
		return USART_OK;
	}
}

char USARTwriteNbytesFromROM (const rom char * st, char count)
{
	if ((counUSARTenvio > 0) || (count > USART_BUFFER_SIZE)) return USART_NOTOK;
	else
	{
		memcpypgm2ram((void *)buffUSARTenvio, (void *)st, count);
		counUSARTenvio 	= count - 1;
		posiUSARTenvio  = 1;
		TXREG = buffUSARTenvio [0];
		PIE1bits.TXIE = 1;
		return USART_OK;
	}
}

char USARTwriteFromRAM (char * st)
{
	if (counUSARTenvio > 0) return USART_NOTOK;
	else
	{
		strcpy((void *)buffUSARTenvio, (void *)st);
		counUSARTenvio 	= (strlen(st)) - 1;
		TXREG = buffUSARTenvio [0];
		PIE1bits.TXIE = 1;
		posiUSARTenvio  = 1;
		return USART_OK;
	}
}

char USARTwriteFromROM (const rom char * st)
{
	if (counUSARTenvio > 0) return USART_NOTOK;
	else
	{
		strcpypgm2ram((void *)buffUSARTenvio, (void *)st);
		counUSARTenvio 	= (strlen(st)) - 1;
		TXREG = buffUSARTenvio [0];
		PIE1bits.TXIE = 1;
		posiUSARTenvio  = 1;
		return USART_OK;
	}
}

char USARTreadToRAM (char *st)
{
	char count;
	if (counUSARTrecep < 1) return USART_NOTOK;
	count = counUSARTrecep;
	memcpy( (void *) st, (void *) buffUSARTrecep, counUSARTrecep);
//	buffUSARTrecep[0] = 0;
	st[count]=0;
	counUSARTrecep = 0;
	//counUSARTrecep = ((int)counUSARTrecep - count) >= 0 ? counUSARTrecep - count : 0;
	return USART_OK;
}

char USARTreadNbytesToRAM (char *st, unsigned char cnt)
{
	char count;
	if (counUSARTrecep < cnt) return USART_NOTOK;
	count = cnt;
	st[0] = 0;

	memcpy( (void *) st, (void *) buffUSARTrecep, count);
	buffUSARTrecep[0] = 0;
	st[count]=0;
	lastcounUSART_recep = counUSARTrecep;
//	counUSARTrecep = ((int)counUSARTrecep - count) >= 0 ? counUSARTrecep - count : 0;
	counUSARTrecep = 0;
	return USART_OK;
}
