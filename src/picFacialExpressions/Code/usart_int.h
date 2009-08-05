#define USART_BAUD			0x68
#define USART_BUFFER_SIZE 	16

#define USART_NOTOK		0
#define USART_OK		1

#define BAUD_9600		1
#define BAUD_115200		2

void USARTopen(unsigned char bd);
char USARTwriteNbytesFromRAM (char * st, char count);
char USARTwriteNbytesFromROM (const rom char * st, char count);
char USARTwriteFromRAM (char * st);
char USARTwriteFromROM (const rom char * st);
char USARTreadToRAM (char *st);
char USARTreadNbytesToRAM (char *st, unsigned char cnt);
