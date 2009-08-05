/*
 * Async serial communication.
 * freely inspired by the metrowerks code :)
 */

/**
 * Asynchronous serial communication channel.
 * It only works on serial port 0 (SCI0).
 * \file asc.c
 */

#include "asc.h"

#define OVERRUN_ERR      1             /* Overrun error flag bit    */
#define FRAMING_ERR      2             /* Framing error flag bit    */
#define PARITY_ERR       4             /* Parity error flag bit     */
#define CHAR_IN_RX       8             /* Char is in RX buffer      */
#define FULL_TX          16            /* Full transmit buffer      */
#define RUNINT_FROM_TX   32            /* Interrupt is in progress  */
#define FULL_RX          64            /* Full receive buffer       */
#define NOISE_ERR        128           /* Noise erorr flag bit      */
#define IDLE_ERR         256           /* Idle character flag bit   */
#define BREAK_ERR        512           /* Break detect              */
#define COMMON_ERR       2048          /* Common error of RX       */

static word SerFlag;                   /* Flags for serial communication */
                                       /* Bits: 0 - OverRun error */
                                       /*       1 - Framing error */
                                       /*       2 - Parity error */
                                       /*       3 - Char in RX buffer */
                                       /*       4 - Full TX buffer */
                                       /*       5 - Running int from TX */
                                       /*       6 - Full RX buffer */
                                       /*       7 - Noise error */
                                       /*       8 - Idle character  */
                                       /*       9 - Break detected  */
                                       /*      10 - Unused */
                                       /*      11 - Unused */
static byte BufferRead;        		   /* Input char for SCI commmunication */
static byte BufferWrite;       		   /* Output char for SCI commmunication */

/*
 * prototypes.
 */
static void HWEnDi (void);


/**
 * enables the serial communication channel.
 */
static void HWEnDi(void)
{
 //   touchReg(SCI0_SCISR);                /* Reset interrupt request flags */
	asm (bftsth #$A800,X:$1302);
	asm (move X:$1303, R0);
	
    setRegBits(SCI0_SCICR, (SCI0_SCICR_TE_MASK | SCI0_SCICR_RE_MASK | SCI0_SCICR_RIE_MASK | SCI0_SCICR_REIE_MASK)); /* Enable device */
    
	/* Is any char in the transmit buffer? */
    if (SerFlag & FULL_TX) 
    {          
		//touchReg(SCI0_SCISR);              /* Reset interrupt request flags */
		asm (bftsth #$8000,X:$1302);
	
		while (!getRegBit(SCI0_SCISR, TDRE)) ; /* Wait for transmitter empty */
		setReg(SCI0_SCIDR, BufferWrite); /* Store char to the transmitter register */
		setRegBit(SCI0_SCICR, TEIE);     /* Enable transmit interrupt */
    }
}


/**
 * initializes the serial channel.
 */
void AS1_init(void)
{
	SerFlag = 0;                        /* Reset flags */
	setReg(SCI0_SCICR, 0);				/* clear */
	//setReg(SCI0_SCIBR, 22);				/* 115200 */
	setReg(SCI0_SCIBR, 260);			/* 9600 */
	HWEnDi();                           /* Enable/disable device according to status flags */
}

/**
 * sends a single character.
 * @param chr a byte to send.
 * @return 0 if all was ok or negative in case of error.
 */
byte AS1_sendChar(byte chr)
{
	if (SerFlag & FULL_TX)             /* Is any char is in TX buffer */
		return ERR_TXFULL;             /* If yes then error */
		
	EnterCritical();                   /* Disable global interrupts */
	
	asm (bftsth #$8000,X:$1302);
	
	setReg(SCI0_SCIDR, chr);           /* Store char to transmitter register */
	setRegBit(SCI0_SCICR, TEIE);       /* Enable transmit interrupt */

	SerFlag |= FULL_TX;                /* Set the flag "full TX buffer" */
	ExitCritical();                    /* Enable global interrupts */
	
	return ERR_OK;                     /* OK */
}

/**
 * sends a single character, safe to call from isr.
 * @param chr a byte to send.
 * @return 0 if all was ok or negative in case of error.
 */
#pragma interrupt called
byte AS1_sendCharSafe(byte chr)
{
	if (SerFlag & FULL_TX)             /* Is any char is in TX buffer */
		return ERR_TXFULL;             /* If yes then error */
		
	asm (bftsth #$8000,X:$1302);
	setReg(SCI0_SCIDR, chr);           /* Store char to transmitter register */
	setRegBit(SCI0_SCICR, TEIE);       /* Enable transmit interrupt */

	SerFlag |= FULL_TX;                /* Set the flag "full TX buffer" */
	
	return ERR_OK;                     /* OK */
}

/**
 * gets a single character from the serial port.
 * @param chr is a pointer to the byte received.
 * @return 0 if ok or a negative value if error.
 */
byte AS1_recvChar(byte *chr)
{
	register byte Result = ERR_OK;       /* Return error code */

	if (!(SerFlag & CHAR_IN_RX))         /* Is any char in RX buffer? */
		return ERR_RXEMPTY;                /* If no then error */

	EnterCritical();                     /* Disable global interrupts */
	*chr = BufferRead;                   /* Received char */
	Result = (byte)((SerFlag & (OVERRUN_ERR|COMMON_ERR))?ERR_COMMON:ERR_OK);
	SerFlag &= ~(OVERRUN_ERR|COMMON_ERR|CHAR_IN_RX); /* Clear all errors in the status variable */
	ExitCritical();                      /* Enable global interrupts */
	
	return Result;                       /* Return error code */
}

/**
 * recv interrupt handler.
 */
#define ON_ERROR    1
#define ON_FULL_RX  2
#define ON_RX_CHAR  4

#pragma interrupt saveall
void AS1_interruptRx(void)
{
	register byte Data;          		/* Temporary variable for data */
	register byte OnFlags = 0;    		/* Temporary variable for flags */

	asm (bftsth #$4800,X:$1302);		/* reset interrupt req flag */
	asm (bfset #$4800, X:$1302);		/* overrun ? */
	
	Data = (byte)getReg(SCI0_SCIDR); 	/* Read data from the receiver */
	if (SerFlag & CHAR_IN_RX)           /* Is any char already present in the receive buffer? */
		SerFlag |= OVERRUN_ERR;         /* If yes then set flag OVERRUN_ERR */
	
	SerFlag |= CHAR_IN_RX;              /* Set flag "char in RX buffer" */
	if (!(SerFlag & OVERRUN_ERR ))		/* Is an overrun detected? */
	{ 
		BufferRead = Data;
		OnFlags |= ON_RX_CHAR;          /* Set flag "OnRxChar" */
	}
}

/**
 * transmission interrupt handler.
 */
#define ON_FREE_TX  1
#define ON_TX_CHAR  2
#pragma interrupt saveall
void AS1_interruptTx(void)
{
	SerFlag &= ~FULL_TX;                 /* Reset flag "full TX buffer" */
	clrRegBit(SCI0_SCICR, TEIE);         /* Disable transmit interrupt */
}

/**
 * receive error interrupt handler.
 */
#pragma interrupt saveall
void AS1_interruptError(void)
{
	register word StatReg = getReg(SCI0_SCISR); /* Read status register */

	setReg(SCI0_SCISR, 0);               /* Reset error request flags */
	if(StatReg & (SCI0_SCISR_OR_MASK|SCI0_SCISR_NF_MASK|SCI0_SCISR_FE_MASK|SCI0_SCISR_PF_MASK)) /* Is an error detected? */
		SerFlag |= COMMON_ERR;             /* If yes then set an internal flag */
}

/**
 * returns the number of chars in the transmission buffer.
 * @return the number of chars in the tx buffer (either 1 or 0).
 */
word AS1_txBufferSize(void)
{
	return ((SerFlag & FULL_TX) != 0)? (word)1 : (word)0; /* Return number of chars in the transmitter buffer */
}

/**
 * gets the receiver buffer size.
 * @return the number of chars in the rx buffer (either 1 or 0).
 */
word AS1_rxBufferSize(void)
{
	return ((SerFlag & CHAR_IN_RX) != 0)? (word)1 : (word)0; /* Return number of chars in receive buffer */
}

/**
 * prints a string to terminal.
 * @param dataAddress is a pointer to the string.
 */
void AS1_printStringEx (char *dataAddress)
{
	int i = 0;
	int charsInBuf = 0;
	
	if (dataAddress == NULL)
		return;
		
	while (dataAddress[i] != 0)
	{
		do
		{
  			charsInBuf = AS1_txBufferSize();
  		}
  		while (charsInBuf > 0);
  		
  		AS1_sendChar(dataAddress[i]);
		i++;
	}
}

/**
 * prints a UWord16 to the terminal.
 * @param data is the variable to display (hex).
 */
void AS1_printUWord16AsChars (UWord16 data)
{
	UWord16 lsb;
	UWord16 buf;
	char c[4];
	int i;

	buf = data;
	for(i=3;i>=0;i--)
	{
		lsb = 0x000F & buf;
		c[i] = AS1_hexToChar(lsb);
		buf = __shr(buf, 4);
	}
	AS1_printString(c, 4);
}

/**
 * prints a Word16 to the terminal.
 * @param data is the variable to display (hex).
 */
void AS1_printWord16AsChars (Word16 data)
{
	UWord16 lsb;
	Word16 buf;
	UWord16 sign;
	char c[5];
	int i;

	sign = 0x8000 & data;
	
	if (sign != 0)
	{
		buf = __negate(data);
		c[0] = '-';
	}
	else
	{
		buf = 0x7FFF & data;
		c[0] = '+';
	}
		
	for(i=4;i>=1;i--)
	{
		lsb = 0x000F & buf;
		c[i] = AS1_hexToChar(lsb);
		buf = __shr(buf, 4);
	}
		
	AS1_printString(c, 5);	
}

/**
 * converts a hex value to char for display.
 * @param byte the hex to convert (lower 4 bits only).
 * @return the character (0-F) or * if outside range.
 */
char AS1_hexToChar (byte value)
{
	char c;
	switch(value)
	{
		case 0:
			c = '0';
			break;
		case 1:
			c = '1';
			break;
		case 2:
			c = '2';
			break;
		case 3:
			c = '3';
			break;
		case 4:
			c = '4';
			break;
		case 5:
			c = '5';
			break;
		case 6:
			c = '6';
			break;
		case 7:
			c = '7';
			break;
		case 8:
			c = '8';
			break;
		case 9:
			c = '9';
			break;
		case 10:
			c = 'a';
			break;
		case 11:
			c = 'b';
			break;
		case 12:
			c = 'c';
			break;
		case 13:
			c = 'd';
			break;
		case 14:
			c = 'e';
			break;
		case 15:
			c = 'f';
			break;
		default:
			c = '*';
			break;
	}
	return c;
}

#define _XSIZE 255
#define AS1_atoiEx (s) AS1_atoi (s, AS1_strlen(s, _XSIZE));

/**
 * converts from ascii to integer (16 bits).
 * @param strAddr is the pointer to the string to be converted.
 * @param strSize is the size of the string.
 * @return the result of the conversion.
 */
int AS1_atoi (char *strAddr, int strSize)
{
	int i, end;
	int val = 0;
	int _pow = 1;
	byte negative = 0;

	if (strSize > 255)
		return 0;
		
	if (strAddr[0] == '-')
	{
		negative = 1;
		end = 1;
	}
	else
	if (strAddr[0] == '+')
	{
		negative = 0;
		end = 1;
	}
	else
		end = 0;

	for (i = strSize-1; (i >= end && strAddr[i] != 0); i--)
	{
		if (strAddr[i] < 48 || strAddr[i] > 57)
			return 0;
		val += _pow * (strAddr[i] - 48);
		_pow *= 10;
	}
	
	if (negative == 1)
		return -val;
	else
		return val;
}


/**
 * converts from ascii to long (32 bits).
 * @param strAddr is the string to be converted.
 * @param strSize is the length of the string.
 * @return the result of the conversion.
 */
long AS1_atol (char *strAddr, int strSize)
{
	byte i, end;
	long val = 0;
	long _pow = 1;
	byte negative = 0;

	if (strSize > 255)
		return 0;
		
	if (strAddr[0] == '-')
	{
		negative = 1;
		end = 1;
	}
	else
	if (strAddr[0] == '+')
	{
		negative = 0;
		end = 1;
	}
	else
		end = 0;

	for (i = strSize-1; (i >= end && strAddr[i] != 0); i--)
	{
		if (strAddr[i] < 48 || strAddr[i] > 57)
			return 0;
		val += (_pow * (strAddr[i] - 48));
		_pow *= 10;
	}
	
	if (negative == 1)
		return -val;
	else
		return val;
}

/**
 * computes the length of a string not necessarily zero terminated.
 * @param strAddr is the string to be converted.
 * @param strSize is the maximum length of the string.
 * @return the length of the string.
 */
int AS1_strlen (char *strAddr, int strSize)
{
	int i = 0;
	while ((strAddr[i] != '\0') && (i<strSize))
	{
		i++;
	}
	return i;
}

/**
 *
 */
void AS1_printString (char *dataAddress, int dataSize)
{
	int i;
	int charsInBuf = 0;
	for (i = 0; i < dataSize; i++)
	{
		do
		{
  			charsInBuf = AS1_txBufferSize();
  		}
  		while (charsInBuf > 0);
  		
		AS1_sendChar (dataAddress[i]);
	}
}

/**
 * receives a string from the terminal.
 * @param dataAddress is the pointer to the buffer to receive the data.
 * @param dataSize is the maximum number of bytes read (well, they're actually 16 bit words).
 */
void AS1_getString (char *dataAddress, int dataSize)
{
	int i;
	int readChars;
	unsigned char c;
	for (i = 0; i < dataSize; i++)
	{
		do
  			readChars = AS1_rxBufferSize ();
  		while (readChars == 0);
  		
		AS1_recvChar (&c);
		dataAddress[i] = c;
	}
}

/**
 * gets a string from the terminal while echoing characters.
 * @param dataAddress is the pointer to the buffer to receive the data.
 * @param dataSize is the length of the buffer.
 * @param echo whether to echo to the terminal.
 * @return the number of characters read.
 */
int AS1_getStringEx (char *dataAddress, int dataSize, bool echo)
{
	int i = 0;
	int readChars = 0;
	unsigned char c = 0;
	
	if (dataAddress == NULL || dataSize <= 0)
		return 0;
	
	do
	{
		do
  			readChars = AS1_rxBufferSize ();
  		while (readChars == 0);
	
		AS1_recvChar (&c);
		if (echo && c >= 32)
			AS1_sendChar (c);
			
		dataAddress[i] = c;
		i++;
	}
	while (c != '\r' && i < dataSize-1);

	if (echo)
		AS1_printStringEx ("\r\n");
	
	if (c == '\r')
		i--;

	dataAddress[i] = 0;	
	return (i > 0) ? i : 0;
}

/**
 * prints a DWord (32 bits) to the terminal.
 * @param data is the variable to display.
 */
void AS1_printDWordAsChars (dword data)
{
	dword dwrd;
	word wrd;
	word buf;
	char c[8];
	int i;

	dwrd = 0x0000FFFF & data;
	wrd = dwrd;
	for(i = 7; i >= 4; i--)
	{
		buf = 0x0000000F & wrd;
		c[i] = AS1_hexToChar (buf);
		wrd = __shr(wrd, 4);
	}
	
	dwrd = 0xFFFF0000 & data;
	wrd = dwrd >> 16;
	
	for(i = 3; i >= 0; i--)
	{
		buf = 0x0000000F & wrd;
		c[i] = AS1_hexToChar(buf);
		wrd = __shr(wrd, 4);
	}
	
	AS1_printString (c, 8); //sizeof(c));	
}

/**
 * prints a DWord (32 bits) as a decimal number.
 * @param data is the variable to display.
 */
void AS1_printDWordAsCharsDec (dword data)
{
	dword dwrd;
	dword sign;
	char c[10];
	int i;
		
	sign = 0x80000000 & data;
	if (sign)
	{
		data = -data;
		c[0] = '-';
	}
	else
		c[0] = '+';
	
	for (i = 0; i < 9; i++)
	{
		dwrd = data % 10;
		data /= 10;
		c[9-i] = dwrd + 48;
	}

	AS1_printString (c, 10);	
}

/**
 * prints a byte to the terminal (although byte is 16 bits this 
 * function prints only the less significant 8 bits).
 * @param data is the variable to display.
 */
void AS1_printByteAsChars (byte data)
{
	byte bty;
	byte buf;
	char c[8];
	int i;
	bty = data;
	
	for(i = 7; i >= 0; i--)
	{
		buf = 00000001 & bty;
		c[i] = AS1_hexToChar(buf);
		bty = __shr(bty, 1);
	}
	
	AS1_printString (c, 8);
}

