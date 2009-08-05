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
static word InpLen;                    /* Length of input buffer's content */
static byte *InpPtrR;         /* Pointer for reading from input buffer */
static byte *InpPtrW;         /* Pointer for writing to input buffer */
static byte InpBuffer[AS1_INP_BUF_SIZE]; /* Input buffer for SCI commmunication */
static word OutLen;                    /* Length of output bufer's content */
static byte *OutPtrR;         /* Pointer for reading from output buffer */
static byte *OutPtrW;         /* Pointer for writing to output buffer */
static byte OutBuffer[AS1_OUT_BUF_SIZE]; /* Output buffer for SCI commmunication */

/* 
	Local Prototypes
*/
void AS1_InterruptRx		(void);
void AS1_InterruptTx		(void);
void AS1_InterruptError		(void);

/**
 * enables the serial communication channel.
 */
static void HWEnDi(void)
{
  getReg(SCI0_SCISR);                  /* Reset interrupt request flags */
  setRegBits(SCI0_SCICR, (SCI0_SCICR_TE_MASK | SCI0_SCICR_RE_MASK | SCI0_SCICR_RIE_MASK | SCI0_SCICR_REIE_MASK)); /* Enable device */
}


/**
 * initializes the serial channel.
 */
void AS1_Init(void)
{
  SerFlag = 0;                         /* Reset flags */
  /* SCI0_SCICR: LOOP=0,SWAI=0,RSRC=0,M=0,WAKE=0,POL=0,PE=0,PT=0,TEIE=0,TIIE=0,RIE=0,REIE=0,TE=0,RE=0,RWU=0,SBK=0 */
  setReg(SCI0_SCICR, 0);               /* Set the SCI configuration */
  InpLen = 0;                          /* No char in the receive buffer */
  InpPtrW = InpPtrR = InpBuffer;       /* Set pointer on the first item in the receive buffer */
  OutLen = 0;                          /* No char in the transmit buffer */
  OutPtrW = OutPtrR = OutBuffer;       /* Set pointer on the first item in the transmit buffer */
  setReg(SCI0_SCIBR, 22);              /* Set prescaler bits */
  HWEnDi();                            /* Enable/disable device according to status flags */
}

/**
 * sends a single character.
 * @param chr a byte to send.
 * @return ERR_OK if all was ok or negative in case of error.
 */
byte AS1_SendChar(byte Chr)
{
  if (OutLen == AS1_OUT_BUF_SIZE)     /* Is number of chars in buffer is the same as a size of the transmit buffer */
    return ERR_TXFULL;                 /* If yes then error */
  //EnterCritical();                     /* Disable global interrupts */
  OutLen++;                            /* Increase number of bytes in the transmit buffer */
  *(OutPtrW++) = Chr;                  /* Store char to buffer */
  if (OutPtrW >= OutBuffer + AS1_OUT_BUF_SIZE) /* Is the pointer out of the transmit buffer */
    OutPtrW = OutBuffer;               /* Set pointer to first item in the transmit buffer */
  setRegBit(SCI0_SCICR, TEIE);         /* Enable transmit interrupt */
  //ExitCritical();                      /* Enable global interrupts */
  return ERR_OK;                       /* OK */
}

/**
 * gets a single character from the serial port.
 * @param chr is a pointer to the byte received.
 * @return ERR_OK if ok or a negative value if error.
 */
byte AS1_RecvChar(byte *Chr)
{
  register byte Result = ERR_OK;       /* Return error code */

  if (InpLen > 0) {                    /* Is number of received chars greater than 0? */
    //EnterCritical();                   /* Disable global interrupts */
    InpLen--;                          /* Decrease number of received chars */
    *Chr = *(InpPtrR++);               /* Received char */
    if (InpPtrR >= InpBuffer + AS1_INP_BUF_SIZE) /* Is the pointer out of the receive buffer? */
      InpPtrR = InpBuffer;             /* Set pointer to the first item into the receive buffer */
    Result = (byte)((SerFlag & (OVERRUN_ERR|COMMON_ERR|FULL_RX))?ERR_COMMON:ERR_OK);
    SerFlag &= ~(OVERRUN_ERR|COMMON_ERR|FULL_RX|CHAR_IN_RX); /* Clear all errors in the status variable */
    //ExitCritical();                    /* Enable global interrupts */
  }
  else {
    return ERR_RXEMPTY;                /* Receiver is empty */
  }
  return Result;                       /* Return error code */
}

/**
 * receives a block of a characters.
 * @param Ptr is the array where the received bytes are stored.
 * @param Size is size of the array.
 * @param Rcv is the number of received bytes
 * @return ERR_OK if all was ok or negative in case of error.
 */
byte AS1_RecvBlock(byte *Ptr,word Size,word *Rcv)
{
  register word count;                 /* Number of received chars */
  register byte result = ERR_OK;       /* Last error */

  for (count = 0; count < Size; count++) {
    result = AS1_RecvChar(Ptr++);
    if (result != ERR_OK) {            /* Receiving given number of chars */
      *Rcv = count;                    /* Return number of received chars */
      return result;                   /* Return last error */
    }
  }
  *Rcv = count;                        /* Return number of received chars */
  return result;                       /* OK */
}

/**
 * Sends a block of a characters.
 * @param Ptr is the array of byte to be send
 * @param Size is size of the array.
 * @param Rcv is the number of sent bytes
 * @return ERR_OK if all was ok or negative in case of error.
 */
byte AS1_SendBlock(byte *Ptr,word Size,word *Snd)
{
  register word count;                 /* Number of sent chars */
  register byte result = ERR_OK;       /* Last error */

  for (count = 0; count < Size; count++) {
    result = AS1_SendChar(*Ptr++);
    if (result != ERR_OK) {            /* Sending given number of chars */
      *Snd = count;                    /* Return number of sent chars */
      return result;                   /* Return last error */
    }
  }
  *Snd = count;                        /* Return number of sended chars */
  return result;                       /* Return error code */
}

/**
 * Clears the Receiver buffer
 */
byte AS1_ClearRxBuf(void)
{
  //EnterCritical();                     /* Disable global interrupts */
  InpLen = 0;                          /* Set number of chars in the transmit buffer to 0 */
  InpPtrR = InpPtrW = InpBuffer;       /* Set pointers on the first item in the transmit buffer */
  //ExitCritical();                      /* Enable global interrupts */
  return ERR_OK;                       /* OK */
}

/**
 * Clears the Transmitter buffer
 */
byte AS1_ClearTxBuf(void)
{
  //EnterCritical();                     /* Disable global interrupts */
  OutLen = 0;                          /* Set number of chars in the receive buffer to 0 */
  OutPtrR = OutPtrW = OutBuffer;       /* Set pointers on the first item in the receive buffer */
  //ExitCritical();                      /* Enable global interrupts */
  return ERR_OK;                       /* OK */
}

/**
 * Return number of chars in receive buffer
 */
word AS1_GetCharsInRxBuf(void)
{
  return InpLen;                       
}

/**
 * Return the number of chars in the transmitter buffer
 */
word AS1_GetCharsInTxBuf(void)
{
  return OutLen;              
}

/**
 * recv interrupt handler.
 */
#define ON_ERROR    1
#define ON_FULL_RX  2
#define ON_RX_CHAR  4

#pragma interrupt saveall
void AS1_InterruptRx(void)
{
  register byte Data;         /* Temporary variable for data */
  register byte OnFlags = 0;           /* Temporary variable for flags */

  getReg(SCI0_SCISR);                  /* Reset interrupt request flags */
  Data = (byte)getReg(SCI0_SCIDR); /* Read data from the receiver */
  if (InpLen < AS1_INP_BUF_SIZE) {    /* Is number of bytes in the receive buffer lower than size of buffer? */
    InpLen++;                          /* Increse number of chars in the receive buffer */
    *(InpPtrW++) = Data;               /* Save received char to the receive buffer */
    if (InpPtrW >= InpBuffer + AS1_INP_BUF_SIZE) /* Is the pointer out of the receive buffer? */
      InpPtrW = InpBuffer;             /* Set pointer on the first item into the receive buffer */
    OnFlags |= ON_RX_CHAR;             /* Set flag "OnRXChar" */
    if (InpLen == AS1_INP_BUF_SIZE)   /* Is number of bytes in the receive buffer equal as a size of buffer? */
      OnFlags |= ON_FULL_RX;           /* If yes then set flag "OnFullRxBuff" */
  }
  else {
    SerFlag |= FULL_RX;                /* If yes then set flag buffer overflow */
    OnFlags |= ON_ERROR;               /* Set flag "OnError" */
  }
  if (OnFlags & ON_ERROR) {            /* Was error flag detect? */
   // AS1_OnError();                    /* If yes then invoke user event */
  }
  else {
    if (OnFlags & ON_RX_CHAR) {        /* Is OnRxChar flag set? */
     // AS1_OnRxChar();                 /* If yes then invoke user event */
    }
    if (OnFlags & ON_FULL_RX) {        /* Is OnFullRxBuf flag set? */
     // AS1_OnFullRxBuf();              /* If yes then invoke user event */
    }
  }
}
/**
 * transmission interrupt handler.
 */
#define ON_FREE_TX  1
#define ON_TX_CHAR  2

#pragma interrupt 
void AS1_InterruptTx(void)
{
  register byte OnFlags = 0;           /* Temporary variable for flags */

  if (SerFlag & RUNINT_FROM_TX)        /* Is flag "running int from TX" set? */
    OnFlags |= ON_TX_CHAR;             /* Set flag "OnTxChar" */
  SerFlag &= ~RUNINT_FROM_TX;          /* Reset flag "running int from TX" */
  if (OutLen) {                        /* Is number of bytes in the transmit buffer greater then 0? */
    OutLen--;                          /* Decrease number of chars in the transmit buffer */
    SerFlag |= RUNINT_FROM_TX;         /* Set flag "running int from TX"? */
    getReg(SCI0_SCISR);                /* Reset interrupt request flags */
    setReg(SCI0_SCIDR,(byte)* OutPtrR++); /* Store char to transmitter register */
    if (OutPtrR >= OutBuffer + AS1_OUT_BUF_SIZE) /* Is the pointer out of the transmit buffer? */
      OutPtrR = OutBuffer;             /* Set pointer on the first item into the transmit buffer */
  }
  else {
    OnFlags |= ON_FREE_TX;             /* Set flag "OnFreeTxBuf" */
    clrRegBit(SCI0_SCICR, TEIE);       /* Disable transmit interrupt */
  }
  if (OnFlags & ON_TX_CHAR) {          /* Is flag "OnTxChar" set? */
  //  AS1_OnTxChar();                   /* If yes then invoke user event */
  }
  if (OnFlags & ON_FREE_TX) {          /* Is flag "OnFreeTxBuf" set? */
   // AS1_OnFreeTxBuf();                /* If yes then invoke user event */
  }
}

/**
 * receive error interrupt handler.
 */
#pragma interrupt 
void AS1_InterruptError(void)
{
  register word StatReg = getReg(SCI0_SCISR); /* Read status register */

  setReg(SCI0_SCISR, 0);               /* Reset error request flags */
  if(StatReg & (SCI0_SCISR_OR_MASK|SCI0_SCISR_NF_MASK|SCI0_SCISR_FE_MASK|SCI0_SCISR_PF_MASK)) /* Is an error detected? */
    SerFlag |= COMMON_ERR;             /* If yes then set an internal flag */
  if (SerFlag & COMMON_ERR) {          /* Was any error set? */
 //   ASC1_OnError();                    /* If yes then invoke user event */
  }
}


/**
 * prints a string to terminal.
 * @param dataAddress is a pointer to the string.
 */
void AS1_printStringEx (char *dataAddress)
{
	int dataSize = 0;
		
	while (dataAddress[dataSize] != '\0')
	{
		dataSize++;
  	}
	
	AS1_printString (dataAddress,  dataSize);
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
 * sends a string to the terminal
 * @param dataAddress is the pointer to the string
 * @param dataSize is the size of the string (in bytes)
 */
void AS1_printString (char *dataAddress, int dataSize)
{  
	word temp =0;
    AS1_SendBlock((byte *) dataAddress,dataSize, &temp);	

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
  			readChars = AS1_GetCharsInRxBuf ();
  		while (readChars == 0);
  		
		AS1_RecvChar (&c);
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
  			readChars = AS1_GetCharsInRxBuf ();
  		while (readChars == 0);
	
		AS1_RecvChar (&c);
		if (echo && c >= 32) AS1_SendChar(c);
			
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
	
	AS1_printString (c, 8); 	
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

