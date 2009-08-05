/*
 * asc.h
 *
 */
 
#ifndef __asch__
#define __asch__

#include "dsp56f807.h"

void AS1_init(void);
byte AS1_sendChar(byte chr);
byte AS1_sendCharSafe(byte chr);
byte AS1_recvChar(byte *chr);
word AS1_txBufferSize(void);
word AS1_rxBufferSize(void);
void AS1_printString (char *dataAddress, int dataSize);
void AS1_printStringEx (char *dataAddress);
void AS1_printDWordAsChars (dword data);
void AS1_printDWordAsCharsDec (dword data);
void AS1_printWord16AsChars (Word16 data);
void AS1_printUWord16AsChars (UWord16 data);
void AS1_printByteAsChars (byte data);
char AS1_hexToChar (byte value);
int  AS1_atoi (char *strAddr, int strSize);
long AS1_atol (char *strAddr, int strSize);
int  AS1_strlen (char *strAddr, int strSize);
void AS1_getString (char *dataAddress, int dataSize);
int  AS1_getStringEx (char *dataAddress, int dataSize, bool echo);

/* int handlers */
void AS1_interruptTx(void);
void AS1_interruptRx(void);
void AS1_interruptError(void);


#endif