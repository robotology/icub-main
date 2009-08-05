/*
 * asc.h
 *
 */
 
#ifndef __asch__
#define __asch__

#include "dsp56f807.h"

/**
 * The size of the input buffer in bytes
 */
#define AS1_INP_BUF_SIZE  64           /* Length of the RX buffer */

/**
 * The size of the output buffer in bytes
 */
#define AS1_OUT_BUF_SIZE  512         /* Length of the TX buffer */

/* Function prototypes */

/**
 * initializes the serial channel.
 */
void AS1_Init				(void);

/**
 * gets a single character from the serial port.
 * @param chr is a pointer to the byte received.
 * @return ERR_OK if ok or a negative value if error.
 */
byte AS1_RecvChar			(byte *Chr);

/**
 * sends a single character.
 * @param chr a byte to send.
 * @return ERR_OK if all was ok or negative in case of error.
 */
byte AS1_SendChar			(byte);

/**
 * receives a block of a characters.
 * @param Ptr is the array where the received bytes are stored.
 * @param Size is size of the array.
 * @param Rcv is the number of received bytes
 * @return ERR_OK if all was ok or negative in case of error.
 */
byte AS1_RecvBlock			(byte *Ptr,word Size,word *Rcv);

/**
 * Sends a block of a characters.
 * @param Ptr is the array of byte to be send
 * @param Size is size of the array.
 * @param Rcv is the number of sent bytes
 * @return ERR_OK if all was ok or negative in case of error.
 */
byte AS1_SendBlock			(byte *Ptr,word Size,word *Snd);

/**
 * Clears the Receiver buffer
 */
byte AS1_ClearRxBuf			(void);

/**
 * Clears the Transmitter buffer
 */
byte AS1_ClearTxBuf			(void);

/**
 * Return the number of chars in the transmitter buffer
 */
word AS1_GetCharsInTxBuf	(void);

/**
 * Return number of chars in receive buffer
 */
word AS1_GetCharsInRxBuf	(void);

/**
 * sends a string to the terminal
 * @param dataAddress is the pointer to the string
 * @param dataSize is the size of the string (in bytes)
 */
void AS1_printString 		(char *dataAddress, int dataSize);

/**
 * prints a string to terminal.
 * @param dataAddress is a pointer to the string.
 */
void AS1_printStringEx 		(char *dataAddress);

/**
 * prints a DWord (32 bits) to the terminal.
 * @param data is the variable to display.
 */
void AS1_printDWordAsChars 	(dword data);

/**
 * prints a DWord (32 bits) as a decimal number.
 * @param data is the variable to display.
 */
void AS1_printDWordAsCharsDec 	(dword data);

/**
 * prints a Word16 to the terminal.
 * @param data is the variable to display (hex).
 */
void AS1_printWord16AsChars 	(Word16 data);

/**
 * prints a UWord16 to the terminal.
 * @param data is the variable to display (hex).
 */
void AS1_printUWord16AsChars 	(UWord16 data);

/**
 * prints a byte to the terminal (although byte is 16 bits this 
 * function prints only the less significant 8 bits).
 * @param data is the variable to display.
 */
void AS1_printByteAsChars 		(byte data);

/**
 * converts a hex value to char for display.
 * @param byte the hex to convert (lower 4 bits only).
 * @return the character (0-F) or * if outside range.
 */
char AS1_hexToChar 				(byte value);

/**
 * converts from ascii to integer (16 bits).
 * @param strAddr is the pointer to the string to be converted.
 * @param strSize is the size of the string.
 * @return the result of the conversion.
 */
int  AS1_atoi 					(char *strAddr, int strSize);

/**
 * converts from ascii to long (32 bits).
 * @param strAddr is the string to be converted.
 * @param strSize is the length of the string.
 * @return the result of the conversion.
 */
long AS1_atol 					(char *strAddr, int strSize);

/**
 * computes the length of a string not necessarily zero terminated.
 * @param strAddr is the string to be converted.
 * @param strSize is the maximum length of the string.
 * @return the length of the string.
 */
int  AS1_strlen 				(char *strAddr, int strSize);

/**
 * receives a string from the terminal.
 * @param dataAddress is the pointer to the buffer to receive the data.
 * @param dataSize is the maximum number of bytes read (well, they're actually 16 bit words).
 */
void AS1_getString 				(char *dataAddress, int dataSize);

/**
 * gets a string from the terminal while echoing characters.
 * @param dataAddress is the pointer to the buffer to receive the data.
 * @param dataSize is the length of the buffer.
 * @param echo whether to echo to the terminal.
 * @return the number of characters read.
 */
int  AS1_getStringEx 			(char *dataAddress, int dataSize, bool echo);

#endif