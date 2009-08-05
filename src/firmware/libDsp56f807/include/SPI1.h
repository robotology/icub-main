#ifndef __SPI1h__
#define __SPI1h__

#include "dsp56f807.h"

/***************************************************************************/
/**
 * This method inits the SPI interface
 ***************************************************************************/
void SPI1_Init(void);

/***************************************************************************/
/**
 * This method reads a byte from the SPI bus
 * @param Chr is the read byte
 * @return ERR_OK always
 ***************************************************************************/
byte SPI1_RecvChar(byte *Chr);

/***************************************************************************/
/**
 * This method sends a byte to the SPI bus
 * @param Chr is the byte to be sent. 
 * @return ERR_OK always
 ***************************************************************************/
byte SPI1_SendChar(byte Chr);

#endif 

