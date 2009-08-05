#ifndef __eeprom_interfaceh__
#define __eeprom_interfaceh__

#include "dsp56f807.h"

/***************************************************************************/
/**
 * This method inits the eeprom interface
 ***************************************************************************/
void EEPROM_Init();

/***************************************************************************/
/**
 * This method enables the writing on the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_WREN();

/***************************************************************************/
/**
 * This method disables the writing on the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_WRDI();

/***************************************************************************/
/**
 * This method reads the status register of the eeprom
 * @param SR is the byte that represents the status register of the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_RDSR(byte *SR);

/***************************************************************************/
/**
 * This method writes the status register of the eeprom
 * @param WSR is the byte to write on the eeprom
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_WRSR(byte WSR);

/***************************************************************************/
/**
 * This method reads an array of bytes from the eeprom
 * @param DataRead is the pointer to the array of data to be filled
 * @param Address is the eeprom address to be read
 * @param NData is the number of bytes to be read 
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_READ(byte * DataRead,UInt16 Address, UInt16 NData);

/***************************************************************************/
/**
 * This method writes an array of bytes to the eeprom
 * @param DataWrite is the pointer to the array of data to be written
 * @param Address is the eeprom address to be witten
 * @param NData is the length in bytes of the array to be written  
 * @return ERR_OK if the operation has been complete successfully
 ***************************************************************************/
byte EEPROM_WRITE(byte * DataWrite, UInt16 Address, UInt16 NData);


#endif
