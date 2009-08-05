/*
 * ifsh1.h
 * flash memory access interface.
 *
 */

#ifndef __ifsh1h__
#define __ifsh1h__

/**
 * \file ifsh1.h 
 * ifsh1.h contains the interface definition to access
 * the dsp flash memory.
 */

#include "dsp56f807.h"

/**
 * writes a byte on the flash
 * @param Addr is the flash address 
 * @param Data is the byte to be written
 * @return ERR_OK if successful.
 */
byte IFsh1_setByteFlash(dword Addr,byte Data);

/**
 * reads a byte from the flash
 * @param Addr is the flash address 
 * @param Data is a pointer filled with the byte read from the flash
 * @return ERR_OK if successful.
 */
byte IFsh1_getByteFlash(dword Addr,byte *Data);

/**
 * writes a word on the flash
 * @param Addr is the flash address 
 * @param Data is the word to be written
 * @return ERR_OK if successful.
 */
byte IFsh1_setWordFlash(dword Addr,word Data);

/**
 * reads a word from the flash
 * @param Addr is the flash address 
 * @param Data is a pointer filled with the word read from the flash
 * @return ERR_OK if successful.
 */
byte IFsh1_getWordFlash(dword Addr,word *Data);

/**
 * writes a double word on the flash
 * @param Addr is the flash address 
 * @param Data is the double word to be written
 * @return ERR_OK if successful.
 */
byte IFsh1_setLongFlash(dword Addr,dword Data);

/**
 * reads a double word from the flash
 * @param Addr is the flash address 
 * @param Data is a pointer filled with the double word read from the flash
 * @return ERR_OK if successful.
 */
byte IFsh1_getLongFlash(dword Addr,dword *Data);

/**
 * initializes the data flash interface.
 */
void IFsh1_init(void);

byte IFsh1_eraseFlash (dword Addr);
byte IFsh1_writeWords2 (dword Addr, word* PData, dword Size);


#endif /* ifndef __ifsh1h__ */
