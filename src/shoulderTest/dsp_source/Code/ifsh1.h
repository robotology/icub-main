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

/* FLASH memory size and location in 16-bit words */
#define DATA_FLASH_START 8192
#define DATA_FLASH_END   16383
#define DATA_FLASH_SECTOR_SIZE 256u

/* Interrupt source/enable/pending register mask for Command complete interrupt flag(s) */
#define DFIU_CC_INT_MASK 256

#ifndef CommonFlashData
#define CommonFlashData

/* Flash user mode commands */
#define PROGRAM      0x20
#define PAGE_ERASE   0x40
#define MASS_ERASE   0x41

#endif

/* Old symbols */
#define DataFlashStart DATA_FLASH_START
#define DataFlashEnd   DATA_FLASH_END


byte IFsh1_setByteFlash(dword Addr,byte Data);
byte IFsh1_getByteFlash(dword Addr,byte *Data);
byte IFsh1_setWordFlash(dword Addr,word Data);
byte IFsh1_getWordFlash(dword Addr,word *Data);
byte IFsh1_setLongFlash(dword Addr,dword Data);
byte IFsh1_getLongFlash(dword Addr,dword *Data);
void IFsh1_init(void);
void IFsh1_interrupt(void);
byte IFsh1_eraseFlash (dword Addr);
byte IFsh1_writeWords2 (dword Addr, word* PData, dword Size);

#endif /* ifndef __ifsh1h__ */
