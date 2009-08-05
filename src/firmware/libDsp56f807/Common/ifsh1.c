/*
 * ifsh1.c
 *	implementation of the flash memory access.
 */

#include "ifsh1.h"


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

void IFsh1_interrupt(void);
/* Registers for usage in asm macros/functions */
#define DFIU_CNTL_ASM X:4960

volatile static byte Err = ERR_OK;              /* Error state of current process */

/**
 * checks whether the requested address is within range.
 * @param addr1 start address.
 * @param addr2 end address.
 * @return true if within range, false otherwise.
 */
static byte outOfRange (dword addr1, dword addr2)
{
	return ((addr1>addr2)||(addr1<DATA_FLASH_START)||(addr2>DATA_FLASH_END));
}

#define SectorSize(addr) (DATA_FLASH_SECTOR_SIZE)
#define ClearFlags() setReg(DFIU_IS,0)
#define AccessError() (getReg(DFIU_IS) & 0x0C01)
#define readflash(address) (*(word *)(address))
#define writeflash(address, data) (*(word *)(address)=data)

/**
 *
 */
static byte procflash (dword Address, word Data, word Command)
{
	register word address = (word)Address;

	if ((getReg (DFIU_CNTL) & 0x8000))      /* Is DFIU busy ? */
		return ERR_BUSY;                   /* If yes then error */
	
	ClearFlags ();                        /* Clear all flags */
	setReg (DFIU_CNTL, 0);                 /* Reset DFIU_CNTL register */
	
	EnterCritical ();                     /* Disable all low level interrupts */
	if (Command == PROGRAM) 
	{
		setReg (DFIU_EE, 0);                /* Clear Erase enable register */
		setReg (DFIU_PE, (address >> 5) + 0x4000); /* Write row number and enable intelligent programming */
	}
	else 
	{
		setReg (DFIU_PE, 0);                /* Clear Program enable register */
		if (Command == MASS_ERASE) 
		{       /* Is the mass erase operation requested? */
			setReg (DFIU_EE, 0x4000);          /* Write page number and enable intelligent erase */
			setRegBit (DFIU_CNTL, MAS1);      /* If yes, then enable mass erase */
		}
		else
			setReg (DFIU_EE, (address >> 8) + 0x4000); /* Write page number and enable intelligent erase */
	}
	
	writeflash (address, Data);           /* Write data to program memory - launch the operation */
	ExitCritical ();                      /* Enable all low level interrupts */

	if (AccessError ())                   /* Is an access error detected ? */
		return ERR_NOTAVAIL;               /* If yes then return the error */
	
	return Err;
}


/**
 *
 */
byte IFsh1_eraseFlash (dword Addr)
{
	Err = ERR_OK;

	if (outOfRange (Addr, Addr+SectorSize(0)-1))
		return ERR_RANGE;
		
	if ((Addr % SectorSize(0)) != 0)
		return ERR_RANGE;
		
	Err = procflash(Addr, 0, PAGE_ERASE);
	if (Err != ERR_OK)
		return Err;

	while (getReg(DFIU_CNTL) & 0x8000) {}	

	setReg(DFIU_PE,0);
	setRegBits (DFIU_IE, 0x0fff);
	
	return Err;
}


/**
 *
 */
byte IFsh1_writeWords2 (dword Addr, word* PData, dword Size)
{
	register word i;
	register word j, cycles, AddrInSec;
	Err = ERR_OK;

	if (outOfRange (Addr, Addr+Size-1))
		return ERR_RANGE;
		
	for (i = 0; i < Size; i++) 
	{
		while (getReg(DFIU_CNTL) & 0x8000) {}
		
		Err = procflash ((Addr+i), PData[i], PROGRAM);
		if (Err != ERR_OK) 
		{
			return Err;
		}
	}
	
	while (getReg(DFIU_CNTL) & 0x8000) {}
	
	setReg(DFIU_PE,0);
	
	for (i = 0; i < Size; i++)
	{
		if (readflash(Addr+i) != PData[i]) 
		{
			return ERR_VALUE;
		}
	}
		
	setRegBits (DFIU_IE, 0x0fff);
	
	return Err;
}


/**
 *
 */
static byte writeWords (dword Addr, word* PData, dword Size)
{
	register word i;
	register word j, cycles, AddrInSec;
	Err = ERR_OK;

	if (outOfRange (Addr, Addr+Size-1))    /* Is the address out of range? */
		return(ERR_RANGE);                 /* If yes then exit */
		
	for (i = 0; i < Size; i++) 
	{             /* For all given data */
		AddrInSec = (word)((Addr+i)&((dword)SectorSize(Addr+i)-1)); /* Calculate relative address in a sector */
		
		if ((AddrInSec==0) || (i==0)) 
		{      /* Is the actual address the first or is it a border of a sector? */
			cycles = (word)(SectorSize(Addr+i) - AddrInSec - /* How many address places have to be testet in the actual sector? */
			(((Size - i < (SectorSize(Addr+i)) - AddrInSec)) ? (SectorSize(Addr + 1) - AddrInSec - Size + i) : 0));
		
			while (getReg(DFIU_CNTL) & 0x8000) {} /* Wait to command complete */
		
			for (j = 0; j < cycles; j++) 
			{        /* For all memory locations which have to be tested in the actual sector */
				if (~(readflash(Addr+i+j))&(PData[i+j])) 
				{ /* Is the sector erasure necessary? */
					Err = procflash((Addr+i),0,PAGE_ERASE); /* Erase the sector */
					if (Err != ERR_OK) 
					{         /* If an error occured then exit */
						return Err;
					}
					break;                       /* The sector is backuped and erased. Now end test of the sector */
				}
			}
		}
	
		while (getReg(DFIU_CNTL) & 0x8000) {} /* Wait to command complete */
		
		Err = procflash ((Addr+i), PData[i], PROGRAM); /* Write new data to Flash */
		
		if (Err != ERR_OK) 
		{                 /* If an error occured then exit */
			return Err;
		}
	}
	
	while (getReg(DFIU_CNTL) & 0x8000) {} /* Wait to command complete */
	
	setReg(DFIU_PE,0);                   /* Clear program enable register - stop programming */
	
	for (i = 0; i < Size; i++)               /* Check all given data were written good */
	{
		if (readflash(Addr+i) != PData[i]) 
		{
			return ERR_VALUE;                /* If an error occured exit */
		}
	}
		
	setRegBits (DFIU_IE, 0x0fff);
	//setRegMask (DFIU_IE,0,DFIU_CC_INT_MASK); /* Enable interrupt */
	
	return Err;
}

/**
 *
 */
byte IFsh1_setByteFlash(dword Addr, byte Data)
{
	word Data16;
	byte rot;
	rot = (byte)((Addr%2)*8);
	Data16 = (readflash(Addr/2)&(0xff00>>rot))+((Data&0xff)<<rot);
	
	return IFsh1_writeWords2(Addr/2, &Data16, sizeof(Data16)/sizeof(word)); /* Write data to FLASH - use block method */
}

/**
 *
 */
byte IFsh1_getByteFlash(dword Addr,byte *Data)
{
	word Data16;
	if (outOfRange(Addr/2,Addr/2))       /* Check range of address */
		return ERR_RANGE;
	
	Data16 = readflash((Addr/2));        /* Read word from FLASH */
	*Data = Addr%2?Data16>>8:Data16&0xff;
	
	return ERR_OK;
}

/**
 *
 */
byte IFsh1_setWordFlash(dword Addr,word Data)
{
	return IFsh1_writeWords2(Addr, &Data, sizeof(Data)/sizeof(word)); /* Write data to FLASH - use block method */
}

/**
 *
 */
byte IFsh1_getWordFlash(dword Addr,word *Data)
{
	if (outOfRange(Addr,Addr))           /* Check range of address */
		return ERR_RANGE;
		
	*Data = readflash(Addr);             /* read data from FLASH */
	return ERR_OK;
}

/**
 *
 */
byte IFsh1_setLongFlash(dword Addr,dword Data)
{
	return IFsh1_writeWords2(Addr, (word *)&Data, sizeof(Data)/sizeof(word)); /* Write data to FLASH - use block method */
}

/**
 *
 */
byte IFsh1_getLongFlash(dword Addr,dword *Data)
{
	if (outOfRange(Addr+1,Addr+1))       /* Check range of address */
		return ERR_RANGE;
	
	*Data = readflash(Addr) + (((dword)(readflash(Addr+1)))<<16); /* Read data from FLASH */
	return ERR_OK;
}

/**
 * initializes the data flash interface.
 */
void IFsh1_init(void)
{
	/* DFIU_PE: DPE=0,IPE=0,??=0,??=0,??=0,??=0,ROW=0 */
	setReg(DFIU_PE, 0);                   /* Reset Program Enable register */

	/* DFIU_EE: DEE=0,IEE=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,PAGE=0 */
	setReg(DFIU_EE, 0);                   /* Reset Erase Enable register */

	/* DFIU_CKDIVISOR: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,N=12 */
	setReg(DFIU_CKDIVISOR, 12);           /* Set Flash Clock Divisor register */

	/* DFIU_TMEL: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,TMEL=255 */
	setReg(DFIU_TMEL, 255);               /* Set Flash Tme Limit register */

	/* DFIU_TERASEL: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,??=0,TERASEL=127 */
	setReg(DFIU_TERASEL, 127);            /* Set Flash Terase Limit register */

	/* DFIU_TNVSL: ??=0,??=0,??=0,??=0,??=0,TNVSL=256 */
	setReg(DFIU_TNVSL, 256);              /* Set Flash Tnvs Limit register */

	/* DFIU_TPGSL: ??=0,??=0,??=0,??=0,TPGSL=512 */
	setReg(DFIU_TPGSL, 512);              /* Set Flash Tpgs Limit register */

	/* DFIU_TPROGL: ??=0,??=0,TPROGL=1024 */
	setReg(DFIU_TPROGL, 1024);            /* Set Flash Tprog Limit register */

	/* DFIU_TNVHL: ??=0,??=0,??=0,??=0,??=0,TNVHL=256 */
	setReg(DFIU_TNVHL, 256);              /* Set Flash Tnvh Limit register */

	/* DFIU_TNVH1L: ??=0,TNVH1L=4096 */
	setReg(DFIU_TNVHL1, 4096);            /* Set Flash Tnvh1 Limit register */

	/* DFIU_TRCVL: ??=0,??=0,??=0,??=0,??=0,??=0,??=0,TRCVL=64 */
	setReg(DFIU_TRCVL, 64);               /* Set Flash Trcv Limit register */

	/* DFIU_IE: ??=0,??=0,??=0,??=0,IE=0 */
	clrRegBits (DFIU_IE, 0x0fff);

	Err = ERR_OK;
}

/**
 *
 */
#pragma interrupt 
void IFsh1_interrupt(void)
{
	if (getReg(DFIU_IP)&DFIU_CC_INT_MASK) 
	{
		clrRegBits (DFIU_IS, 0x0fff);
		clrRegBits (DFIU_IE, 0x0fff);
		
	//	setRegMask(DFIU_IS, DFIU_CC_INT_MASK, 0); /* Clear command complete interrupt flag(s) */
	//	setRegMask(DFIU_IE, DFIU_CC_INT_MASK, 0); /* Disable command complete interrupt  */
	}
}
