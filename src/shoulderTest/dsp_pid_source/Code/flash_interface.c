#include "dsp56f807.h"
#include "flash_interface.h"
#include "asc.h"
#include "ifsh1.h"
#include "pid.h"

#define ADP(x,amount) x+=amount

/***************************************************************************/
/*
 * Global variables
 * flash_channels is the number of channels present on the board (can be 2 or 4)
 ***************************************************************************/
byte flash_channels   = 0;
Int16 _flash_addr 	  = 0;

/***************************************************************************/
/**
 * writes data on the flash memory at the specified address
 * @param addr it's the address of the flash memory
 * @return ERR_OK always
 ***************************************************************************/
byte writeToFlash (word addr, byte board_ID)
{
	dword ptr = (dword)addr;
	byte i, err;
	word tmp;
	bool gerr = false;

	IFsh1_eraseFlash (addr);

	tmp = BYTE_W(board_ID, 0);
	err = IFsh1_setWordFlash(ptr, tmp);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);
	err = IFsh1_setWordFlash(ptr, _version);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);
	
	for (i = 0; i < flash_channels; i++)
	{
		err = IFsh1_setWordFlash(ptr, _kp[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _kd[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _ki[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _ko[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _kr[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _integral_limit[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, _pid_limit[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		
		err = IFsh1_setLongFlash(ptr, _min_position[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,4);
		err = IFsh1_setLongFlash(ptr, _max_position[i]);
		gerr |= (err != ERR_OK);
		ADP(ptr,4);
	}

	if (gerr)
		AS1_printStringEx ("Error while writing to flash memory, pls try again\r\n");
	
	return ERR_OK;
}

/***************************************************************************/
/**
 * Inits the Flash interface
 * @param b_type is the board type, can be:
 * BOARD_TYPE_2DC  (2 channels)
 * BOARD_TYPE_4DC  (4 channels)
 * BOARD_TYPE_2BLL (2 channels)
 ***************************************************************************/
void flash_interface_init (byte number_of_channels)
{
	flash_channels=number_of_channels;
	IFsh1_init ();
}

/***************************************************************************/
/**
 * Reads data from the flash memory at the specified address
 * @param addr it's the address of the flash memory
 * @return ERR_OK always
 ***************************************************************************/
byte readFromFlash (word addr, byte* board_ID)
{
	dword ptr = (dword)addr;
	word tmp;
	int i;

	IFsh1_getWordFlash(ptr, &tmp);
	*board_ID = BYTE_H(tmp) & 0x0f;
	ADP(ptr,2);
	//IFsh1_getWordFlash(ptr, (unsigned int *)&_version);
	ADP(ptr,2);

	for (i = 0; i < flash_channels; i++)
	{
		IFsh1_getWordFlash(ptr, (word *)(_kp+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_kd+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_ki+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_ko+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_kr+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_integral_limit+i));
		ADP(ptr,2);
		IFsh1_getWordFlash(ptr, (word *)(_pid_limit+i));
		ADP(ptr,2);
		
		IFsh1_getLongFlash(ptr, (dword *)(_min_position+i));
		ADP(ptr,4);
		IFsh1_getLongFlash(ptr, (dword *)(_max_position+i));
		ADP(ptr,4);
	}
	
	return ERR_OK;
}

/******************************************************************
/* defined by the linker */
extern _data_ROM2_addr;

/***************************************************************************/
/**
 * Gets the address of the flash memory
 * @return the address of the flash memory
 ***************************************************************************/
asm int get_flash_addr (void)
{
	move #_data_ROM2_addr, y0
	rts
}