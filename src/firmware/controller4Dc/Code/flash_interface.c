#include "dsp56f807.h"
#include "flash_interface.h"
#include "asc.h"
#include "ifsh1.h"
#include "pid.h"

#define ADP(x,amount) x+=amount

extern  byte	_board_ID;
extern  char    _additional_info[32];
extern Int16   _flash_version;
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
byte writeToFlash (word addr)
{
	dword ptr = (dword)addr;
	byte i, err;
	word tmp = 0;
	bool gerr = false;

	IFsh1_eraseFlash (addr);
	
	tmp =  CURRENT_BOARD_TYPE;
	tmp <<= 8;
	tmp |= _board_ID;
	err = IFsh1_setWordFlash(ptr,  tmp);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);	
		
	err = IFsh1_setWordFlash(ptr, _version);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);

    set_additional_info ();
    ADP(ptr,32);
    	
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
 * reset flash memory to its initial values
 * @param addr it's the address of the flash memory
 * @return ERR_OK always
 ***************************************************************************/
byte resetFlash (word addr)
{
	dword ptr = (dword)addr;
	byte i, err;
	word tmp = 0;
	bool gerr = false;

	IFsh1_eraseFlash (addr);
	
	_board_ID = DEFAULT_BOARD_ID;
	tmp =  CURRENT_BOARD_TYPE;
	tmp <<= 8;
	tmp |= _board_ID;
	err = IFsh1_setWordFlash(ptr,  tmp);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);	
		
	err = IFsh1_setWordFlash(ptr, _version);
	gerr |= (err != ERR_OK);
	ADP(ptr,2);

	for (i = 0; i < 32; i++)
	_additional_info [i] = 0;	
    set_additional_info ();
    ADP(ptr,32);
    	
	for (i = 0; i < flash_channels; i++)
	{
		err = IFsh1_setWordFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		err = IFsh1_setWordFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,2);
		
		err = IFsh1_setLongFlash(ptr, 0);
		gerr |= (err != ERR_OK);
		ADP(ptr,4);
		err = IFsh1_setLongFlash(ptr, 0);
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
 * @param b_type 
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
byte readFromFlash (word addr)
{
	dword ptr = (dword)addr;
	word tmp = 0;
	int i;

	//check if flash memory is empty and needs reinit
	IFsh1_getWordFlash(ptr, (&tmp));
	if (tmp == 0xFFFF)
		resetFlash (addr);
		
	//get board_ID
	IFsh1_getWordFlash(ptr, (&tmp)); 
	_board_ID = tmp & 0xFF;
	
	ADP(ptr,2);
	
	IFsh1_getWordFlash(ptr, (unsigned int *)&_flash_version);
	ADP(ptr,2);

    //Free space for additional info
    get_additional_info ();
    ADP(ptr,32);
    
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

/***************************************************************************/
/**
 * Stores a 32-byte string in the flash memory
 * @param info is the info to be stored in the flash memory
 ***************************************************************************/
void set_additional_info ()
{
	int i=0;
	int j=0;
	byte err = 0;
	byte len = 0;
	word temp;

	for (i = 0; i < 32; i++)
	{
	  if (_additional_info[i]==0)
	  	{
	  	 	for (j=i; j <32; j++) _additional_info[j] = 0;
	  	 	break;
	  	}
	}
	
	for (i = 0; i < 16; i++)
	{
		temp = _additional_info[i*2];
		temp <<= 8;
		temp |= _additional_info[i*2+1];
		err |= IFsh1_setWordFlash(_flash_addr+4+i*2, temp);
	}

	if (err)
	AS1_printStringEx ("Error while writing to flash memory, pls try again\r\n");
}

/***************************************************************************/
/**
 * Reads a 32-byte string from the flash memory
 * @param info is the info read from the flash memoryy
 ***************************************************************************/
void get_additional_info ()
{
	int i=0;
	word temp = 0;
	byte err = 0;
	
	for (i = 0; i < 16; i++)
	{
		err |= IFsh1_getWordFlash(_flash_addr+4+i*2, &temp);
		_additional_info[i*2] =   (temp >> 8) & 0xFF;
		_additional_info[i*2+1] = temp & 0xFF;
	}

	if (err)
	AS1_printStringEx ("Error reading from flash memory, pls try again\r\n");
}

#ifdef DEBUG_SERIAL			
/***************************************************************************/
/**
 * Dump the content of the flash memory (first 100 bytes)
 * 
 ***************************************************************************/
void dump_flash_memory (void)
{
	int i=0;
	word temp = 0;
	byte tmp1 = 0;
	byte tmp2 = 0;
	byte err = 0;
	AS1_printStringEx ( "Flash memory dump\r\n" );
	AS1_printStringEx ( "Address | value(hex) | value(char)\r\n" );
	for (i = 0; i< 50; i++)
	{
		err |= IFsh1_getWordFlash(_flash_addr+i*2, &temp);
		tmp1 = (temp >> 8) & 0xFF;
		tmp2 = temp & 0xFF;	
		
		AS1_printDWordAsCharsDec (i*2);
		AS1_printStringEx ( " " );
		AS1_printWord16AsChars (tmp1);
		AS1_printStringEx ( " " );
		if (tmp1 >= '0' && tmp1 <= 'z' )
			AS1_printByteAsChars ( tmp1 );
		else 
			AS1_printStringEx ("*");
		
		AS1_printStringEx ( "\r\n" );	
		AS1_printDWordAsCharsDec (i*2+1);
		AS1_printStringEx ( " " );
		AS1_printWord16AsChars (tmp2);
		AS1_printStringEx ( " " );
		if (tmp2 >= '0' && tmp2 <= 'z' )
			AS1_printByteAsChars ( tmp2 );
		else 
			AS1_printStringEx ("*");
		AS1_printStringEx ( "\r\n" );	
	}
}
#endif

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