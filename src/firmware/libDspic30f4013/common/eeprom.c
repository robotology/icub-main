#include <p30f4013.h>
#include <timer.h>
#include <string.h>
#include <libpic30.h>
#include <dsp.h>
#include <reset.h>

#include "eeprom.h"

// EEDATA *must* be declared GLOBAL
// EEProm data structures init



void SaveEepromBoardConfig()
{
	// 
	// Save Board Cnfiguration to EE
	//

	_prog_addressT EE_addr;
	int i=0, *DMAdx;
	
	// initialize a variable to represent the Data EEPROM address 
	_init_prog_address(EE_addr, ee_data);
	
	// Erase Data EEPROM at ee_data  
	for(i=0 ; i < sizeof(BoardConfig) ; i++)
	{
	  _erase_eedata(EE_addr++, _EE_WORD);
	  _wait_eedata();
	}
	
	// Write BoardConfig to Data EEPROM
	_init_prog_address(EE_addr, ee_data);
	DMAdx = (int*) &BoardConfig;
	for(i=0 ; i < sizeof(BoardConfig) ; i++)
	{
	  _write_eedata_word(EE_addr, *DMAdx );
	  EE_addr+=2;
	  DMAdx++;
	  _wait_eedata();
	}
}

void RecoverConfigurationFromEEprom()
{
  	_prog_addressT EE_addr;
    _init_prog_address(EE_addr, ee_data);
    _memcpy_p2d16(&BoardConfig, EE_addr, sizeof(BoardConfig));
}
