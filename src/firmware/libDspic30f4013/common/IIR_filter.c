#include <p30f4013.h>
#include <timer.h>
#include <string.h>
#include <libpic30.h>
#include <adc12.h>
#include <dsp.h>
#include <reset.h>
#include "IIR_filter.h"

// eeprom filter data init
/*static*/ struct s_eeIIRTransposedCoefs _EEDATA(1) eeIIRTransposedCoefs =
{
  2,      // n. BiQuads
  {
  // filter coefs LPF 50Hz 
  0x00E3,
  0x00DD,
  0x68D6,
  0x00E3,
  0xD487,
  0x0343,
  0xFDD5,
  0x728F,
  0x0343,
  0xC914
  }
};

IIRTransposedStruct iirt[6];
s_eeIIRTransposedCoefs IirTransposedCoefs __attribute__((__space__(xmemory))) = { 0 };
fractional IirTransposedState1[6][IIR_LPF_N_MAX_BQ * 2] __attribute__((__space__(ymemory)));
fractional IirTransposedState2[6][IIR_LPF_N_MAX_BQ * 2] __attribute__((__space__(ymemory)));

void IIRFiltersINIT()
{
  int i;
  
  for( i=0; i<6 ; i++) 
  { 
    iirt[i].numSectionsLess1 = IirTransposedCoefs.IIR_N_BQ - 1;
    iirt[i].coeffsBase = &IirTransposedCoefs.IirTransposedCoefs[0];
    iirt[i].coeffsPage = COEFFS_IN_DATA;
    iirt[i].delayBase1 = &IirTransposedState1[i][0];
    iirt[i].delayBase2 = &IirTransposedState2[i][0];
	iirt[i].finalShift = 0; 
  
    // Zero the filter state
  //  IIRTransposedInit(&iirt[i]);
  }
}

void SaveEepromIIRFilter()
{
	// 
	// Save IIR Filter Coefs to EE
	//
	
	_prog_addressT EE_addr;
	int i=0, *DMAdx;

	// initialize Data EEPROM address 
	_init_prog_address(EE_addr, eeIIRTransposedCoefs);
	
	// Erase Data EEPROM at ee_data  
	for(i=0 ; i < sizeof(IirTransposedCoefs) ; i++)
	{
	  _erase_eedata(EE_addr++, _EE_WORD);
	  _wait_eedata();
	}
	
	// Write eeIIRTransposedCoefs to Data EEPROM
	_init_prog_address(EE_addr, eeIIRTransposedCoefs);
	// save NBiQuads+coeffs
	DMAdx = (int*) &(IirTransposedCoefs);
	for(i=0 ; i < sizeof(IirTransposedCoefs) ; i++)
	{
	  _write_eedata_word(EE_addr, *DMAdx );
	  EE_addr+=2;
	  DMAdx++;
	  _wait_eedata();
	}
}

void RecoverIIRFilterFromEEprom()
{
  	_prog_addressT EE_addr;
    _init_prog_address(EE_addr, eeIIRTransposedCoefs);
    _memcpy_p2d16(&IirTransposedCoefs, EE_addr, sizeof(IirTransposedCoefs));
}
