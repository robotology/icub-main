#include <p30f4013.h>
#include <timer.h>
#include <string.h>
#include <libpic30.h>
#include <reset.h>

#include "ext_mux.h"
#include "errors.h"
#include <dsp.h>


void AMUXInit(unsigned CH)
{
	TRISCbits.TRISC13=0;
	TRISCbits.TRISC14=0;
}	
	// select the specified analog nux channel and 
// perform phisical channel descrambling
// 

void AMUXChannelSelect(unsigned CH)
// select the specified analog nux channel and 
// perform phisical channel descrambling
// 
{
  // 
  // MAIS
  // 
  switch (CH)
  {
  case 0:  // T1 Thumb
  case 1:  // T2
  case 2:  // T3
  case 3:  // I1 Index
  case 4:  // I2
  case 5:  // I3
  case 6:  // M1 Middle
  case 7:  // M2
  case 8:  // M3
  case 9:  // R1 Ring
  case 10: // R2
  case 11: // R3
    LATCbits.LATC14 = 0; // A14 MUXA0
    LATCbits.LATC13 = 0; // A13 MUXA1
  break;

  case 12:  // L1 Little
    LATCbits.LATC14 = 1; // A14 MUXA0
    LATCbits.LATC13 = 0; // A13 MUXA1
  break;

  case 13:  // L2
    LATCbits.LATC14 = 0; // A14 MUXA0
    LATCbits.LATC13 = 1; // A13 MUXA1
  break;

  case 14:  // L3
    LATCbits.LATC14 = 1; // A14 MUXA0
    LATCbits.LATC13 = 1; // A13 MUXA1
  break;
 
 default:
    // add an MUXINDEXINGERROR error in cueue
    EEnqueue(ERR_MUX_INDEXING);
  break;
  }

}
