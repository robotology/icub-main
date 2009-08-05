#include <p30f4013.h>
#include <timer.h>
#include <string.h>
#include <libpic30.h>
#include <dsp.h>
#include <reset.h>

#include "ext_mux.h"
#include "errors.h"

void AMUXChannelSelect(unsigned CH)
// select the specified analog nux channel and 
// perform phisical channel descrambling
// 
{
  switch (CH)
  {
  case 0: 
    // CH1 -> (J1-5) -> S5 -> A(1,0,0) -> RB(...x,x,0,0,1,x,x,x)
    LATBbits.LATB3 = 1; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 0; // A0
  break;
  case 1: 
    // CH2 -> (J1-2) -> S7 -> A(1,1,0) -> RB(...x,x,0,1,1,x,x,x)
    LATBbits.LATB3 = 1; // A2
    LATBbits.LATB4 = 1; // A1
    LATBbits.LATB5 = 0; // A0
  break;
  case 2:
    // CH3 -> (J3-2) -> S1 -> A(0,0,0) -> RB(...x,x,0,0,0,x,x,x)
    LATBbits.LATB3 = 0; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 0; // A0
  break;
  case 3: 
    // CH4 -> (J3-5) -> S4 -> A(0,1,1) -> RB(...x,x,1,1,0,x,x,x)
    LATBbits.LATB3 = 0; // A2
    LATBbits.LATB4 = 1; // A1
    LATBbits.LATB5 = 1; // A0
  break;
  case 4: 
    // CH5 -> (J2-5) -> S2 -> A(0,0,1) -> RB(...x,x,1,0,0,x,x,x)
    LATBbits.LATB3 = 0; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 1; // A0
  break;
  case 5: 
    // CH6 -> (J2-2) -> S6 -> A(1,0,1) -> RB(...x,x,1,0,1,x,x,x)
    LATBbits.LATB3 = 1; // A2
    LATBbits.LATB4 = 0; // A1
    LATBbits.LATB5 = 1; // A0
  break;

  default:
    // add an MUXINDEXINGERROR error in cueue
    EEnqueue(ERR_MUX_INDEXING);
  break;
  }
}



