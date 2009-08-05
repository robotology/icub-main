#include <p30f4013.h>
#include <libpic30.h>
#include <dsp.h>

#include "ports.h"


void InitPorts()
{
  // 
  // MAIS
  // 
  TRISFbits.TRISF4 = 0; // set F4 as out (LED Yellow)
  TRISDbits.TRISD1 = 0; // set F5 as out (LED Blue)
}
