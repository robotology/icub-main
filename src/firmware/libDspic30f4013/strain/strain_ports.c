#include <p30f4013.h>
#include <libpic30.h>
#include <dsp.h>

#include "ports.h"

void InitMaisIoPorts()
{
  // 
  // MAIS
  // 
  TRISFbits.TRISF4 = 0; // set F4 as out (LED Yellow)
  TRISFbits.TRISF5 = 0; // set F5 as out (LED Blue)
}

void InitStrainIoPorts()
{
  //
  // STRAIN
  //

  // TRISA = 0x0800;  // all inputs
  TRISB = 0x1FFF;  // all inputs
  TRISC = 0xE000;  // all inputs
  TRISD = 0x030F;  // all inputs
  // TRISE = 0x013F;  // all inputs
  TRISF = 0x007F;  // all inputs

  // - - - 

  TRISBbits.TRISB12 = 0; // set B12 as out (LED)

//@@@@
// For test puropuses
  TRISBbits.TRISB6 = 0; // set sampling trigger
  TRISBbits.TRISB7 = 0; // set mux CH1 trigger
  LATBbits.LATB6=0; 
  LATBbits.LATB7=0;
//@@@@

  TRISBbits.TRISB5  = 0;  // set B5 as out (MUXA0)
  TRISBbits.TRISB4  = 0;  // set B4 as out (MUXA1)
  TRISBbits.TRISB3  = 0;  // set B3 as out (MUXA2)
  
  TRISAbits.TRISA11 = 0; // set A11 as out (STARTCONV ADC)
  LATAbits.LATA11   = 0; // Start conversion inactive   

  TRISDbits.TRISD9  = 0; // set D9 as out (\SS1 = \SS DAC)
  LATDbits.LATD9    = 1; // SS DAC inactive
}
