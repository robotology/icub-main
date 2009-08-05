#include <spi.h>
#include <dsp.h>

#include "utils.h"
#include "errors.h"
#include "ext_adc_dac.h"

unsigned int SPISTATValue; // Holds the information about SPI Enable/Disable
static unsigned int SPICONValue; // Holds the information about SPI configuartion

void SetDAC(unsigned PowerDownMode, unsigned int DACValue)
{
  unsigned int tmp; 

  //LATBbits.LATB6=1;  //led on

  if( DACValue > 0x3FF) 
  {
    EEnqueue(ERR_DAC_VALUE2BIG);
    DACValue = 0x3FF;  // saturate to max
  }

//LATAbits.LATA11 = 1; // startconv must be longer than 3.2u  

  while(SPI1STATbits.SPITBF);  // transmission buffer full
  tmp = PDM_NORMAL & (DACValue<<2);
  __delay32(30);
  LATDbits.LATD9  = 0; // SS DAC 
//LATAbits.LATA11 = 0;  
 
  WriteSPI1(tmp); 
  __delay32(10); 
  LATDbits.LATD9 = 1;
  //*ADCValue=ReadSPI1(); // get ADC value

  //LATBbits.LATB6=0; //led off
}

void GetADC(unsigned int *ADCValue)
{
  //unsigned int tmp; 

LATBbits.LATB6=1;  //led on
LATAbits.LATA11 = 1; // startconv must be longer than 3.2u  

  while(SPI1STATbits.SPITBF);  // transmission buffer full
  //tmp = PDM_NORMAL & (DACValue<<2);
  __delay32(50);
 //LATDbits.LATD9  = 0; // SS DAC 
LATBbits.LATB6=0; //led off
LATAbits.LATA11 = 0;  
 
  //WriteSPI1(tmp); 
  __delay32(10); 
 //LATDbits.LATD9 = 1;
 *ADCValue=ReadSPI1(); // get ADC value

}

void SetDACGetADC(unsigned PowerDownMode, unsigned int DACValue, unsigned int *ADCValue)
// set the value for the DAC and get ADC value
// PAY ATTENCTION: do not change the sequence of operation: tight timing at work!
// Durata circa 3 usec
{
  unsigned int tmp; 
  if( DACValue > 0x3FF) 
  {
    EEnqueue(ERR_DAC_VALUE2BIG);
    DACValue = 0x3FF;  // saturate to max
  }
LATBbits.LATB6=1;  //led on
LATAbits.LATA11 = 1; // startconv must be longer than 3.2u  

  while(SPI1STATbits.SPITBF);  // transmission buffer full
  tmp = PDM_NORMAL & (DACValue<<2);
  __delay32(30);
 LATDbits.LATD9  = 0; // SS DAC 
LATAbits.LATA11 = 0;  
 LATBbits.LATB6=0; //led off
 
  WriteSPI1(tmp); 
  __delay32(10); 
 LATDbits.LATD9 = 1;
 *ADCValue=ReadSPI1(); // get ADC value
}

void InitSPI()
{
  // Configure SPI1 module to transmit 16 bit timer1 value in master mode
  SPICONValue  =  FRAME_ENABLE_OFF & FRAME_SYNC_OUTPUT &
    ENABLE_SDO_PIN & SPI_SMP_ON & SPI_CKE_OFF & SPI_MODE16_ON &
    SLAVE_ENABLE_OFF & CLK_POL_ACTIVE_HIGH &
    MASTER_ENABLE_ON & SEC_PRESCAL_1_1 & PRI_PRESCAL_1_1; 

  SPISTATValue  = SPI_ENABLE & SPI_IDLE_CON & SPI_RX_OVFLOW_CLR;
  ConfigIntSPI1(SPI_INT_PRI_4 &  SPI_INT_EN);

  OpenSPI1(SPICONValue,SPISTATValue );
}

//
// SPI IRQ Service Routines
// 
void __attribute__((interrupt, no_auto_psv)) _SPI1Interrupt(void)
{   
  IFS0bits.SPI1IF = 0;
}  
