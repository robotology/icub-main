#include <p30f4013.h>
#include <timer.h>
#include <adc12.h>
#include <string.h>
#include <libpic30.h>
#include <dsp.h>

#include "int_adc.h"
#include "eeprom.h"

extern struct s_eeprom BoardConfig; 


void init_internal_adc()
{
  unsigned int PinConfig;
  unsigned int Adcon3_reg, Adcon2_reg, Adcon1_reg;
	
  ADCON1bits.ADON = 0;         // turn off ADC

  ConfigIntADC12(ADC_INT_DISABLE);

  PinConfig  =
    ENABLE_AN0_ANA & // [ADPCFG] enable 0..12 as analog inputs 
    ENABLE_AN1_ANA &
    ENABLE_AN2_ANA &
    ENABLE_AN3_ANA &
    ENABLE_AN4_ANA &
    ENABLE_AN5_ANA &
    ENABLE_AN6_ANA &
    ENABLE_AN7_ANA &
    ENABLE_AN8_ANA &
    ENABLE_AN9_ANA &
    ENABLE_AN10_ANA &
    ENABLE_AN11_ANA;

  Adcon3_reg = ADC_SAMPLE_TIME_10 & 
    ADC_CONV_CLK_SYSTEM & 
    ADC_CONV_CLK_13Tcy;

  Adcon2_reg = ADC_VREF_AVDD_AVSS & 
    ADC_SCAN_OFF & // use ch selected in adchs
    ADC_ALT_BUF_OFF &
    ADC_ALT_INPUT_OFF &
    ADC_SAMPLES_PER_INT_1; // use one buffer, enable done reading

  Adcon1_reg = ADC_MODULE_OFF & 
    ADC_IDLE_CONTINUE & 
    ADC_FORMAT_INTG &
    ADC_CLK_MANUAL &
    ADC_AUTO_SAMPLING_OFF &
    ADC_SAMP_OFF;

  ADPCFG = PinConfig;

  // configure the input scan selection bits
  ADCON3 = Adcon3_reg;
  ADCON2 = Adcon2_reg;
  ADCON1 = Adcon1_reg;

  IFS0bits.ADIF = 0;
  ADCON1bits.ADON = 1;   
  // 
  // MAIS
  // 
//
//  unsigned int Channel, PinConfig, Scanselect;
//  unsigned int Adcon3_reg, Adcon2_reg, Adcon1_reg;
//
//  ADCON1bits.ADON = 0;         // turn off ADC
//
//  Channel = 
//    ADC_CH0_POS_SAMPLEA_AN0 &  // [CH0SA]
//    ADC_CH0_POS_SAMPLEA_AN1 & 
//    ADC_CH0_POS_SAMPLEA_AN2 & 
//    ADC_CH0_POS_SAMPLEA_AN3 & 
//    ADC_CH0_POS_SAMPLEA_AN4 & 
//    ADC_CH0_POS_SAMPLEA_AN5 & 
//    ADC_CH0_POS_SAMPLEA_AN6 & 
//    ADC_CH0_POS_SAMPLEA_AN7 & 
//    ADC_CH0_POS_SAMPLEA_AN8 & 
//    ADC_CH0_POS_SAMPLEA_AN9 & 
//    ADC_CH0_POS_SAMPLEA_AN10 & 
//    ADC_CH0_POS_SAMPLEA_AN11 & 
//    ADC_CH0_POS_SAMPLEA_AN12 & 
//    
//    ADC_CH0_NEG_SAMPLEA_NVREF; // [CH0NA] A/D Chan 0 neg i/p sel for SAMPLE A is -Vref 
//  SetChanADC12(Channel);
//
//  ConfigIntADC12(ADC_INT_PRI_3 & ADC_INT_ENABLE);
//
//  PinConfig  = // ENABLE_ALL_ANA;  // enable 0..15 analog inputs 
//    ENABLE_AN0_ANA & // [ADPCFG] enable 0..12 as analog inputs 
//    ENABLE_AN1_ANA &
//    ENABLE_AN2_ANA &
//    ENABLE_AN3_ANA &
//    ENABLE_AN4_ANA &
//    ENABLE_AN5_ANA &
//    ENABLE_AN6_ANA &
//    ENABLE_AN7_ANA &
//    ENABLE_AN8_ANA &
//    ENABLE_AN9_ANA &
//    ENABLE_AN10_ANA &
//    ENABLE_AN11_ANA &
//    ENABLE_AN12_ANA;
//
//  Scanselect = // SCAN_ALL;        // scan all channels
//    SKIP_SCAN_AN13 & // [ADCCSSL] Skip ch13, 14 and 15
//    SKIP_SCAN_AN14 &
//    SKIP_SCAN_AN15;
//
//  Adcon1_reg = ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_FORMAT_INTG & 
//    ADC_CLK_AUTO &          // 
//    ADC_AUTO_SAMPLING_ON;   // ADC_AUTO_SAMPLING_OFF;
//  
//  Adcon2_reg =
//    ADC_VREF_AVDD_AVSS &     // [VCFG]  uses AVDD e AGND for reference
//    ADC_SCAN_ON &            // [CSCNA] scan ch0+ inputs
//    ADC_ALT_BUF_OFF &        // [BUFM]  Buffer configured as 1 16-word buffer
//    ADC_ALT_INPUT_OFF &      // [ALTS]  uses only MUXA 
//    ADC_SAMPLES_PER_INT_13;  // [SMPI]  1 IRQ each complete 13 channels scan
//
//  Adcon3_reg = ADC_SAMPLE_TIME_10 & ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_32Tcy;
//
//  OpenADC12(Adcon1_reg, Adcon2_reg, Adcon3_reg,PinConfig, Scanselect);
//
//  ADCON1bits.SAMP = 1;
}


void get_sample_internal_adc()
{
  // 
  // MAIS
  // 

  //LATFbits.LATF4  = ~LATFbits.LATF4; //toggLED
  //LATFbits.LATF5  = ~LATFbits.LATF5;

  // verify if the channel is among active ones 
  if( BoardConfig.EE_AN_ActiveChannels[0] !=0)
    BoardConfig.EE_AN_ChannelValue[0] = ReadADC12(0);
  else  
    BoardConfig.EE_AN_ChannelValue[0] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[1] !=0)
    BoardConfig.EE_AN_ChannelValue[1] = ReadADC12(1);
  else  
    BoardConfig.EE_AN_ChannelValue[1] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[2] !=0)
    BoardConfig.EE_AN_ChannelValue[2] = ReadADC12(2);
  else  
    BoardConfig.EE_AN_ChannelValue[2] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[3] !=0)
    BoardConfig.EE_AN_ChannelValue[3] = ReadADC12(3);
  else  
    BoardConfig.EE_AN_ChannelValue[3] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[4] !=0)
    BoardConfig.EE_AN_ChannelValue[4] = ReadADC12(4);
  else  
    BoardConfig.EE_AN_ChannelValue[4] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[5] !=0)
    BoardConfig.EE_AN_ChannelValue[5] = ReadADC12(5);
  else  
    BoardConfig.EE_AN_ChannelValue[5] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[6] !=0)
    BoardConfig.EE_AN_ChannelValue[6] = ReadADC12(6);
  else  
    BoardConfig.EE_AN_ChannelValue[6] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[7] !=0)
    BoardConfig.EE_AN_ChannelValue[7] = ReadADC12(7);
  else  
    BoardConfig.EE_AN_ChannelValue[7] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[8] !=0)
    BoardConfig.EE_AN_ChannelValue[8] = ReadADC12(8);
  else  
    BoardConfig.EE_AN_ChannelValue[8] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[9] !=0)
    BoardConfig.EE_AN_ChannelValue[9] = ReadADC12(9);
  else  
    BoardConfig.EE_AN_ChannelValue[9] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[10] !=0)
    BoardConfig.EE_AN_ChannelValue[10] = ReadADC12(10);
  else  
    BoardConfig.EE_AN_ChannelValue[10] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[11] !=0)
    BoardConfig.EE_AN_ChannelValue[11] = ReadADC12(11);
  else  
    BoardConfig.EE_AN_ChannelValue[11] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[12] !=0)
    BoardConfig.EE_AN_ChannelValue[12] = ReadADC12(12);
  else  
    BoardConfig.EE_AN_ChannelValue[12] = 0;

  if( BoardConfig.EE_AN_ActiveChannels[13] !=0)
    BoardConfig.EE_AN_ChannelValue[13] = ReadADC12(13);
  else  
    BoardConfig.EE_AN_ChannelValue[13] = 0;
}
