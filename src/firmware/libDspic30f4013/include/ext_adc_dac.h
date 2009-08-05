#ifndef EXT_ADC_DAC_H
#define EXT_ADC_DAC_H

// DAC Power Down Modes
#define PDM_NORMAL     0xCFFF
#define PDM_1KTOGND    0xDFFF
#define PDM_100KTOGND  0xEFFF
#define PDM_THREESTATE 0xFFFF

void InitSPI();
void SetDAC(unsigned PowerDownMode, unsigned int DACValue);
void GetADC(unsigned int *ADCValue);
void SetDACGetADC(unsigned PowerDownMode, unsigned int DACValue, unsigned int *ADCValue);

#endif
