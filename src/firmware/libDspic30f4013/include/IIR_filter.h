#ifndef IIR_FILTER_H
#define IIR_FILTER_H

// EEPROM memorized IIR filter coefs 
// has to be separated from s_eeprom var 'couse inits data in xmem
#define IIR_LPF_N_MAX_BQ 5  // Numero massimo di BiQuads dell'IIR LPF 
typedef struct s_eeIIRTransposedCoefs
{
  int IIR_N_BQ;
  fractional IirTransposedCoefs[5*IIR_LPF_N_MAX_BQ];   
} s_eeIIRTransposedCoefs;

extern IIRTransposedStruct iirt[6];
extern s_eeIIRTransposedCoefs IirTransposedCoefs;
extern s_eeIIRTransposedCoefs /*_EEDATA(1)*/ eeIIRTransposedCoefs;
extern fractional IirTransposedState1[6][IIR_LPF_N_MAX_BQ * 2];
extern fractional IirTransposedState2[6][IIR_LPF_N_MAX_BQ * 2];
extern void IIRTransposedInit(IIRTransposedStruct* filter);
extern fractional* IIRTransposed (int numSamps, fractional* dstSamps, fractional* srcSamps, IIRTransposedStruct* filter);

void IIRFiltersINIT();
void RecoverIIRFilterFromEEprom();
void SaveEepromIIRFilter();

#endif
