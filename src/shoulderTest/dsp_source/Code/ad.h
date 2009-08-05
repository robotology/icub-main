/*
 * ad.h
 * 	AD sampling interface.
 */

#ifndef __adh__
#define __adh__

/**
 * \file ad.h 
 * ad.h contains the interface definition to the AD converter.
 * Current implementation is made to acquire from a single channel only.
 */

#include "dsp56f807.h"

void AD_interruptCCA(void);
void AD_interruptCCB(void);
byte AD_measureA(bool wait);
byte AD_measureB(bool wait);
byte AD_enableIntTriggerA(void);
byte AD_enableIntTriggerB(void);
byte AD_stopAcquisitionA(void);
byte AD_stopAcquisitionB(void);
byte AD_getValue16A(word *values);
byte AD_getValue16B(word *values);
byte AD_getChannel16A(byte i, word *value);
byte AD_getChannel16B(byte i, word *value);
void AD_init(void);


#endif /* ifndef __adh__ */
