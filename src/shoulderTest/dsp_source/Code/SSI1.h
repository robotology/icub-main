 /* SSI1.h
 * 	SSI1 communication interface
 */
 
 
#ifndef __SSI1h__
#define __SSI1h__
 
 
/**
 * \file SSI.h 
 */

#include "dsp56f807.h"

void SSI1_Init();
UInt16 SSI1_GetVal(byte mask); //mask define the ChipSelect 


#endif 