/*
 * qd0.h
 * quadrature decoder interface.
 *
 */

/**
 * \file qd0.h
 * these are the definitions of the quadrature decoder interface.
 */
 
#ifndef __qd0h__
#define __qd0h__

#include "dsp56f807.h"

#ifndef __BWUserType_TStateValues
#define __BWUserType_TStateValues
  typedef struct {                     /* Structure contains actual state of the position, position diference and revolution counters. */
    dword Position;                    /* Position */
    word Difference;                   /* Difference position */
    word Revolution;                   /* Revolution */
  } TStateValues;
#endif



byte QD0_setPositionInit (dword Position);
byte QD0_setPosition (dword Position);
byte QD0_getPosition (dword *Position);
byte QD0_initPosition (void);
byte QD0_getSignals (bool Filtered, word *Signals);
void QD0_init (void);

#endif /* ifndef __qd0h__ */
