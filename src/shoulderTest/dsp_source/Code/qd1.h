/*
 * qd1.h
 * quadrature decoder interface.
 *
 */

/**
 * \file qd1.h
 * these are the definitions of the quadrature decoder interface.
 */
 
#ifndef __qd1h__
#define __qd1h__

#include "dsp56f807.h"

#ifndef __BWUserType_TStateValues
#define __BWUserType_TStateValues
  typedef struct {                     /* Structure contains actual state of the position, position diference and revolution counters. */
    dword Position;                    /* Position */
    word Difference;                   /* Difference position */
    word Revolution;                   /* Revolution */
  } TStateValues;
#endif



byte QD1_setPositionInit (dword Position);
byte QD1_setPosition (dword Position);
byte QD1_getPosition (dword *Position);
byte QD1_initPosition (void);
byte QD1_getSignals (bool Filtered, word *Signals);
void QD1_init (void);

#endif /* ifndef __qd1h__ */
