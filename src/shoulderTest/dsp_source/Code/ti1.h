/*
 * ti1.h
 *	the interface and definitions to the timer interrupt handler.
 *
 */

#ifndef __ti1h__
#define __ti1h__

/**
 * \file ti1.h
 * the definitions of the timer interrupt module.
 */

#include "dsp56f807.h"

/*
 * prototypes.
 */
void TI1_init (void);
void TI1_interrupt (void);
void TIC_init (void);

#endif /* ifndef __TI1 */
