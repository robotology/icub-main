/*
 * dsp56f807.c
 *
 */

#include "dsp56f807.h"


volatile word __dummy = 0;

volatile word SR_lock=0;               /* Lock */
volatile word SR_reg;                  /* Current value of the SR reegister */

