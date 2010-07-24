#ifndef __decoupling_h__
#define __decoupling_h__

#include "dsp56f807.h"

void decouple_positions (void);
void decouple_dutycycle (Int32 *);
void decouple_dutycycle_new (Int32 *);

#endif