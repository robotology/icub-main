#ifndef __decoupling_h__
#define __decoupling_h__

#include "dsp56f807.h"

#define USE_NEW_DECOUPLING 1

void decouple_positions (void);
void decouple_dutycycle (Int32 *);
void decouple_dutycycle_new_motor(Int32 *);
void decouple_dutycycle_new_joint(Int32 *pwm);

#endif