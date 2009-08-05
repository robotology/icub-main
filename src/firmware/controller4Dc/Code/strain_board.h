#ifndef __strain_board_h__
#define __strain_board_h__

#include "controller.h"

extern Int32 _Feq;
extern Int16 _strain[6];
extern Int16 _strain_init[6];
extern Int16 _strain_old[6];
extern byte  _strain_wtd;  //watchdog timer: disables pwm if strain doesn't communicate
#define STRAIN_SAFE 15     //number of ms for the watchdog timer

void init_strain ();
word read_strain(byte jnt, bool sign);

#endif