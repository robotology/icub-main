#ifndef __strain_board_h__
#define __strain_board_h__

#include "controller.h"

#define STRAIN_MAX 2

extern Int16 _strain[STRAIN_MAX][6];
extern Int16 _strain_init[STRAIN_MAX][6];
extern Int16 _strain_old[STRAIN_MAX][6];
extern byte  _strain_wtd[STRAIN_MAX];  		//watchdog timer: disables pwm if strain doesn't communicate
#define STRAIN_SAFE 15     					//number of ms for the watchdog timer

void init_strain ();
word read_strain(byte jnt, bool sign);
void start_strain(word can_address);
void stop_strain(word can_address);


#endif