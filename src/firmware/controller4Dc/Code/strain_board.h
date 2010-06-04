#ifndef __strain_board_h__
#define __strain_board_h__

#include "controller.h"

#define STRAIN_MAX 4

#define CAN_ID_JNT_STRAIN_11		11
#define CAN_ID_JNT_STRAIN_12		12
#define CAN_ID_6AX_STRAIN_13		13
#define CAN_ID_6AX_STRAIN_14 		14

#define WDT_JNT_STRAIN_11		0
#define WDT_JNT_STRAIN_12		1
#define WDT_6AX_STRAIN_13		2
#define WDT_6AX_STRAIN_14 		3


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