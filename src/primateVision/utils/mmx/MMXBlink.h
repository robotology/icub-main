
#ifndef _MMX_BLINK_H
#define _MMX_BLINK_H

#include "MMXMatrix.h"

int MMXBlink(unsigned char *now, unsigned char *then,
	     int width, int height,
	     int *sum, float *vel);

#endif

