
#ifndef _MMX_CONVOLVE_H
#define _MMX_CONVOLVE_H

#include "MMXMatrix.h"

MMXMatrix *convKernel(MMXMatrix *m);
int MMXConvolve(MMXMatrix *src, MMXMatrix *ker, MMXMatrix *dst);

#endif

