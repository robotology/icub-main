
#ifndef _MMX_CORREL_H
#define _MMX_CORREL_H

#include "MMXMatrix.h"

MMXMatrix *MMXCorrelKernel(MMXMatrix *m, float *Tbar, float* const Tsq);
MMXMatrix *MMXCorrelKernel2(MMXMatrix *m, float *Tbar, float *Tsq);
MMXMatrix *MMXCorrel(MMXMatrix *I, MMXMatrix *T, int w, float Tbar, float Tsq);

#endif
