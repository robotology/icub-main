
#ifndef _MMX_KERNEL_H
#define _MMX_KERNEL_H

#include "MMXMatrix.h"

MMXMatrix *gaussMatrix(mmtype type, int w, int h, float sigmaX, float sigmaY);
MMXMatrix *gaussKernel(mmtype type, int w, int h, float sigmaX, float sigmaY);
MMXMatrix *sobelKernel(mmtype type, int w, int h, 
		       float angle, float sigmaX, float sigmaY);

#endif
