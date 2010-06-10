/* 
*  mpisearchBinary.cc -- legacy support file for face detector
 * 
 *  Created by Ian Fasel on Feb 02, 2003.
 * 
 *  Copyright (c) 2003 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *  
 * ......
 */
#ifndef _MPISEARCH_BINARY_H_
#define _MPISEARCH_BINARY_H_

#include "mpisearchFaceDetector.h"

// Legacy support: Many applications need to refer to it as MPISearchBinary
typedef MPISearchFaceDetector MPISearchBinary;
#define MPISEARCH_PIXEL_TYPE float

#endif
