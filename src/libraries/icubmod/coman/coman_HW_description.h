#ifndef _COMAN_HW_DESCRIPTION_H_
#define _COMAN_HW_DESCRIPTION_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <assert.h>
#include <pthread.h>
#include <bits/local_lim.h>

#include <boost/circular_buffer.hpp>


#include "Debug.h"
#include <string.h>

#warning "_AC_: This file contains a description of how the coman robot is built... \
this should be moved into some comfiguration file in the future!!"


// home position in degree
static const float arrayHomePos[] = {
    // lower body #15
    0,  1,  2,  3,  0,  0,  0, -2,  0,  0,  0,  0, -2,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 70,  0,-10,  0,-70,  0,-10,  0,  0};
// 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 

// boards ID
static const float arrayRLeg[]  = {  4,  6,  7,  8,  9, 10};
static const float arrayLLeg[]  = { 5, 11, 12, 13, 14, 15};
static const float arrayWaist[] = { 1,  2, 3};
static const float arrayRArm[]  = { 16, 17, 18 ,19};
static const float arrayLArm[]  = { 20, 21, 22, 23};
static const float arrayNeck[]  = { 24, 25};

#endif
