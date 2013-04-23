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

#warning "This file contains a description of how the coman robot is built... \
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

///////////////////////////////////////////////////////////////////////////////
//  help funtions
///////////////////////////////////////////////////////////////////////////////


// trajectory will be computed by the external application, not here
#if 0
static void trajectory(int pos[], short vel[], short tor[]) {

    static double freq_Hz = 1.0;
    unsigned long long tNow = get_time_ns();
    unsigned long long dt_ns = tNow - g_tStart;
    double  trj = 1-cos((2.0 * M_PI * freq_Hz * dt_ns)/1e9); // 0..2

    /////////////////////////////////////////////////////////////////
    // lower body boards
    // waist
    pos [ 0] = 0;
    pos [ 1] = DEG2mRAD(homePos [1]) + DEG2mRAD(0)  * trj;
    pos [ 2] = DEG2mRAD(homePos [2]) + DEG2mRAD(0)  * trj;
    // legs
    // right leg
    pos [ 3] = DEG2mRAD(homePos [3]) + DEG2mRAD(10) * trj;
    pos [ 5] = DEG2mRAD(homePos [5]) + DEG2mRAD(-10)  * trj;
    pos [ 6] = DEG2mRAD(homePos [6]) + DEG2mRAD(10)  * trj;
    pos [ 7] = DEG2mRAD(homePos [7]) + DEG2mRAD(10)* trj;
    pos [ 8] = DEG2mRAD(homePos [8]) + DEG2mRAD(5) * trj;
    pos [ 9] = DEG2mRAD(homePos [9]) + DEG2mRAD(0)  * trj;
    // left leg
    pos [ 4] = DEG2mRAD(homePos [4]) + DEG2mRAD(10) * trj;
    pos [10] = DEG2mRAD(homePos[10]) + DEG2mRAD(10)  * trj;
    pos [11] = DEG2mRAD(homePos[11]) + DEG2mRAD(0)  * trj;
    pos [12] = DEG2mRAD(homePos[12]) + DEG2mRAD(10)* trj;
    pos [13] = DEG2mRAD(homePos[13]) + DEG2mRAD(5) * trj;
    pos [14] = DEG2mRAD(homePos[14]) + DEG2mRAD(0)  * trj;
    /////////////////////////////////////////////////////////////////
    // upper body boards
    // r_arm
    pos [15] = DEG2mRAD(homePos[15]) + DEG2mRAD(-10) * trj;
    pos [16] = DEG2mRAD(homePos[16]) + DEG2mRAD(0) * trj;
    pos [17] = DEG2mRAD(homePos[17]) + DEG2mRAD(0) * trj;
    pos [18] = DEG2mRAD(homePos[18]) + DEG2mRAD(-20)     * trj;
    // l_Arm
    pos [19] = DEG2mRAD(homePos[19]) + DEG2mRAD(-10) * trj;
    pos [20] = DEG2mRAD(homePos[20]) + DEG2mRAD(0) * trj;
    pos [21] = DEG2mRAD(homePos[21]) + DEG2mRAD(0) * trj;
    pos [22] = DEG2mRAD(homePos[22]) + DEG2mRAD(-20)  * trj;

    // desired velocity
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        vel[i] = DEG2RAD(50)*1000;
    }

    // desired torque
    for (int i=0; i<MAX_DSP_BOARDS; i++) {
        tor[i] = 0;
    }

}
#endif

#endif
