/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 *
 */



#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sys/time.h>

#define TCREATE struct timeval timing_t0;struct timeval timing_t1;struct timeval timing_tresult;struct timezone timing_tz;

#define TSTART gettimeofday(&timing_t0,&timing_tz);

#define TSTOP gettimeofday(&timing_t1,&timing_tz);timeval_subtract(&timing_tresult, &timing_t1, &timing_t0); printf("%ds %dus\n",timing_tresult.tv_sec,timing_tresult.tv_usec);

#define TEND delete &timing_t1; delete &timing_t0; delete &timing_tz; delete &timing_tresult;

void timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y);
