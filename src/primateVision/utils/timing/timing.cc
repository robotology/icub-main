/**
 * @ingroup icub_primatevision_utils
 *
 * \defgroup icub_primatevision_utils_timing Timing
 *
 * 
 * Simple timing macros used for loop timing.  Simply type "TCREATE" before entering the loop to initiate timer, then "TSTART" immediately after entering the loop, and "TSTOP" immediately before exiting the loop. "TEND" calls the destructor.
 *
 *
 *For example, the following code will print the loop duration once every cycle:
 *
 *<pre>
 main(){
   
  TCREATE
 
  while(true){
   TSTART
 
   //some processing.
 
 
   TSTOP
  }//while
 
  TEND

 }
 </pre>
 *
 *
 * \author Andrew Dankers
 *
 *
 * This file can be edited at \in src/primateVision/utils/timing/timing.cc
 */



/*
 * Copyright (C) 2003-2008 Andrew Dankers. All rights reserved.
 * 
 *
 */


#include <stdio.h>
#include <iostream>
#include <math.h>
#include <sys/time.h>

using namespace std;


//timing macros:

//write TCREATE at beginning of program, and TEND at end.
//then write "TSTART" at the beginning and "TSTOP" at the end of the 
//section of code (or loop) you want to time.  The rest is done for you!

void timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y){ 
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec; 
}


