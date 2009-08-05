#ifndef POSITIONESTIMATOR
#define POSITIONESTIMATOR


# define _WINSOCKAPI_

//#include <stdio.h>
//#include "windows.h"  // this doesn't seem needed, and kills Linux -paulfitz


#include <stdio.h>
#include <math.h>
#define Fs 44100

class PositionEstimator
{


public:

float pan_low, pan_high, tilt_low, tilt_high;
	//Class Constructor
	PositionEstimator();		
	
	//Class Destructor
	~PositionEstimator();

	void Position_Finder(double *features, double *map, double *position, double *limits);
	//features contains the sound parameters ITD and notch frequency
	//map contains the parameters to turn the head

};


#endif //PositionEstimator
