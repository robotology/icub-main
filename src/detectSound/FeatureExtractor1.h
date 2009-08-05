#ifndef FEATUREEXTRACTOR1
#define FEATUREEXTRACTOR1

# define _WINSOCKAPI_
//#include "windows.h"  // this doesn't seem needed, and kills Linux -paulfitz
#include <stdio.h>
#include <math.h>
#include <gsl/gsl_multifit.h>
#include "YARPFft.h"

#define Fs 44100


class FeatureExtractor1
{


public:


	double left_ear_Im[4000], right_ear_Im[4000];
	int numsamples;
	double ISD[4000];
	double CrossCorr[4000];

//	FILE *fp_ISD;


	//Class Constructor
	FeatureExtractor1();		
	
	//Class Destructor
	~FeatureExtractor1();

	//int pan(byte *addr, float par, float vel, char blocking);
		/*addr is the motor no to actuate
		par is the angle to rotate in radians
		vel is the velocity with which it rotates
		blocking enabled implies that it is gonna stay there after it reaches its destination*/
	void cross_corr(double *left_ear, double *right_ear, double *CrossCorr);
	int maximum(double *CrossCorr);
	double ILD(double *left_ear, double *right_ear);
	double ITD(double *left_ear, double *right_ear);
	double notch(double *left_ear, double *right_ear);
	int polyfit(double *ISD);

	YARPFft * fft;
};


#endif //FEATUREEXTRACTOR1
