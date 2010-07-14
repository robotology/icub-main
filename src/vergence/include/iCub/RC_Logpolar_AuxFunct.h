
#ifndef RC_Logpolar_AuxFunct_h
#define RC_Logpolar_AuxFunct_h


//#include <ace/config.h>
//#include <ace/OS.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <iCub/RC_DIST_FB_logpolar_mapper.h>

namespace _logpolarParams
{
	const int _xsize = 320;
	const int _ysize = 240;
	const int _xsizeR = 240;
	const int _ysizeR = 240;
	const int _srho = 152;
	const int _stheta = 252;
	const double _overlap = 0.00;
	const int _fmode = 2;
};

using namespace yarp::sig;

struct Image_Data
{
	// Log Polar Metrics
	int Size_Rho;
	int Size_Theta;
	int Size_LP;
	int Fovea_Mode;
	double overlap;

	// Remapped Cartesian Metrics
	int Size_X_Remap;
	int Size_Y_Remap;
	int Size_Img_Remap;

	// Original Cartesian Metrics
	int Size_X_Orig;
	int Size_Y_Orig;
	int Size_Img_Orig;

	//Computed Parameters
	double scale_Orig;
	double scale_Remap;
	double LogIndex;
	double firstRing;
	double r0;
	int Size_Fovea;

};

struct shift_Struct
{
	int index;
	double disp;
	double corr;

};

Image_Data SetParam(int rho, int theta, int mode, double overlap, int xo, int yo, int xr, int yr);
void Build_Shift_Table(Image_Data par, char *path);
int Load_Shift_Table(Image_Data par, int *shiftTab, double *stepList, char *path);
double getX(Image_Data par, int rho, int theta);
double getY(Image_Data par, int rho, int theta);
int getRho(Image_Data par, double x, double y);
int getTheta(Image_Data par, double x, double y);
void img2unpaddedVect(unsigned char *v, Image img);
void unpaddedVect2img(unsigned char *v, Image &img);

inline double __max (double x, double y)
{
    return (x > y) ? x : y;
}

inline double __min (double x, double y)
{
    return (x < y) ? x : y;
}

inline bool isWithin(int v,int max, int min)
{
	if ( (v <= max) && (v >= min) )
		return true;
	else
		return false;
}

inline double find_max_value(double *v, int size)
{
	double tmpMax = v[0];
	for (int i = 0; i < size; i++)
	{
		if (v[i] >= tmpMax)
			tmpMax = v[i];
	}
	return tmpMax;
}

inline int find_max_index(double *v, int size)
{
	int ind = 0;
	double tmpMax = v[0];
	for (int i = 0; i < size; i++)
	{
		if (v[i] >= tmpMax)
		{
			tmpMax = v[i];
			ind = i;
		}
	}
	return ind;
}

inline double find_min_value(double *v, int size)
{
	double tmpMin = v[size-1];
	for (int i = size - 1; i >= 0; i--)
	{
		if (v[i] <= tmpMin)
			tmpMin = v[i];
	}
	return tmpMin;
}

inline int find_min_index(double *v, int size)
{
	int ind = size - 1;
	double tmpMin = v[0];
	for (int i = size - 1; i >= 0; i--)
	{
		if (v[i] <= tmpMin)
		{
			tmpMin = v[i];
			ind = i;
		}
	}
	return ind;
}

#endif
