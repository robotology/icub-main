#pragma once

#include "cv.h"
#include "highgui.h"

class CPlotter{
private:
	void init();

	CvPoint xAxis_ini, xAxis_fin, yAxis_ini, yAxis_fin;

public:
	CPlotter(void);
	~CPlotter(void);

	double yMax, yMin, yFactor;
	IplImage *plotImg;
	CvSize	wSize;
	const char *wName;

	void init( int, int, int, int, double, double );
	void show( double * );
	void show( double *, double * );

};
