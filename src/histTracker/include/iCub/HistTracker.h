// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_HISTTRACKER_INC
#define ICUB_HISTTRACKER_INC

#ifndef PI
#define PI 3.14159265358979323846
#endif

#include <cv.h>
#include <iCub/weightedHistCalc.h>
#include <math.h>
//yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>


namespace iCub {
    namespace contrib {
        class HistTracker;
    }
}

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::contrib;

class YarpTimer 
{
private:
	double global_start, global_end, global_acc, lap_partial, lap_end, last_lap, lap_acc;
	int lap_counter;
public:
	YarpTimer(){reset();};
	~YarpTimer(){};

	void reset(){
		lap_counter = 1;
		last_lap = 0;
		global_acc = 0;
		lap_acc = 0;
		global_start = Time::now();
	};

	double now(){
		return Time::now() - global_start;
	};

	void endlap(){
		lap_counter++;
		global_acc += lap_acc;
		last_lap = lap_acc;
		lap_acc = 0;
		start();
	}

	void start(){
		lap_partial = Time::now();
	}

	void stop()	{
		double delta = Time::now() - lap_partial;	
		lap_acc += delta;
	}

	// acessors
	int cyc(){return lap_counter;}
	double lap(){return lap_acc;};
	double lastlap(){return last_lap;};
	double tot(){return lap_acc;}
	double avg(){
		if(lap_counter)
			return (global_acc/lap_counter);
		else
			return 0;
	}
};

/**
 * A color salience filter, following Laurent Itti and Brian Scassellati.
 */
class iCub::contrib::HistTracker : public IConfig {
public:
	HistTracker();
	virtual ~HistTracker();
    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();
	bool track(const Image &in);
	bool init(CvSize currImgSize);
	//bool learnModel(const Image &in);
	bool learnModel( const Image &in, const CvRect &track_window_initial );
	bool turnedOn;
	CvRect track_window;
    CvRect track_window_actual;
	double totalTrackTime;
private:
	int nIter;
	bool _needInit;
	CvSize          _oldInSize;
	YarpTimer cycleTimer;
	IplImage *currentColorImage;
	IplImage *imgGray;
	IplImage *currentGrayFloatImage;
	IplImage *redOriginal;
	IplImage *greenOriginal;
	IplImage *blueOriginal;
	IplImage *blueNorm;
	IplImage *epanechnikovK;
	IplImage *binLookUpTableRed;
	IplImage *binLookUpTableGreen;
	IplImage *binLookUpTableY0Red;
	IplImage *binLookUpTableY0Green;
	IplImage *redNorm;
	IplImage *greenNorm;
	IplImage *redNormCopy;
	IplImage *greenNormCopy;
	float bhattacharyyaThresh;
	float epsilon;
	bool backgroundWeight;
	bool kalmanFilter;
	bool adaptiveScale;
	bool bhattacharyyaThreshActive;
	float sigmaX;
	float sigmaY;
	float sigmaXVel;
	float sigmaYVel;
	float sigmaXNoise;
	float sigmaYNoise;
	float sigmaXVelNoise;
	float sigmaYVelNoise;
	CvMat *P;
	CvMat *Q;
	CvMat *H;
	CvMat *currentObservation;
	CvMat *currentXHat;
	CvMat *precedentXHat;
	CvMat *SMatrix;
	CvMat *KMatrix;
	CvMat *identityMat;
	CvMat *RMatrix;
	CvMat *F;
	CvMat *Ft;
	CvMat *innovation;
	CvMat *Hx;
	CvMat *Ht;
	CvMat *KH;
	CvMat *PH;
	CvMat *tempRes;
	int nIterMax;
	float gamma;
	float deltaScaleRatio;
	int binSize;
	CvHistogram *hist2D;
	CvHistogram *y0Hist;
	CvHistogram *y1Hist;
	CvHistogram *backgroundHist2D;
	CvHistogram *weightedHist2D;
	CvHistogram *weightedHist2DY0;
	float* ranges[2];
	int hist_size[2];
	float *myHist;
	float *myHistY0;
	//CvRect track_window;
	CvRect background_window;
	CvRect background_window_actual;
	float timeDelta;
	int initScale;
	int finalScale;
	float bhattacharyyaCoef[3];
	float deltaX[3];
	float deltaY[3];
	float scaleRatio[3];
	float currentXPos;
	float currentYPos;
	float currentWidth;
	float currentHeight;
	int scaleCounter;
	float xHatPrecedent;
	float yHatPrecedent;
	//float currentXVel;
	//float currentYVel;
	int width;
	int height;
	int converge;
	float optimalScaleRatio;
	float optimalDeltaX;
	float optimalDeltaY;
	float currentBhattacharyyaCoef;
	float optimalWidth;
	float optimalHeight;
	float newWidth;
	float newHeight;
	float **backgroundWeights;
	float **weightedHistogramPointer;
	void backgroundWeightedHistogramComputation ( IplImage** imageCopy, CvRect track_window, CvRect background_window,
											 CvHistogram *backgroundHistogram, CvHistogram *weightedHistogram,
											 CvHistogram *modelHistogram, int *hist_size, float **ranges, 
											 float *histogramPointer,
											 float **backgroundWeights,
											 float **weightedHistogramPointer);
	void imageBoundingBoxComputation ( CvRect actualWindow, CvRect *displayWindow, int imageWidth, int imageHeight );
	int round (float value){
	if (value < 0)
		return -(int)(floorf(-value + 0.5f));
	// or as an alternate use:
	// return ceil ( value - 0.5);
	else
		return   (int)floorf( value + 0.5f);
	}
};

#endif
