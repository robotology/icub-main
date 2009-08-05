// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Chris McCarthy
 *
*/

#ifndef __MYMODULE_H
#define __MYMODULE_H

 // std
#include <stdio.h>

// opencv
#include <cv.h>
#include <cvDefs.h>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Module.h>

#define BUFF_SIZE	100
#define DIV_LEVEL    0
#define DIV_SCALE    10
#define TTC_SCALE    1.25
#define MAG_SCALE    100
#define MAG_LOW_LIMIT   0.015
#define MAG_HIGH_LIMIT  5.00
#define DIFF_THRESHOLD  60
#define DIV_BOX_SKIP    10
#define MAG_BOX_SKIP    2
#define DIV_BOX         80
#define MAG_BOX         20
#define MAX_POINTS      5000 // number of points
#define WIN_SIZE        5  // LK Pyr window
#define POINT_DIST      15 // distance between flow points
#define MAX_MATCH_WINS  9

#define TEMPLATE_WIN_SIZE     5
#define SUPPORT_WIN_SIZE      5
#define DIV_COUNT_THRESHOLD   0
#define SOBEL_SIZE            7
#define APPROACH_ANGLE_THRESHOLD 0 //CV_PI/4


#define DIV             0
#define DOWN            1
#define RIGHT           2
#define UP              3
#define LEFT            4
#define DOWN_RIGHT      5
#define UP_LEFT         6
#define UP_RIGHT        7
#define DOWN_LEFT       8

// labels for interest points
#define IP_CENTER          1
#define IP_MAX_DIV         2
#define IP_MAXMIN_DIV      3
#define IP_FULL_IMAGE      4

#define NO_TTC          1000

// TTC estimate
#define TTC_KP    0.7
#define TTC_KI    0.2// not used at present
#define TTC_KD    0.0//0.1
#define TTC_N     2
#define TTC_ARRAY_SIZE  (TTC_N+1)

#define rKP 0.5//0.6 //0.8 //0.5
#define rKD 0.0 //0.1//0.8//(1-rKP)
#define rKI 0.2

//tangent angle
#define tKP 0.9
#define tKD 0.1
#define tKI 0.0

// Max div location
#define DIVPOS_KP 0.3//(0.1)
#define DIVPOS_KD 0.1//(0.05)
#define DIVPOS_KI 0//(0.05)

#define ROOT2 1.414214

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class ImpactMapModule : public Module, Thread {

private:
	Port _videoIn;
	Port _videoOut;
	Port _dataOut;
	IplImage **_buf;
	int _last, _idx1, _idx2;
	Semaphore _sem;
	CvSize _size;
	int _alive;

	// for PID filters
	int _firstDivPosUpdate;
	int _divPosCount;
	float _prevTTC;
	float _prevDiffx;
	float _prevDiffy;
	float _maxDiffxAcc;
	float _maxDiffyAcc;
	float _prevMinDiffx;
	float _prevMinDiffy;
	float _minDiffxAcc;
	float _minDiffyAcc;
	float _prev;
	float _prevTTCDiff;
	float _accTTCDiff;
	float _ttcErr[TTC_ARRAY_SIZE];
	CvPoint2D32f _locHist[TTC_ARRAY_SIZE];

	CvPoint _prevMaxDivPoint;
	CvPoint _prevMinDivPoint;
	int _flowUpdate;
	float _accTime;
	int _pointMatchCount;
	CvPoint _maxDivPoint, _minDivPoint;
	int _divCount;
	float _ratioAcc;
  	float	_prevRatio;
	int _ratioCount;

	CvPoint _windowPoints[MAX_POINTS];
	int _windowPointMatches[MAX_POINTS];

	CvRect _maxCompRect;
	CvPoint _center;

	double _angle;
	int _maxSize;
	int _maxIndex, _lastMaxIndex;
	int _interestPoint;


public:

    ImpactMapModule();
    ~ImpactMapModule();

	 //implemented Module functions
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

	 //implemented Thread functions
	 virtual bool threadInit();
	 virtual void run();
	 virtual void threadRelease();

	 // impact map functions
	 bool updateBuffer(IplImage *img);
	 void createDivergenceTemplate(IplImage *tmpMatchImg[2], 
						  								int width, int height);
	 void createTranslationTemplates(IplImage *tmpMatchImg[MAX_MATCH_WINS][2],
						 								 int width, int height);
	
	 void computeVels(CvPoint2D32f *points[2], IplImage *u_vels,
                IplImage *v_vels, IplImage *mags, char *status0, char *status,
                IplImage *vels_mask, int count);


	  CvRect supportWindowFilter(IplImage *img, IplImage *vels_mask,
                         IplImage *new_vels_mask, CvSize supportWindowSize,
                         int *divCount);

		
	  void findMaxDivergence(IplImage *u_vels, IplImage *v_vels,
                        IplImage *div_map, IplImage *u_tmp, IplImage *v_tmp,
                        double *max_div, double *av_div, IplImage *vels_mask);


		int classifyRegion(IplImage *u_vels, IplImage *v_vels,
                     IplImage *mag_map,
                     IplImage *tmpMatchImg[MAX_MATCH_WINS][2],
                     IplImage *vels_mask2, CvRect boundingBox, 
							float *ratio, float *tangentAngle, float div);


		float calcTangentAngle(IplImage *u_vels, IplImage *v_vels,
                       IplImage *u_res, IplImage *v_res, IplImage *vels_mask,
                       IplImage *tmpX[2], IplImage *tmpY[2], CvRect tmpRect,
                       float *mag);

		float computeApproachAngle(float new_ratio);

		float calcTimeToContact(float div, float kp, float ki, float kd);


		int computeImpactLocation(CvPoint2D32f *impactLocation, float ttc,
										  float approachAngle, float tangentAngle,
										  CvPoint imagePos);


		void makeImpactLocationImage(IplImage *impactImage, float ttc,
                                               CvPoint2D32f impactLocation);


		void makePyrVectorImage(IplImage *image, CvPoint2D32f* points[2],
							 int count, char *status, IplImage *vels_mask,
							 float tangentAngle, CvPoint center, CvRect boundingBox);

		void mergeImages(IplImage *src1, IplImage *src2, IplImage *out);


};

#endif
