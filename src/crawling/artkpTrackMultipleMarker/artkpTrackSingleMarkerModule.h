// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 Alex Bernardino, Vislab, IST/ISR
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ARTKPTRACKSINGLEMARKERMODULE__
#define __ARTKPTRACKSINGLEMARKERMODULE__

 // std
#include <stdio.h>




// OpenCV
#include  <cv.h>

//object database
//#include "object.h"
//#include "coord_frames.h"

#include <map>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/Module.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/Polydriver.h>
#include <yarp/math/Math.h>


using namespace yarp;
using namespace yarp::math;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace yarp::dev;

// ARToolkitPlus
#include "ARToolKitPlus/TrackerSingleMarkerImpl.h"

/**
 *
 * ARToolkitPlus Track Single Marker Module
 *
 *
 */

#define GREEN_MARKER_ID 76
#define RED_MARKER_ID 3
#define EPSILON_POS 0.01
#define MIN_COUNTER_VALID 3
#define MAX_AGE 30

class MyLogger : public ARToolKitPlus::Logger
{
    void artLog(const char* nStr)
    {
        printf(nStr);
    }
};

class ARTKPTrackSingleMarkerModule : public Module 
{

private:
	ARToolKitPlus::TrackerSingleMarker *tracker;
	MyLogger      logger;
	bool useBCH;
    bool _reference_found;
    int _xsize, _ysize;
    int _thresh;
    int _object_num;
    ARFloat _object_center[2];
	ARFloat _object_width;
    //ARParam cparam;
    //ObjectData_T *_object;
    IplImage *_frame;
    double _reference[3][4];
	double _timestart;

    BufferedPort<ImageOf<PixelRgb> >       _imgPort;
	BufferedPort<ImageOf<PixelRgb> >       _viewPort;
	BufferedPort<yarp::sig::Vector>        _targetrelativePort;
    BufferedPort<Bottle>                   _configPort;
	BufferedPort<Bottle>                   _coordsPort;
	BufferedPort<Bottle>				   _targetPosPort;
	BufferedPort<Bottle>					threeDPosPort;

    void copytrans(double src[3][4], double dst[3][4]);
    void calibtrans(double src[3][4], double dst[3][4]);

	map<string, PolyDriver *>drivers;
	map<string, int>nbJoints;

	string robot;
	string eyeName;
	string ThreeDPosPort;

	struct PositionValid
	{
		sig::Vector position;
		int counter;
		int age;
		string color;
	};
	vector<PositionValid> positionsBuffer;

public:
    //cCoordFrames myFrames;

    ARTKPTrackSingleMarkerModule();
    ~ARTKPTrackSingleMarkerModule();
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
	sig::Vector VisionPositionToRootPosition(sig::Vector &visionPosition, const sig::Matrix &transformationMatrix);
	sig::Vector GetJointAngles(string partName);
	PolyDriver *CreatePolyDriver(string partName);
	sig::Matrix GetTransformationMatrix();
	void ValidatePosition( sig::Vector &position, string color);
	void SendValidPosition(void);

};


#endif
