// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ATTENTIONSELECTIONMODULE__
#define __ATTENTIONSELECTIONMODULE__

#ifndef M_PI_2
#define M_PI_2	((float)(asin(1.0)))
#endif
#ifndef M_PI
#define M_PI	((float)(2*M_PI_2))
#endif

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/AttentionSelectionInterfaces.h>
#include <iCub/RemoteEgoSphere.h> // to access egosphere remotely

namespace iCub {
    namespace contrib {
        class AttentionSelectionModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * Attention selection Module class
 *
 * \see icub_attentionselection
 *
 */
class iCub::contrib::AttentionSelectionModule : public RFModule {

private:

    // ports
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >       _prtImgFloatSalienceIn;
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> >		 _prtImgFloatSelectionOut;
    yarp::os::BufferedPort<yarp::sig::VectorOf<double> >          _prtVctPosOut;  // sending [azimuth, elevation, depth] to gaze control
    yarp::os::BufferedPort<yarp::sig::VectorOf<double> >          _trackersignalOutput_port;  
    yarp::os::Port _configPort;
    yarp::os::BufferedPort<yarp::sig::Vector> _prtVctTrackerIn;
	yarp::os::BufferedPort<yarp::os::Bottle> _prtBotGazeStateIn;

    void getPeak(yarp::sig::ImageOf<yarp::sig::PixelFloat> &img, int &i, int &j, float &v);

	bool _inhibitOutput; // inhibit motor output
    double _hViewAngle; // horizontal view field (degrees)
    double _vViewAngle; // vertical view field (degrees)
    float _thresholdDifference;
    float _thresholdDistanceFraction;
    float _thresholdDistance;
    double _timeStart;
    double _timeDelay;
    int _resX;
    int _resY;
    int _limitAzimuthRightPix;
    int _limitAzimuthLeftPix;
    int _limitElevationTopPix;
    int _limitElevationBottomPix;

    double _maxSalience;
    int _maxSalienceX;
    int _maxSalienceY;
    int _gazeX;
    int _gazeY;
    int _gazeXOld;
    int _gazeYOld;
	double _gazeAz, _gazeEl;
	double _gazeAzOld, _gazeElOld;
	int _intGazeState;
	double _dblGazeAz;
	double _dblGazeEl;
	double _trackerX;
	double _trackerY;
	double _trackerSalience;
	double _w;
	double _w2;
	double _h;
	double _h2;

    long _saccadeIndex;

    bool _verbose;

	// application
	int _intFPS;
	int _intFPSAchieved;
	int _intPrintFPSAfterNumFrames;
	int _intFC; // frame counter
	double _dblTPF; // time per frame (ms)
	double _dblTPFAchieved; // actual time per frame 
	double _dblStartTime;

    float MAX_SALIENCE;
	float TRACKER_PRIORITY; // a small value making sure an object tracked is most salient at the time of detection, set to zero if a detected object should have MAX_SALIENCE at detection time

	RemoteEgoSphere _remoteEgosphere;

    yarp::os::Semaphore _mutex;

public:

    AttentionSelectionModule();
    virtual ~AttentionSelectionModule();
    
    /** Passes config on to iCub::contrib::CalibTool */
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const Bottle &command,Bottle &reply);

};


#endif
