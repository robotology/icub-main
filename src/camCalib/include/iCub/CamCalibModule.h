// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __CAMCALIBMODULE__
#define __CAMCALIBMODULE__

 // std
#include <stdio.h>

// opencv
#include <cv.h>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/CalibToolFactory.h>
#include <iCub/ICalibTool.h>
#include <iCub/Framerate.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::contrib;
using namespace std;

namespace iCub {
    namespace contrib {
        class CamCalibModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * Camera Calibration Module class
 *
 * \see icub_camcalib
 *
 */
class iCub::contrib::CamCalibModule : public RFModule {

private:

    BufferedPort<ImageOf<PixelRgb> >    _prtImgIn;
	BufferedPort<ImageOf<PixelRgb> >    _prtImgOut;
    yarp::os::Port _configPort;

    ICalibTool                           *_calibTool;

    Semaphore                           _semaphore;

	// framerate
    int _intFPS;
	int _intFPSAchieved;
	int _intPrintFPSAfterNumFrames;
	int _intFC; // frame counter
	double _dblTPF; // time per frame (ms)
	double _dblTPFAchieved; // actual time per frame 
	double _dblStartTime;

public:

    CamCalibModule();
    ~CamCalibModule();
    
    /** Passes config on to iCub::contrib::CalibTool */
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual double getPeriod();

};


#endif
