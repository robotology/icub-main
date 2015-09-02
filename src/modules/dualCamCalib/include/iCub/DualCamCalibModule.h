// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright (C) 2015 Istituto Italiano di Tecnologia - iCub Facility
// Author: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __DUALCAMCALIBMODULE__
#define __DAULCAMCALIBMODULE__

// opencv
#include <cv.h>

// yarp
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Image.h>

// iCub
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/CalibToolFactory.h>
#include <iCub/ICalibTool.h>

class CamCalibModule : public yarp::os::RFModule {

private:

    bool verboseExecTime;
    enum align_type
    {
      ALIGN_WIDTH=0,
      ALIGN_HEIGHT=1
    } align;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageInLeft;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageInRight;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageOut;
    yarp::os::Port  configPort;

    ICalibTool *    calibToolLeft;
    ICalibTool *    calibToolRight;
    double requested_fps;
    double time_lastOut;
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb> calibratedImgLeft;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> calibratedImgRight;


public:

    CamCalibModule();
    ~CamCalibModule();
    
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    virtual double getPeriod();
};


#endif
