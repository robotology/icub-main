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
#include <opencv2/core/core_c.h>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/CalibToolFactory.h>
#include <iCub/ICalibTool.h>

/**
 *
 * Camera Calibration Port class
 *
 */
class CamCalibPort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >
{
private:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *portImgOut;
    ICalibTool     *calibTool;

    bool verbose;
    double t0;
    double currSat;

    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb> &yrpImgIn);

public:
    CamCalibPort();

    void setSaturation(double satVal);
    void setPointers(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *_portImgOut, ICalibTool *_calibTool);
    void setVerbose(const bool sw) { verbose=sw; }
};


/**
 *
 * Camera Calibration Module class
 *
 * \see icub_camcalib
 *
 */
class CamCalibModule : public yarp::os::RFModule {

private:

    CamCalibPort    _prtImgIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  _prtImgOut;
    yarp::os::Port  _configPort;

    ICalibTool *    _calibTool;

public:

    CamCalibModule();
    ~CamCalibModule();

    /** Passes config on to CalibTool */
    virtual bool configure(yarp::os::ResourceFinder &rf);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
    virtual bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    virtual double getPeriod();

};


#endif
