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
#include <yarp/os/Stamp.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/CalibToolFactory.h>
#include <iCub/ICalibTool.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::contrib;
using namespace std;

namespace iCub {
    namespace contrib {
        class CamCalibPort;
        class CamCalibModule;
    }
}

using namespace iCub::contrib;


/**
 *
 * Camera Calibration Port class
 *
 */
class iCub::contrib::CamCalibPort : public BufferedPort<ImageOf<PixelRgb> >
{
private:
    yarp::os::Port *portImgOut;
    ICalibTool     *calibTool;

    bool verbose;
    double t0;

    virtual void onRead(ImageOf<PixelRgb> &yrpImgIn);

public:
    CamCalibPort();
    void setPointers(yarp::os::Port *_portImgOut, ICalibTool *_calibTool);
    void setVerbose(const bool sw) { verbose=sw; }
};


/**
 *
 * Camera Calibration Module class
 *
 * \see icub_camcalib
 *
 */
class iCub::contrib::CamCalibModule : public RFModule {

private:

    CamCalibPort   _prtImgIn;
	yarp::os::Port _prtImgOut;
    yarp::os::Port _configPort;

    ICalibTool *_calibTool;

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
