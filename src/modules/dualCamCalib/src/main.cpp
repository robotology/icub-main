// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright (C) 2015 Istituto Italiano di Tecnologia - iCub Facility
// Author: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

// ***** THIS MODULE IS EXPERIMENTAL *****

// yarp
#include <yarp/os/Network.h>

// iCub
#include <iCub/CalibToolFactory.h>
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/DualCamCalibModule.h>

// OpenCV
#include <opencv2/core/core_c.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char *argv[]) {
     
    CalibToolFactories& pool = CalibToolFactories::getPool();
    pool.add(new CalibToolFactoryOf<PinholeCalibTool>("pinhole"));
    pool.add(new CalibToolFactoryOf<SphericalCalibTool>("spherical"));

    Network yarp;
    ResourceFinder rf;
    rf.setDefaultConfigFile("camCalib.ini");    //overridden by --from parameter
    rf.setDefaultContext("cameraCalibration");  //overridden by --context parameter
    rf.configure(argc, argv);
    CamCalibModule module;      
    return module.runModule(rf);
}
