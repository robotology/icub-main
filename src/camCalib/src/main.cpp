// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

// iCub
#include <iCub/CalibToolFactory.h>
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/CamCalibModule.h>

// OpenCV
#include <cv.h>

using namespace std;
using namespace yarp::os;

using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_camCalib camCalib
 *
 * Camera image calibration module.
 *
 * \dot
 * digraph module_camcalib_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_camcalib {
 *      color = "black"; style = "solid";
 *      label = "camcalib module";
 *       "/camCalib/in";
 *       "/camCalib/out";
 *     }
 *     "/camera" -> "/camCalib/in"
 *     "/camCalib/out" -> "/viewer/in"
 * \enddot
 *
 * For calibration configuration options see: iCub::contrib::PinholeCalibTool::configure
 * 
 * More information on camera calibration:\n
 * OpenCV: http://opencvlibrary.sourceforge.net/CvReference#cv_3d\n
 * Matlab Toolbox: http://www.vision.caltech.edu/bouguetj/calib_doc/\n
 * 
 * \see iCub::contrib::CamCalibModule
 * \see iCub::contrib::CalibTool
 *
 * \author Lijin Aryananda, Jonas Ruesch
 *
 */


int main(int argc, char *argv[]) {

     // prepare CalibTools
    CalibToolFactories& pool = CalibToolFactories::getPool();
    pool.add(new CalibToolFactoryOf<PinholeCalibTool>("pinhole"));
    pool.add(new CalibToolFactoryOf<SphericalCalibTool>("spherical"));

    Network yarp;
    CamCalibModule module;
    module.setName("/camcalib"); // set default name of module
    return module.runModule(argc,argv);
}
