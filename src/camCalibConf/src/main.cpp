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
#include <iCub/CamCalibConfModule.h>

using namespace std;
using namespace yarp::os;

using namespace iCub::contrib;

/**
 * @ingroup icub_module
 *
 * \defgroup icub_camcalibconf camcalibconf
 *
 * Calculates camera calibration parameters for the camcalib module.
 *
 * The module detects inner corners of a presented chessboard pattern.
 * In order to calculate the camera parameters a specified number (~25)
 * of corner detection cycles for different pattern positions/orientations
 * must be completed. The user is ask to press 'g' (grab) for every pattern
 * position he wants to use.\n
 * After the corner positions are acquired the module calculates the calibration
 * parameters and writes or updates the specified configuration file.\n
 * To observe the corner detection process the image port of this module is
 * supposed to be connected to a yarp image viewer.\n
 * Camcalibconf expects a configuration file with section 
 * [CAMERA_CALIBRATION_CONFIGURATION] where details about the presented chessboard
 * pattern are specified (size of a single chessboard square in mm, number of inner
 * corners in x and y direction, absolute configuraiton file name, number of 
 * detection cycles you wish to run). See terminal output for exact syntax.
 *
 * \dot
 * digraph module_camcalib_example {
 *     graph [ rankdir = "LR" ];
 *     edge [arrowhead="open", style="solid"];
 *     node [shape=ellipse];
 *     subgraph cluster_camcalib {
 *      color = "black"; style = "solid";
 *      label = "camcalibconf module";
 *       "/camcalibconf/image";
 *     }
 *     "/camera" -> "/camcalibconf/image"
 *     "/camcalibconf/image" -> "/yarpview/i:img"
 * \enddot
 *
 * \see iCub::contrib::CamCalibConfModule
 *
 * \author Jonas Ruesch
 *
 */


int main(int argc, char *argv[]) {

	Network yarp;
	ResourceFinder rf;
	rf.setVerbose(true);
	rf.setDefaultConfigFile("camCalibConf.ini"); //overridden by --from parameter
	rf.setDefaultContext("camCalibConf"); //overridden by --context parameter
	rf.configure("ICUB_ROOT", argc, argv);
    CamCalibConfModule module;
    module.setName("/camCalibConf");
	module.attachTerminal();
    return module.runModule(rf);   
}
