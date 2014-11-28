// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

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
 *       "/camCalib/conf";
 *     }
 *     "/camera" -> "/camCalib/in"
 *     "/camCalib/out" -> "/viewer/in"
 *     "/camCalib/conf" -> "rpc port"
 * \enddot
 *
 * \section lib_sec Libraries
 *
 * YARP 
 * OpenCV (version >= 2.0) 
 * spmap (icub library)
 * 
 * We have enabled changes of the image saturation directly from the rpc port (see port description). 
 * This has been done (temporary) in order to change the saturation for the bayer images.
 * The command are sent via rpc can be:
 * 
 * - sat 1.0  -  no changes in saturation 
 * - sat x where x is < 1.0  -  will decrease saturation until a gray image is obtained
 * - sat x where x is > 1.0  -  will increase saturation 
 * 
 * \section parameters_sec Parameters
 * 
 * Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - \c --from \c camcalib.ini \n 
 *   specifies the configuration file
 *
 * - \c --context \c cameraCalibration \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c --name \c camcalib \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 * For calibration configuration options see: PinholeCalibTool::configure
 * 
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *
 * <pre>
 * projection pinhole
 * drawCenterCross 1
 * w  320
 * h  240
 * fx 221.607
 * fy 221.689
 * cx 174.29
 * cy 130.528
 * k1 -0.397161
 * k2 0.180303
 * p1 4.08465e-005
 * p2 0.000456613
 *
 * </pre>
 * \section portsc_sec Ports Created
 *
 * Input port 
 *
 * - \c /camCalib/in \n
 *   Input image to calibrate (from camera grabber) (rgb)
 *
 * Output port
 *
 * - \c /camCalib/out \n
 *   Calibrated output image (rgb)
 *
 * Rpc port
 *
 * - \c /camCalib/conf \n
 *    Rpc port to change the output image saturation used
 *    primarely for the bayer images
 *
 * \section conf_file_sec Configuration Files
 *
 * \c camcalib.ini  in \c $ICUB_ROOT/app/cameraCalibration \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10, Debian Stable, squeeze and windows 
 *
 * \section example_sec Example Instantiation of the Module
 *
 * <tt>camCalib --name /icub/camcalib/left --context cameraCalibration --from icubEyes.ini --group CAMERA_CALIBRATION_LEFT</tt>
 *
 * More information on camera calibration:\n
 * OpenCV: http://opencvlibrary.sourceforge.net/CvReference#cv_3d\n
 * Matlab Toolbox: http://www.vision.caltech.edu/bouguetj/calib_doc/\n
 * 
 * \see CamCalibModule
 * \see CalibTool
 *
 * \author Lijin Aryananda, Jonas Ruesch
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
using namespace yarp::sig;


int main(int argc, char *argv[]) {
     
    CalibToolFactories& pool = CalibToolFactories::getPool();
    pool.add(new CalibToolFactoryOf<PinholeCalibTool>("pinhole"));
    pool.add(new CalibToolFactoryOf<SphericalCalibTool>("spherical"));

    Network yarp;
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("camCalib.ini");    //overridden by --from parameter
    rf.setDefaultContext("cameraCalibration");  //overridden by --context parameter
    rf.configure(argc, argv);
    CamCalibModule module;      
    return module.runModule(rf);
}
