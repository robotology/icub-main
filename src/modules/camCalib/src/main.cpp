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
 *
 * - \c --group \c CAMERA_CALIBRATION_LEFT \n
 *   selects the configuration group to load and, for fisheye rectification, the camera stream
 *   to rectify (\c CAMERA_CALIBRATION_LEFT or \c CAMERA_CALIBRATION_RIGHT)
 *
 * \section pinhole_sec Pinhole Lens Calibration
 *
 * Set \c projection \c pinhole in the selected camera calibration group to undistort a
 * standard pinhole camera image. The following parameters are required in that group:
 *
 * - \c drawCenterCross: \c 0 to disable or \c 1 to draw a cross at the principal point
 * - \c w, \c h: image size used when deriving the calibration
 * - \c fx, \c fy: focal lengths
 * - \c cx, \c cy: principal point
 * - \c k1, \c k2: radial distortion coefficients
 * - \c p1, \c p2: tangential distortion coefficients
 *
 * The intrinsic matrix is automatically rescaled when the runtime image size differs from
 * \c w and \c h. The distortion coefficients remain unchanged.
 *
 * Example pinhole configuration:
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
 *
 * \section fisheye_sec Fisheye Lens Stereo Rectification
 *
 * The module also supports \c projection \c fisheye for stereo fisheye lenses. In this mode,
 * \c camCalib rectifies one camera stream at a time using the calibration of both cameras plus
 * the stereo extrinsic transform stored in the same configuration file.
 *
 * Fisheye requirements:
 *
 * - \c projection \c fisheye in the selected camera group
 * - \c --group set to either \c CAMERA_CALIBRATION_LEFT or \c CAMERA_CALIBRATION_RIGHT
 * - both \c [CAMERA_CALIBRATION_LEFT] and \c [CAMERA_CALIBRATION_RIGHT] groups available
 * - a \c [STEREO_DISPARITY] group containing \c HN as a 4x4 homogeneous transform
 *
 * Each fisheye camera group must contain:
 *
 * - \c w, \c h
 * - \c fx, \c fy, \c cx, \c cy
 * - \c k1, \c k2, \c k3, \c k4
 *
 * Fisheye rectification settings:
 *
 * - \c balance: value in [0,1] controlling the crop level after rectification. It can be
 *   specified globally or in the selected camera group; the global value takes precedence.
 * - \c fovScale: value greater than 0 that scales the output field of view. It can be
 *   specified globally or in the selected camera group; the global value takes precedence.
 * - \c rectifyAlpha: global fallback value for \c balance when \c balance is not set.
 * - \c drawEpipolars: global boolean that overlays horizontal epipolar lines on the
 *   rectified image.
 * - \c epipolarLineStep: global pixel spacing between epipolar lines; values less than
 *   one are treated as one.
 *
 * Fisheye rectification automatically rescales the intrinsic matrices if the runtime image size
 * differs from the calibration size.
 *
 * Example fisheye configuration:
 *
 * <pre>
 * [CAMERA_CALIBRATION_LEFT]
 * projection fisheye
 * w 640
 * h 480
 * fx 320.0
 * fy 320.0
 * cx 320.0
 * cy 240.0
 * k1 -0.01
 * k2 0.001
 * k3 0.0
 * k4 0.0
 * balance 0.0
 * fovScale 1.0
 *
 * [CAMERA_CALIBRATION_RIGHT]
 * projection fisheye
 * w 640
 * h 480
 * fx 320.0
 * fy 320.0
 * cx 320.0
 * cy 240.0
 * k1 -0.01
 * k2 0.001
 * k3 0.0
 * k4 0.0
 *
 * [STEREO_DISPARITY]
 * HN (-1 0 0 0 0 -1 0 0 0 0 1 0 0.05 0 0 1)
 * </pre>
 *
 * Example invocation:
 *
 * <tt>camCalib --name /icub/camcalib/left --context cameraCalibration --from icubEyes.ini --group CAMERA_CALIBRATION_LEFT</tt>
 *
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

// iCub
#include <iCub/CalibToolFactory.h>
#include <iCub/PinholeCalibTool.h>
#include <iCub/SphericalCalibTool.h>
#include <iCub/FisheyeCalibTool.h>
#include <iCub/CamCalibModule.h>

// OpenCV
#include <opencv2/core/core_c.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char *argv[]) {
     
    CalibToolFactories& pool = CalibToolFactories::getPool();
    pool.add(new CalibToolFactoryOf<PinholeCalibTool>("pinhole"));
    pool.add(new CalibToolFactoryOf<SphericalCalibTool>("spherical"));
    pool.add(new CalibToolFactoryOf<FisheyeCalibTool>("fisheye"));

    Network yarp;
    ResourceFinder rf;
    rf.setDefaultConfigFile("camCalib.ini");    //overridden by --from parameter
    rf.setDefaultContext("cameraCalibration");  //overridden by --context parameter
    rf.configure(argc, argv);
    CamCalibModule module;      
    return module.runModule(rf);
}
