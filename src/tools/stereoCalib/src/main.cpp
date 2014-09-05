/* 
 * Copyright (C) 2011 RobotCub Consortium
 * Author: Sean Ryan Fanello
 * email:   sean.fanello@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
\defgroup icub_stereoCalib stereoCalib

@ingroup icub_tools

Calibrate the iCub's stereo system (both intrinsics and extrinsics).

Copyright (C) 2011 RobotCub Consortium
 
Author: Sean Ryan Fanello
 
Date: first release on 26/03/2011

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
The module performs the stereo calibration (both intrinsic and extrinsic parameters).
A chessboard pattern is required, for convenience, a chessboard calibration image is provided in the 
icub-main/app/cameraCalibration/data directory. In the config 
file calibrationConf.ini you should have the following group: 

\code
[STEREO_CALIBRATION_CONFIGURATION]
boardWidth W
boardHeight H
boardSize S
numberOfImages N
MonoCalib value
\endcode

This is the ONLY group used by the module. Other groups in your config file will be discarded. See below for the parameter description. Calibration results will be saved in the specified context (default is: $ICUB_ROOT/main/app/cameraCalibration/conf/outputCalib.ini). You should replace your old calibration file (e.g. icubEyes.ini) with this new one.

\note If you are using low resolution images (320x240) you should use a big chessboard pattern (i.e. with square side length of ~4cm). 
\note Make sure to show the pattern horizontally. 
\note Further information can be found in the <a 
      href="http://wiki.icub.org/wiki/Stereo_calibration">wiki</a>.
 
\section lib_sec Libraries 
YARP libraries and OpenCV

\section parameters_sec Parameters
--boardWidth \e numOfCornW 
- The parameter \e numOfCornW identifies the number of inner corners in the Width direction of the chess.
   
--boardHeight \e numOfCornH 
- The parameter \e numOfCornH identifies the number of inner corners in the Height direction of the chess.

--boardSize \e squareLength 
- The parameter \e squareLength identifies the length (in meters) of the squares in the chess (usually 0.03m).

--numberOfImages \e Num 
- The parameter \e Num identifies the number of pairs with the pattern needed before the stereo calibration (default 30).

--MonoCalib \e Val 
- The parameter \e Val identifies if the module has to run the stereo calibration (Val=0) or the mono calibration (Val=1). For the mono calibration connect only the camera that you want to calibrate.

\section portsc_sec Ports Created
- <i> /stereoCalib/cam/left:i </i> accepts the incoming images from the left eye. 
- <i> /stereoCalib/cam/right:i </i> accepts the incoming images from the right eye. 

- <i> /stereoCalib/cam/left:o </i> outputs the left eye image.
- <i> /stereoCalib/cam/right:o </i> outputs the right eye image.

- <i> /stereoCalib/cmd </i> for terminal commands comunication. 
 Recognized remote commands:
    - [start]: Starts the calibration procedure, you have to show the chessboard image in different positions and orientations. After N times (N specified in the config file, see above). the module will run the stereo calibration

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
outputCalib.ini 
 
\section tested_os_sec Tested OS
Windows and Linux.

\author Sean Ryan Fanello
*/ 

#include "stereoCalibModule.h"
#include <yarp/dev/Drivers.h>

YARP_DECLARE_DEVICES(icubmod)


int main(int argc, char * argv[])
{
   YARP_REGISTER_DEVICES(icubmod)
   Network yarp;
   stereoCalibModule stereoModule; 
   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("icubEyes.ini"); 
   rf.setDefaultContext("cameraCalibration");
   rf.configure(argc, argv);

   stereoModule.runModule(rf);
    return 1;
}
