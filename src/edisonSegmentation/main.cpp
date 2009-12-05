// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/** 
@ingroup icub_module

\defgroup icub_edisonSegmentation edisonSegmentation

Wrapper to the The EDISON system: a low-level vision tool that performs confidence 
based edge detection and synergistic image segmentation.

\section intro_sec Description

This module wraps around some function os the EDISON system from the 
Robust Image Understanding Laboratory at Rutgers University. 

The purpose of the module is to obtain good segmentation of color images.
The image is split into reagions corresponding to uniformly colored patches.


Details on the employed algorithms is provided in the following paper:
[1] D. Comanicu, P. Meer: "Mean shift: A robust approach toward feature space analysis".
    IEEE Trans. Pattern Anal. Machine Intell., May 2002.

[2] P. Meer, B. Georgescu: "Edge detection with embedded confidence". IEEE Trans. Pattern Anal.
    Machine Intell., 28, 2001.

[3] C. Christoudias, B. Georgescu, P. Meer: "Synergism in low level vision". 16th International
    Conference of Pattern Recognition, Track 1 - Computer Vision and Robotics, Quebec City,
    Canada, August 2001.

The edison source files are provided in the subfolder edison_src. 
Some changes to these files had to be made in order to obtain the desired functionality.

\section lib_sec Libraries
YARP libraries.
OpenCV library.

\section parameters_sec Parameters
--from edisonConfig.ini (module configuration parameters, search in default context folder locations)

\section portsa_sec Ports Accessed
Port with raw RGB image.

\section portsc_sec Ports Created

/conf
for module configuration

/rawimg:i
receive the original RGB image to segment

/rawimg:o
output the original RGB image 

/labelimg:o 
segmented image with the labels (PixelInt)

/viewimg:o
segmented image with the colors models for each region (good to visualize) 

\section in_files_sec Input Data Files
None

\section out_data_sec Output Data Files
None

\section conf_file_sec Configuration Files

Requires a file containing paramenter for the image segmentation algorithm.

The file may contain the following paramenters:

width, height - Dimension of the images to be processed. This may differ from the dimension of the input images. 
Forcing a smaller dimension will save computation power at the cost of resolution. 
Values larger that the input image dimension will be discarded.
These parameters do no influence the dimension of the output. This is always the same as the input dimension.
Default: the size of the original image.

sigmaS - The spatial bandwidth (neighborhood in the pixel domain).
Default: 7

sigmaR - The color bandwidth (neighborhood in the color color domain).
Default: 6.5

minregion - The minimal area for segmented regions (in pixels)
Default: 20.0

gradWinRad - The radius of the window used for the computation of the gradient and confidence map (positive integer).
Default: 2.0

threshold - edge strength threshold (must be in the interval [0,1])
Default: 0.3

mixture - mixture parameter (must be in the interval [0,1])
Default: 0.2

speedup - accelerate computation by doing some approximations.
Possible values 0 (NO_SPEEDUP), 1 (MED_SPEEDUP), 2 (HIGH_SPEEDUP)
Default: MED_SPEEDUP


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module

yarp server
edisonSegmentation.exe --from configFile.ini
yarpdev --device opencv_grabber --movie H:\DataSets\testImages2009_07_21\segm_test_icub.avi --loop --framerate 0.1
yarpview /raw
yarpview /view
yarp connect /grabber /edisonSegm/rawimg:i
yarp connect /edisonSegm/rawimg:o /raw
yarp connect /edisonSegm/viewimg:o /view

\author Alexandre Bernardino

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.


This file can be edited at src/edisonSegmentation/main.cpp.
*/ 


// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
using namespace yarp::os;

#include "EdisonSegmModule.h"

int main(int argc, char *argv[]) {

    Network yarp;
    EdisonSegmModule module;
    module.setName("/edisonSegm"); // set default name of module
    return module.runModule(argc,argv);
}
