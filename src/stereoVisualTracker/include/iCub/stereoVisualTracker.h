// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   <firstname.secondname>@robotcub.org
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





#ifndef __STEREO_VISUAL_TRACKER_H__
#define __STEREO_VISUAL_TRACKER_H__

#include "iCub/stereovision.h"
#include "iCub/yarpFrameGrabber.h"


/**
 * @ingroup icub_module
 *
 * \defgroup icub_stereoVisualTracker  stereoVisualTracker  
 *
 * This module performs color-based 3d tracking with stereovision.
 *
 * \section intro_sec Description
 * This module performs color-based 3d tracking with stereovision.
 *
 * Color is selected by
 * selecting a region of that color on the opencv windows. User interface is
 * provided by keyboard commands. Up to 3 objects can be tracked.
 *
 * Calibration is mandatory and done with a checkerboard. The calibration
 * files are respectively params/<dir>/params1 and  params/<dir>/params2 
 * for the first and the second camera, where <dir> may be given as argument in the
 * command line (see below).  
 * If you want to track more objects
 * recompile redefine the MAX_OBJECTS and recompile. 
 * Commands must be given while an opncv window is active. Commands include: 
 * - 't' enable/disable 2d tracking
 * - 'l' enable/disable 3d localization
 * - 'a' add an object to be tracked
 * - 'u' start/stop sending the 3d location through yarp
 * - 'k' enable/disable kalman filtering for all objects
 * - 'n' switch to next object
 * - 'M' raise measurement noise of the kalman filter for the current object
 * - 'm' lower measurement noise of the kalman filter for the current object
 * - 'P' raise process noise of the kalman filter for the current object
 * - 'p' lower process noise of the kalman filter for the current object
 * - 'r' stop/resume tracking the current object (only one of them can be stopped)
 * - 'c' calibrate the camera intrinsics parameters (distortion coeffs)
 * - '3' calibrate the camera extrinsics parameters (for position and orientation)
 * - 'w' switch color mode (YCbCr vs )
 * - 'd' show effect of undistortion
 * - 'q' exit application
 * - 's' save color files
 * - 'g' load color files
 *
 * For selecting the color, draw an imaginary square on the window on the color you want to
 * select, with a left click. For deslecting it, right-click on the window. Each window has
 * it own colors i.e. you must select the color in both windows (as cameras may have be different)
 *.
 * \section parameters_sec Parameters
 * - -- width : <width > width chessboard (number of squares)
 * - -- height: <height> height of chessboard (number of squares, must be smaller and <width>)
 * - -- size: <size> size a the squares of the chessboard (in the units you want the tracking 
 *    to be done).
 * 
 * Those three parametes are mandatory (even if you already have calibration files, but in that
 * case they will not be used unless you recalibrate ('c' or '3' command). 
 *
 * - --file <config_file>
 * - --name <port_base_name>
 * - --directory <dir> directory where to save and load parameters, must exist 
 * within the "params" directory.
 *
 * \section portsa_sec Ports Accessed
 * no port accessed directly in the code. But to make it work you have to connect
 * to some port sending images, typically /icub/cam/right and /icub/cam/left. See the
 * lauch script for an example
 * 
 * \section portsc_sec Ports Created
 * input ports:
 * - <port_base_name>/image0:i to get the first image
 * - <port_base_name>/image1:i to get the second image
 * output ports:
 * - <port_base_name>/position<j>:o to send the tracked position, x,y,z in calibration coordinates,
 * where <j> is the index of the object that is tracked. <j> if only one object is tracked, <j> is
 * 0 and only one port is created. If there are more (because you sent the 'a' command) more 
 * ports are created. Ports are created/destroyed when you send the 'u' command. 
 *
 * \section tested_os_sec Tested OS
 * Developped and tested on linux, partially on windows as well.
 *
 * \section example_sec Example Instantiation of the Module
 * To launch it in a new terminal:
 * \code
 * xterm -T "stereovision" -e "./stereoVisualTracker --name /micha --width 6 --height 8 --size 30" & 
 * sleep 1
 * yarp connect /icub/cam/right /micha/image0:i
 * sleep 1
 * yarp connect /icub/cam/left /micha/image1:i
 * \endcode
 * 
 *
 *\author Micha Hersch
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited in src/stereoVisualTracker/include/iCub/stereoVisualTracker.h
 */

 
class stereoVisualTracker : public yarp::os::Module, public Stereovision {

 protected:

    virtual void SendOutput(VisionServer *port, double *position); 
    double mPeriod;
 public:
    /**
     * grabs the two frames and performs the processing
     */
    bool updateModule();
    bool open(Searchable& config);
    bool close();
    virtual void InitCameras(){}
    /**
     * initializes the yarpFrameGrabber. Connection must be done manually
     */
    
    virtual bool InitGrabber(int i);
    
    virtual double 	getPeriod ();
};

#endif
