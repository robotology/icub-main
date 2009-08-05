// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008
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

#ifndef __VISUAL_ROTATION_MODULE_H__
#define __VISUAL_ROTATION_MODULE_H__

#include "visualRotation.h"

/**
 * @ingroup icub_module
 * \defgroup icub_visualRotation visualRotation
 * Estimates the image rotations, in other words computes the rotation
 * of the camera between to consecutive runs
 * 
 * \section intro_sec Description
 * This module takes an image, finds good features to track, and then looks
 * in the next image where those features are. It projects the features in 3d
 * and then computes the rotation that best accounts for the set of features
 * in the first image and their images in the second images. This is done
 * using the algorithm described in Horn (1987). 
 *
 * \section lib_sec Libraries
 * yarp_os and openCV 0.9.7 or 1.0.0. Depending on the version of opencv a
 * correct -DOPENCV_0_9_7 should be added in the CMakeLists.txt
 *
 * \section parameters_sec Parameters
 * 
 * no mandatory parameters. Optional parameters are
 * - --name <basename>: basename for the ports
 * - --nb_points <nb> : number of features to track. Default is 500
 *
 *\section portsa_sec Ports Accessed
 * 
 * The module connects to /icub/cam/right of the icubInterface. 
 * If you want to connect to the left eye, it is possible to change it 
 * in the code (stereoVisualTracker/src/visualRotationModule.cpp). There should
 * not be any problem
 *
 *\section portsc_sec Ports Created
 *
 * Input ports:
 *
 * <basename>/image:i gets the image. A time stamp is expected
 *
 * Ouput ports:
 *
 * <basename>/visualRotation:o Output the estimated rotation as a 3d vector
 * the firsts three (complex) part of the quaternion and the time stamps
 * corresponding to the two images : qx qy qz t0 t1
 *
 * \section in_files_sec Input Data Files
 * none
 * 
 * \section out_data_sec Output Data Files 
 * none
 *
 * \section conf_file_sec Configuration Files
 * parameters can be passed through a file with the --file option
 *
 * \section tested_os_sec Tested OS
 * Linux, partially on windows as well
 *
 *\section example_sec Example Instantiation of the Module 
 *
 * \code
 *  ./visualRotation --name visualFlow
 * \endcode 
 *
 * \author Micha Hersch for the yarp compatibility, Basilio Noris for the core
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



class VisualRotationModule : public yarp::os::Module, public VisualRotation{

protected:
    VisionServer port;
    YarpFrameGrabber yGrabber;

public:
    
    bool updateModule(){return (bool) Loop();};
    bool open(Searchable& config);
    bool close(){Free();port.Stop();return true;};
    void sendOutput(double t1=0);
    virtual int Loop();
    virtual bool InitGrabber();
    void Init(int nb_points=400);
    int ExecuteCommand(int key);
};


#endif
