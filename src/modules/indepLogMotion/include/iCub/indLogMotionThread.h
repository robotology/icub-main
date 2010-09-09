// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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
 * @file indLogMotionThread.h
 * @brief Definition of a thread that receive a logimage and applies independent motion detector on it. It is istantited by
 * independet log-motion module (see indLogMotionModule.h).
 */

#ifndef _INDLOG_MOTION_THREAD_H_
#define _INDLOG_MOTION_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <iostream>

class indLogMotionThread : public yarp::os::Thread {
private:
    int psb;
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        //original dimension of the input and output images
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *tmp;           //temporarily image used to upload the relative buffer
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImage;    //input image reference left
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *outImage;      //pointer to the output image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inPort; //a port for the inputLeftImage 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > interInPort; //a port for the input of independent motion detector
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > interOutPort; //a port for the output of independent motion detector
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPort; //port whre the output is sent
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized

public:
    /**
    * default constructor
    */
    indLogMotionThread();

    /**
     * destructor
     */
    ~indLogMotionThread();

    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

};

#endif  //_INDLOG_MOTION_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

