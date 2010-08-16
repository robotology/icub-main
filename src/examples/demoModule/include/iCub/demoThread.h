// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: $YOUR_NAME
 * email:   $YOUR_EMAIL
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
 * @file demoThread.h
 * @brief this file contains the definition of an additional thread that does the job for the main module.
 */

#ifndef _DEMO_THREAD_H_
#define _DEMO_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>

/**
 * theoretically, here you should document your class(es) and methods. 
 */
class demoThread : public yarp::os::Thread {
private:
   int      x, y;
   yarp::sig::PixelRgb rgbPixel;
   yarp::sig::ImageOf<yarp::sig::PixelRgb> *image;
  	    
   yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;
   yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;   
   int *thresholdValue; 

public:
   /**
    * contructor.
    * @param threshold threshold for image filtering.
    */
   demoThread(int *threshold);

   /**
    * destructor.
    */
   ~demoThread() {}

   // and so forth... don't forget to document all your methods.
   bool threadInit();     
   void threadRelease();
   void run(); 
   void onStop();
};

#endif  //_DEMO_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
