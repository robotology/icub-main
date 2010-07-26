// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _DEMO_THREAD_H_
#define _DEMO_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>


class demoThread : public yarp::os::Thread {
private:

   /* class variables */

   int      x, y;
   yarp::sig::PixelRgb rgbPixel;
   yarp::sig::ImageOf<yarp::sig::PixelRgb> *image;
  	    
   yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;
   yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;   
   int *thresholdValue; 
   

public:

   /* class methods */

   demoThread(int *threshold );
   bool threadInit();     
   void threadRelease();
   void run(); 
   void onStop();
};

#endif  //_DEMO_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
