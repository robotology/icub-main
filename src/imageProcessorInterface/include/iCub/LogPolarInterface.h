// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _LOGPOLARINTERFACE_H_
#define _LOGPOLARINTERFACE_H_


#include <iCub/RC_DIST_FB_logpolar_mapper.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/*
  This example adds a moving circle to an image stream.
  It is the same as image_process.cpp, except it is built
  as a Yarp "Module".  This makes it a bit cleaner to start/stop.
  

  Suppose we have an image source on port /source such as:
    yarpdev --device test_grabber --name /source --mode line --framerate 10

  And suppose we have an image viewer on port /view:
    yarpview --name /view

  Then we can hook this program up in between as follows:
    ./image_process --name /worker
    yarp connect /source /worker
    yarp connect /worker /view

  You should see the normal scrolling line of test_grabber, with a moving
  circle overlaid.

 */

class LogPolarInterface(){

}

#endif //_LOGPOLARINTERFACE_H