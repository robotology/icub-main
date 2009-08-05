// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-



/*

 * Copyright (C) 2008 Alex Bernardino, Vislab, IST/ISR

 * CopyPolicy: Released under the terms of the GNU GPL v2.0.

 *

 */



#ifndef __ARMARKERDETECTORMODULE__

#define __ARMARKERDETECTORMODULE__



 // std

#include <stdio.h>





// ARToolkit

#include <AR/ar.h>



// yarp

#include <yarp/String.h>

#include <yarp/os/all.h>

#include <yarp/sig/all.h>

#include <yarp/os/Module.h>



using namespace yarp;

using namespace yarp::os;

using namespace yarp::sig;



using namespace std;



#include "ARToolKitTracker.h"

// OpenCV

#include  <cv.h>



//object database

#include "object.h"









/**

 *

 * ARToolkit Marker Detector Module

 *

 *

 */



class ARMarkerDetectorModule : public Module {



private:



	ARToolKitTracker tracker;

    BufferedPort<ImageOf<PixelRgb> >        _imgPort;
    BufferedPort<yarp::os::Bottle>          _configPort;
    BufferedPort<yarp::os::Bottle>          _outPort;

	double timelastround;

public:



    ARMarkerDetectorModule();

    ~ARMarkerDetectorModule();

    virtual bool open(Searchable& config);

    virtual bool close();

    virtual bool interruptModule();

    virtual bool updateModule();

};


#endif

