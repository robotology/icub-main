// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_MOBILEEYEMODULE_INC
#define ICUB_MOBILEEYEMODULE_INC

// std
//#include <deque>
#include <stdio.h>
#include <float.h>

// cv
//#include <cv.h>

// yarp
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>

// iCub
//#include <iCub/MobileEye.h>

//namespace iCub {
//    namespace contrib {
//        class MobileEyeModule;
//    }
//}

using namespace yarp::os;
using namespace yarp::sig;
//using namespace iCub::contrib;

/**
 *
 * 
 *
 * \see 
 *
 * \author 
 *
 */
//class iCub::contrib::MobileEyeModule : public Module,
//                                      public IMobileEyeModuleControls {

class MobileEyeModule : public Module {
private:
    BufferedPort<ImageOf<PixelRgb> > imgInPort;
    BufferedPort<ImageOf<PixelRgb> > imgOutPort;
    //BufferedPort<ImageOf<PixelFloat> > filteredPort;
	BufferedPort<Bottle> gazePort; //For streaming saliency peak coordinates (Alex, 31/05/08)
    //BufferedPort<Bottle> configPort;
    yarp::os::Semaphore mutex;
    bool drawMobileEyeGaze;
    int oldSizeX, oldSizeY;
    bool needInit;
    //virtual void resizeBufferedImages(int w, int h);
    //virtual void getPeak(ImageOf<PixelFloat> &img, int &i, int &j, float &v);
    
public:
	MobileEyeModule();
	virtual ~MobileEyeModule();
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();

    /**
     * The respond method implements the IMobileEyeModuleControls interface
     * and the IMobileEyeControls for the first/top filter.\n
     * If a 'fn' filtername tag is found and the value (filtername) matches the
     * the active filter the command bottle is passed on to that filters
     * respond method (which might pass it on further to child filters).
     * The value for the 'fn' command is supposed to be a string with format
     * "topFilterA.childFilterXY.childFilterYZ"
     */
    // virtual bool respond(const Bottle &command,Bottle &reply);

    virtual bool updateModule();

};

#endif

