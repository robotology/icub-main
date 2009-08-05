// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_SALIENCEMODULE_INC
#define ICUB_SALIENCEMODULE_INC

// std
#include <deque>
#include <stdio.h>
#include <float.h>

// cv
#include <cv.h>

// yarp
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/os/ResourceFinder.h>

// iCub
#include <iCub/Salience.h>
#include <iCub/SalienceFactory.h>
#include <iCub/SalienceInterfaces.h>
#include <iCub/IOR.h>
#include <iCub/Framerate.h>

namespace iCub {
    namespace contrib {
        class SalienceModule;
    }
}

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::contrib;

/**
 *
 * Helper for creating and networking salience filters.
 *
 * \see icub_salience
 *
 * \author Paul Fitzpatrick
 *
 */
class iCub::contrib::SalienceModule : public Module,
                                      public ISalienceModuleControls {
private:
    BufferedPort<ImageOf<PixelRgb> > imgPort;
    BufferedPort<ImageOf<PixelFloat> > filteredPort;
	BufferedPort<Bottle> peakPort; //For streaming saliency peak coordinates (Alex, 31/05/08)
    BufferedPort<Bottle> configPort;
    Salience *filter;
    yarp::os::Semaphore mutex;
    int numBlurPasses;
    bool drawSaliencePeak;
    bool activateIOR; // inhibition of return
    double thresholdSalience;
    int oldSizeX, oldSizeY;
    bool needInit;
    IOR ior;
	Framerate _framerate;
    //virtual void resizeBufferedImages(int w, int h);
    virtual void drawRgbFromFloat(ImageOf<PixelFloat> &imgFloat, 
                                      ImageOf<PixelRgb> &imgRgb, 
                                      float scale);
    virtual void getPeak(ImageOf<PixelFloat> &img, int &i, int &j, float &v);
    
public:
	SalienceModule();
	virtual ~SalienceModule();
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    /**
     * The respond method implements the ISalienceModuleControls interface
     * and the ISalienceControls for the first/top filter.\n
     * If a 'fn' filtername tag is found and the value (filtername) matches the
     * the active filter the command bottle is passed on to that filters
     * respond method (which might pass it on further to child filters).
     * The value for the 'fn' command is supposed to be a string with format
     * "topFilterA.childFilterXY.childFilterYZ"
     */
    virtual bool respond(const Bottle &command,Bottle &reply);
    virtual bool updateModule();

    // ISalienceModuleControls
    double getSalienceThreshold();
    bool setSalienceThreshold(double thr);
    int getNumBlurPasses();
    bool setNumBlurPasses(int num);
    int getTemporalBlur();
    bool setTemporalBlur(int size);
};

#endif

