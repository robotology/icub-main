// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ICUB_HISTTRACKERMODULE__
#define __ICUB_HISTTRACKERMODULE__

 // std
#include <stdio.h>
#include <iostream>
//#include <fstream>
//#include <sstream>
//#include <string>
//#include <vector>

// opencv
#include <cv.h>

// yarp
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/DetectorOutput.h>
#include <iCub/HistTracker.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
    namespace contrib {
        class HistTrackerModule;
    }
}

using namespace iCub::contrib;

/**
 *
 * ...
 *
 * \see icub_camcalibconf
 *
 */
class iCub::contrib::HistTrackerModule : public Module {

private:

    //BufferedPort<ImageOf<PixelRgb> >    _prtImg;

    /*IplImage                            *_ocvImgIn;
    IplImage                            *_ocvImgTmp1*/

	BufferedPort< DetectorOutput >	_detectorInput;

	BufferedPort<ImageOf<PixelRgb> >    _prtImgRgb;
	BufferedPort<ImageOf<PixelRgb> >	_prtImgTrackedRegionsRgb;

	IplImage *_currentImage;
	BufferedPort< DetectorOutput >	_trackerOutput;

	BufferedPort< Bottle > _emotionOutput;

	bool active;
	//bool needInit;
	list< HistTracker *> trackers;
	Bottle trackerSetup;
	bool anyTrackerActive;
	bool oneTargetShortTerm;
	/////temporary port added for control gaze demo
	yarp::os::BufferedPort< yarp::sig::Vector > _trackersignalOutput_port;
public:

    HistTrackerModule();
    ~HistTrackerModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
	void drawTrackedRegions(IplImage *currentImage, int lineThickness, CvScalar color);
};

#endif

