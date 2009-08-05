// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __ICUB_FACEEYEDETECTORMODULE__
#define __ICUB_FACEEYEDETECTORMODULE__

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

#include <iCub/FaceEyeDetector.h>
#include <iCub/DetectorOutput.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

namespace iCub {
    namespace contrib {
        class FaceEyeDetectorModule;
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
class iCub::contrib::FaceEyeDetectorModule : public Module {

private:

    BufferedPort<ImageOf<PixelRgb> >    _prtImgRgb;
	BufferedPort<ImageOf<PixelRgb> >	_prtImgDetectedFacesRgb;

	IplImage *_currentImage;
	BufferedPort< DetectorOutput >	_detectorOutput;

    /*IplImage                            *_ocvImgIn;
    IplImage                            *_ocvImgTmp1*/

	FaceEyeDetector fEDetector;
	bool openResult;
public:

    FaceEyeDetectorModule();
    ~FaceEyeDetectorModule();
    
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();
	void drawDetectedFaces(IplImage *currentImage, int lineThickness, CvScalar color);
};

#endif

