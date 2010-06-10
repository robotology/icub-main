// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_EMDSALIENCE_INC
#define ICUB_EMDSALIENCE_INC

// std
#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>

// iCub
#include <iCub/vis/OptFlowEMD.h>
#include <iCub/vis/Salience.h>

// yarp
#include <yarp/sig/Image.h>
#include <yarp/os/Value.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/IConfig.h>

namespace iCub {
    namespace vis {
        class EMDSalience;
    }
}

using namespace std;

/**
 * Salience filter based on the Reichardt/correlation model for motion detection.
 */
class iCub::vis::EMDSalience : public Salience {

private:
  
    OptFlowEMD                                  _emd;

    // Images
    IplImage			*_ocvBgrImgInput;	// Ipl copy of grabbed yarp image (otherwise trouble with header data of wrapped yarp image)
    IplImage			*_ocvGryImgNew;		// new input image grayscale 
	IplImage			*_ocvGryImgOld;		// old input image grayscale
    IplImage	        *_ocvFltImgX;		// flow in x direction floating point
	IplImage			*_ocvFltImgY;		// flow in y direction floating point
	IplImage			*_ocvFltImgAbs;	    // flow absolute floating point

    CvSize              _sizeOld;
    bool                _blurInput;
    bool                _blurOutput;

    void initImages(CvSize size);
    void releaseImages();
    void colorRgbFromFloat(IplImage* imgFloat, IplImage* imgRgb, float scaleFactor);

public:
    EMDSalience();
    virtual ~EMDSalience();

    virtual bool open(yarp::os::Searchable& config);
    virtual bool configure(yarp::os::Searchable& config);
    virtual bool close();

    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                       yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

};

#endif
