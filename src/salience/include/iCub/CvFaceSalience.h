// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_CVFACESALIENCE_INC
#define ICUB_CVFACESALIENCE_INC

// std
#include <stdio.h>
#include <iostream>
#include <string>
#include <math.h>

// cv
#include <cv.h>

// iCub
#include <iCub/Salience.h>

// yarp
#include <yarp/sig/all.h>
#include <yarp/os/Value.h>
#include <yarp/os/Searchable.h>
#include <yarp/os/IConfig.h>

namespace iCub {
    namespace contrib {
        class CvFaceSalience;
    }
}

using namespace std;

/**
 * Salience filter for faces, based on OpenCV face detection
 *
 * Note: There seems to be a memory leak using OpenCV beta 5, this is solved
 * in OpenCV 1.0.
 */
class iCub::contrib::CvFaceSalience : public Salience {

private:
 
    // Images
    IplImage *_ocvGrayImg;	

    CvHaarClassifierCascade* _haarCascade;
    CvMemStorage* _storage;

    CvSize              _sizeOld;

    void initImages(CvSize size);
    void releaseImages();
	void createGaussianFilter2D(float **filter, int sizex, int sizey, float ex, float ey, bool normalize=true);
    //void colorRgbFromFloat(IplImage* imgFloat, IplImage* imgRgb, float scaleFactor);

public:
    CvFaceSalience();
    virtual ~CvFaceSalience();

    virtual bool open(yarp::os::Searchable& config);
    virtual bool configure(yarp::os::Searchable& config);
    virtual bool close();

    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                       yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

};

#endif
