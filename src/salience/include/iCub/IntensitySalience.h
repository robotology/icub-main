// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_INTENSITYSALIENCE_INC
#define ICUB_INTENSITYSALIENCE_INC

//opencv
#include <cv.h>

//yarp
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include "Salience.h"

namespace iCub {
    namespace contrib {
        class IntensitySalience;
    }
}

/**
 * Just a conversion to grayscale
 */
class iCub::contrib::IntensitySalience : public Salience {
private:
    IplImage *_imgGray;
    CvSize _sizeOld;
    void initImages(CvSize size);
    void releaseImages();
    void gray2Float(IplImage *imgGray, IplImage *imgFloat);
public:
    IntensitySalience();
    virtual ~IntensitySalience(){}
    virtual bool open(yarp::os::Searchable& config);
    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
               yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
               yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

};

#endif
