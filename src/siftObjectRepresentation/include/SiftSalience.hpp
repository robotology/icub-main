// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_MOTIONSALIENCE_INC
#define ICUB_MOTIONSALIENCE_INC

#include "Salience.h"

#include <deque>

namespace iCub {
    namespace contrib {
        class MotionSalience;
    }
}

/**
 * A simple temporal-difference filter.
 */
class iCub::contrib::MotionSalience : public Salience {
private:
    std::deque<yarp::sig::ImageOf<yarp::sig::PixelRgb> > history;
    int targetLen;
public:
    MotionSalience(int gap = 5) {
        targetLen = gap;
    }

    virtual bool open(yarp::os::Searchable& config);
    
    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                       yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

};

#endif
 
