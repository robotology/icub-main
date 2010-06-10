// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_RUDDYSALIENCE_INC
#define ICUB_RUDDYSALIENCE_INC

#include <iCub/vis/Salience.h>

namespace iCub {
    namespace vis {
        class RuddySalience;
    }
}

/**
 * A simple skin hue filter.  Such filters are very unreliable
 * in general lighting conditions.
 */
class iCub::vis::RuddySalience : public Salience {
private:
    float transformWeights[6];
    float transformDelta;
public:
    RuddySalience();

    float eval(yarp::sig::PixelRgb& pix);

    virtual bool open(yarp::os::Searchable& config);

    void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
               yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
               yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

};

#endif
