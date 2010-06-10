// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ICUB_COLORSALIENCE_INC
#define ICUB_COLORSALIENCE_INC

//yarp
#include <yarp/os/Searchable.h>
#include <yarp/os/Value.h>

#include <iCub/vis/Salience.h>

namespace iCub {
    namespace vis {
        class ColorSalience;
    }
}

/**
 * A color salience filter, following Laurent Itti and Brian Scassellati.
 */
class iCub::vis::ColorSalience : public Salience {
public:
    virtual bool open(yarp::os::Searchable& config);
    virtual void applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
               yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
               yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal);

};

#endif
