// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __UZH_ICALIBTOOL__
#define __UZH_ICALIBTOOL__

// yarp
#include <yarp/sig/Image.h>
#include <yarp/os/IConfig.h>

namespace iCub {
    namespace contrib{
        class ICalibTool;
    }
}

using namespace yarp::os;
using namespace yarp::sig;

/**
 * Interface to calibrate and project input image based on camera's internal parameters and projection mode\n
 */
class iCub::contrib::ICalibTool : public IConfig
{

public:

    // IConfig
    virtual bool open (Searchable &config) = 0;
    virtual bool close () = 0;
    virtual bool configure (Searchable &config) = 0;

    virtual void apply(const ImageOf<PixelRgb> & in, ImageOf<PixelRgb> & out) = 0;    
};


#endif

 
 
 
