// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/Salience.h>

using namespace yarp::sig;
using namespace iCub::contrib;

Salience::Salience(){
  
}

Salience::~Salience(){
 
}

bool Salience::open(yarp::os::Searchable& config){
    bool ok = true;
    filterName = config.check( "filterName",
                            yarp::os::Value("no_name"),
                            "Name of this instance of the color filter (string).").asString();
    weight = config.check( "weight",
                            yarp::os::Value(1.0),
                            "Specifies a weighting value if filter is used within a group of filters (double).").asDouble();
    activateConspicuity = (bool)config.check("activateConspicuity",
                            Value(0),
                            "Calculate conspicuity (int [0|1]).").asInt();
    if(activateConspicuity)
        ok = ok && conspicuity.open(config);
    return ok;
}

bool Salience::close(){
    bool ok = true;
    ok = ok && conspicuity.close();
    return ok;
}

//bool Salience::configure(yarp::os::Searchable& config){
//
//    return true;
//}

void Salience::apply(ImageOf<PixelRgb>& src, 
                   ImageOf<PixelRgb>& dest,
                   ImageOf<PixelFloat>& sal) {
    if (weight > 0.0){
        applyImpl(src, dest, sal);
        if(activateConspicuity)
            conspicuity.apply((IplImage*)sal.getIplImage(), (IplImage*)sal.getIplImage());
    }
}

void Salience::applyImpl(yarp::sig::ImageOf<yarp::sig::PixelRgb>& src, 
                       yarp::sig::ImageOf<yarp::sig::PixelRgb>& dest,
                       yarp::sig::ImageOf<yarp::sig::PixelFloat>& sal){
    // by default, do nothing
    dest.copy(src);
    sal.resize(src);
    sal.zero();
}

