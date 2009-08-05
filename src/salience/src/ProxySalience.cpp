// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <yarp/sig/all.h>
#include <iCub/ProxySalience.h>

#include <stdio.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace iCub::contrib;


bool ProxySalience::close(){
    port.close();
    return true;
}


bool ProxySalience::open(yarp::os::Searchable& config){
    bool ok = Salience::open(config);
    if (!ok) return false;
    
    // for now, local port names do not inherit module root prefix

    ConstString localName = 
        config.check("local",
                     Value("..."),
                     "local port name for proxy").asString();

    ConstString remoteName = 
        config.check("remote",
                     Value(""),
                     "remote port name to get data from").asString();

    ConstString carrierName = 
        config.check("carrier",
                     Value(""),
                     "carrier to use (tcp, mcast, etc)").asString();

    if (remoteName=="") {
        return false;
    }

    port.open(localName.c_str());
    Network::connect(remoteName.c_str(),
                     port.getName().c_str(),
                     (carrierName!="")?carrierName.c_str():NULL);

    return true;
}


void ProxySalience::applyImpl(ImageOf<PixelRgb>& src, 
                              ImageOf<PixelRgb>& dest,
                              ImageOf<PixelFloat>& sal) {
    // we ignore the local image source, and read from a remote port

    if (last.width()!=0) {
        if (dest.width()!=0) {
            dest.copy(last);
        }
        sal.copy(last);
    } else {
        sal.resize(src);
        sal.zero();
        dest.resize(src);
        dest.zero();
    }
}


void ProxySalience::onRead(yarp::sig::ImageOf<yarp::sig::PixelFloat>& datum) {
    mutex.wait();
    last = datum;
    mutex.post();
}


