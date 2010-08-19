// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Author: Giorgio Metta
 * Copyright (C) 2009 The RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 * @file ClientLogpolarFrameGrabber.cpp
 * @brief Implementation of the network wrapper device driver for the logpolar subsampling of images.
 *
 * @cond
 *
 * @author Giorgio Metta
 */

#include <stdio.h>
#include <string.h>

#include <yarp/dev/ServerLogpolarFrameGrabber.h>
#include <yarp/dev/ClientLogpolarFrameGrabber.h>
#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <yarp/dev/PolyDriver.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/*
 * implementation of the ClientLogpolarFrameGrabber class.
 */

ClientLogpolarFrameGrabber::ClientLogpolarFrameGrabber() : RemoteFrameGrabberDC1394(), nmutex(1) {}

bool ClientLogpolarFrameGrabber::open(yarp::os::Searchable& config) {
    nmutex.wait();
    bool ok = RemoteFrameGrabberDC1394::open(config);
    if (!ok) {
        fprintf(stderr, "ClientLogpolarFrameGrabber: can't open the dc1394 controls device\n");
        nmutex.post();
        return false;
    }

    ConstString loc = local + "/logpolar";
    portLogpolar.open(loc);
    ConstString loc2 = local + "/fovea";
    portFoveal.open(loc2);

    // don't connect if !remote.
    if (remote != "") {
        yarp::os::ConstString carrier = 
            config.check("stream",yarp::os::Value("tcp"),
                         "carrier to use for streaming").asString();
         ConstString rem = remote + "/logpolar";
         ConstString rem2 = remote + "/fovea";
         bool ok = false;
         ok = Network::connect(rem, loc, carrier);
         ok &= Network::connect(rem2, loc2, carrier);
         if (!ok) {
             fprintf(stderr, "ClientLogpolarFrameGrabber: can't connect to the server\n");
             portLogpolar.close();
             portFoveal.close();
             nmutex.post();
             return false;
         }
    }
    
    // attach the readers.
    readerLogpolar.attach(portLogpolar);
    readerFoveal.attach(portFoveal);

    nmutex.post();
    return true;
}

bool ClientLogpolarFrameGrabber::close() {
    nmutex.wait();
    portLogpolar.close();
    portFoveal.close();
    nmutex.post();
    return RemoteFrameGrabberDC1394::close();
}

/**
 * @endcond
 */

