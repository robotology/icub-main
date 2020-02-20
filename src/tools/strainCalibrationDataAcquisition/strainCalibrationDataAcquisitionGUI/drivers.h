// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: (C) 2009 RobotCub Consortium
 * Author: Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef YARPDEV_IMPORTS
#define YARPDEV_IMPORTS

#include <string>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

namespace yarp {
    namespace dev {
        class DriverCollection;
    }
}


extern "C" {


extern void add_icubmod_devices();

}

/**
 *
 * This is an automatically generated class to initialize a collection
 * of drivers.
 *
 * Instantiate it in your main() function as:
 *   yarp::dev::DeviceCollection dev;
 *
 * That's all!  You can print the output of the dev.status() method just to 
 * make sure that all the devices you expect to be present actually are.
 *
 * To actually instantiate devices, use the yarp::dev::PolyDriver class.
 *
 */
class yarp::dev::DriverCollection {

public:

    /**
     * Add devices from all imported libraries.
     */
    DriverCollection() {
        add_icubmod_devices();
    }

    /**
     * Return a string listing all devices, to allow sanity-checking.
     */
    std::string status() {
        return Drivers::factory().toString();
    }
};

#endif
