// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * \defgroup icub_factoryinterface IFactoryInterface
 * @ingroup icub_icubDev
 *
 * Interface for a factory device; a device that can create objects.
 *
 * Author: Lorenzo Natale
 * Copyright (C) 2010 RobotCub Consortium
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
*/

#ifndef __FACTORYINTERFACE__
#define __FACTORYINTERFACE__

#include <yarp/dev/DeviceDriver.h>

/* LATER: is it likely that some of these would move into iCub::dev namespace? */
namespace yarp{
    namespace dev {
        class IFactoryInterface;
    }
}

class yarp::dev::IFactoryInterface {
public:
    virtual ~IFactoryInterface() {}
    
    /* Creates a DeviceDriver object, return a valid pointer if successfull. */
    virtual yarp::dev::DeviceDriver *createDevice(yarp::os::Searchable& config)=0;
};


#endif