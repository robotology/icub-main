// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Lorenzo Natale
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file FactoryInterface.h
 * @brief Interface for a factory device; a device that can create objects.
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

/**
 * @ingroup icub_icubDev
 * Interface for a factory device; a device that can create objects.
 *
 */
class yarp::dev::IFactoryInterface {
public:
    virtual ~IFactoryInterface() {}
    
    /* Creates a DeviceDriver object, return a valid pointer if successfull. */
    virtual yarp::dev::DeviceDriver *createDevice(yarp::os::Searchable& config)=0;
};


#endif
