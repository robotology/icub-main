// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file LoggerInterface.h
 * @brief Group interfaces that handles logging status messages for devices.
 */

#ifndef __LOGGERINTERFACES__
#define __LOGGERINTERFACES__

#include <yarp/dev/DeviceDriver.h>

namespace yarp{
    namespace dev {
        class IClientLogger;
        class IServerLogger;
    }
}

/**
 * @ingroup icub_icubDev
 * Interface for a server that logs status messages.
 *
 */
class yarp::dev::IServerLogger
{
public:
    /*
    * Log data to server.
    * @param key value identifier/address 
    * @param data
    * @return true/false on success failure (failure is notified if the 
    * format of the message is unknown)
    */
    virtual bool log(const std::string &key,const yarp::os::Value &data)=0;

    /*
    * Log data to server.
    * @param data a bottle contains the data to log
    * @return true/false on success failure (failure is notified if the 
    * format of the message is unknown)
    */
    virtual bool log(const yarp::os::Bottle &data)=0;
};

/**
 * @ingroup icub_icubDev
 * Interface for a client that logs status messages to a server.
 *
 */
class yarp::dev::IClientLogger
{
public:
    /*
    * Set data logger server.
    * @param server is a pointer to a server implementing IServerLogger interface.
    */
    virtual void setServerLogger(const IServerLogger *server)=0;
};

#endif
