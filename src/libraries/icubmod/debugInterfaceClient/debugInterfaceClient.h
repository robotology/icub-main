// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#ifndef __DEBUG_INTERFACE_MOD_H__
#define __DEBUG_INTERFACE_MOD_H__

//#include <stdio.h>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <iCub/DebugInterfaces.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/dev/ControlBoardHelpers.h>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

namespace yarp{
    namespace dev {
        class DebugInterfaceClient;
    }
}

class yarp::dev::DebugInterfaceClient : public IDebugInterface, public DeviceDriver 
{
protected:
    Port rpc_p;
    Port command_p;
    PortReaderBuffer<yarp::sig::Vector> state_buffer;
    PortWriterBuffer<CommandMessage> command_buffer;

    ConstString remote;
    ConstString local;
    mutable Stamp lastStamp;
    Semaphore mutex;
    int nj;

public:

    DebugInterfaceClient();
    virtual ~DebugInterfaceClient();
    virtual bool open();
    virtual bool open(Searchable& config);
    virtual bool close();
	bool setParameter(int j, unsigned int type, double t);
    bool getParameter(int j, unsigned int type, double *t);
	bool setDebugParameter(int j, unsigned int index, double t);
    bool getDebugParameter(int j, unsigned int index, double *t);
};

#endif
