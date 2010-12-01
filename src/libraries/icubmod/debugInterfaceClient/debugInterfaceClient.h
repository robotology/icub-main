// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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
};

#endif
