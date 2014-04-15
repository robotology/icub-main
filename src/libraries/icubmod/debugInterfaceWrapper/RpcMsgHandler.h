// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef __DEBUG_INTERFACE_RPCMSGHANDLER__
#define __DEBUG_INTERFACE_RPCMSGHANDLER__

/*
* Copyright (C) 2014 RobotCub Consortium
* Author: Alberto Cardellino
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

// RpcMsgHandler
// A small helper class that parse the RPC command from the port and
// calls appropriate DebugInterfaceWrapper method, then create and send
// a response if any.


#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <iCub/DebugInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/Wrapper.h>


namespace iCub{
    namespace Wrapper{
        namespace iDebug{
            class RpcMsgHandler;
        }
    }
}

// just a forward declaration is enough here, the include is in the cpp file.
// To solve compilation problem for entwined usege of the 2 class
class DebugInterfaceWrapper;

// Callback handler for RPC commands
class iCub::Wrapper::iDebug::RpcMsgHandler :  public yarp::dev::DeviceResponder
{
protected:
    DebugInterfaceWrapper            *caller;
//    yarp::dev::IDebugInterface       *iDbg_rpcHandler;
//    yarp::dev::IPositionControl      *iPos_rpcHandler;
    int controlledJoints;
    yarp::sig::Vector vect;

    yarp::os::Stamp lastRpcStamp;
    yarp::os::Semaphore mutex;

public:
    /**
    * Constructor.
    * @param x is the pointer to the instance of the object that uses the CommandsHelper.
    * This is required to recover the pointers to the interfaces that implement the responses
    * to the commands.
    */
    RpcMsgHandler(DebugInterfaceWrapper *x);
    ~RpcMsgHandler();

    /**
    * Initialize the internal data.
    * @return true/false on success/failure
    */
    virtual bool initialize();

    virtual bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& response);
};


#endif //__DEBUG_INTERFACE_RPCMSGHANDLER__
