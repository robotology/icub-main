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

#include <string.h>

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Log.h>

#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <yarp/dev/PreciselyTimed.h>
#include <debugInterfaceClient.h>


using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

/**
* @ingroup dev_impl_wrapper
*
* The client for the debug interface.
*/
yarp::dev::DebugInterfaceClient::DebugInterfaceClient()
{ 
    nj = 0;
}

/**
* Destructor.
*/
yarp::dev::DebugInterfaceClient::~DebugInterfaceClient()
{
}


/**
* Default open.
* @return always true.
*/
bool yarp::dev::DebugInterfaceClient::open()
{
    return true;
}

bool yarp::dev::DebugInterfaceClient::open(Searchable& config)
{
    remote = config.find("remote").asString().c_str();
    local = config.find("local").asString().c_str();

    ConstString carrier = 
        config.check("carrier",
        Value("udp"),
        "default carrier for streaming robot state").asString().c_str();

    if (local != "")
	{
        ConstString s1 = local;
        s1 += "/rpc:o";
        rpc_p.open(s1.c_str());
        s1 = local;
        s1 += "/command:o";
        command_p.open(s1.c_str());
 
    }

    bool connectionProblem = false;
    if (remote != "")
	{
        ConstString s1 = remote;
        s1 += "/rpc:i";
        ConstString s2 = local;
        s2 += "/rpc:o";
        bool ok = false;
        ok=Network::connect(s2.c_str(), s1.c_str());
        if (!ok)
			{
				printf("Problem connecting to %s, is the remote device available?\n", s1.c_str());
				connectionProblem = true;
			}
        s1 = remote;
        s1 += "/command:i";
        s2 = local;
        s2 += "/command:o";
        ok = Network::connect(s2.c_str(), s1.c_str(), carrier);
        if (!ok) 
			{
				printf("Problem connecting to %s, is the remote device available?\n", s1.c_str());
				connectionProblem = true;
			}
    }

    if (connectionProblem)
	{
        return false;
    }

    state_buffer.setStrict(false);
    command_buffer.attach(command_p);

	bool ok=true;
	/*
	// @@@ check this
	ok = get1V1I(VOCAB_AXES, nj);
    if (nj==0) {
        ok = false;
    }
	*/

    if (!ok) {
        printf("Problems with obtaining the number of controlled axes\n");
        return false;
    }

    return true;
}

/**
* Close the device driver and stop the port connections.
* @return true/false on success/failure.
*/
bool yarp::dev::DebugInterfaceClient::close()
{
    rpc_p.close();
    command_p.close();
    return true;
}

bool yarp::dev::DebugInterfaceClient::setParameter(int j, unsigned int type, double t)
{ 
    Bottle cmd, response;
    cmd.addVocab(VOCAB_SET);
    cmd.addVocab(VOCAB_GENERIC_PARAMETER);
    cmd.addInt(j);
	cmd.addInt(type);
	cmd.addDouble(t);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool yarp::dev::DebugInterfaceClient::getParameter(int j, unsigned int type, double *t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_GENERIC_PARAMETER);
    cmd.addInt(j);
	cmd.addInt(type);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
		&&(resp.get(1).asInt()==VOCAB_GENERIC_PARAMETER)
        &&(resp.get(2).asInt()==j)
        &&(resp.get(3).asInt()==type))
    {
        ok=ok&&true;
        *t=resp.get(4).asDouble();
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::setDebugParameter(int j, unsigned int index, double t)
{ 
    Bottle cmd, response;
    cmd.addVocab(VOCAB_SET);
    cmd.addVocab(VOCAB_DEBUG_PARAMETER);
    cmd.addInt(j);
	cmd.addInt(index);
	cmd.addDouble(t);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool yarp::dev::DebugInterfaceClient::getDebugParameter(int j, unsigned int index, double *t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_PARAMETER);
    cmd.addInt(j);
	cmd.addInt(index);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
		&&(resp.get(1).asInt()==VOCAB_DEBUG_PARAMETER)
        &&(resp.get(2).asInt()==j)
        &&(resp.get(3).asInt()==index))
    {
        ok=ok&&true;
        *t=resp.get(4).asDouble();
    }
    return ok;
}

// implementation of CommandsHelper
yarp::dev::DriverCreator *createDebugInterfaceClient() {
    return new DriverCreatorOf<DebugInterfaceClient>("debugInterfaceClient", 
        "controlboard",
        "DebugInterfaceClient");
}

