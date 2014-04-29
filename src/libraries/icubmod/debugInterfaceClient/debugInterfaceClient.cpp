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
    }

    if (connectionProblem)
	{
        return false;
    }

    state_buffer.setStrict(false);
    command_buffer.attach(command_p);

    Bottle cmd;
    Bottle response;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_AXES);

    bool ok = rpc_p.write(cmd, response);
    if (CHECK_FAIL(ok, response))
    {
        nj = response.get(2).asInt();
    }

    if (!ok || (nj == 0) ) {
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

bool yarp::dev::DebugInterfaceClient::getDebugReferencePosition(int j, double *t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_DESIRED_POS);
    cmd.addInt(j);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
		&&(resp.get(1).asInt()==VOCAB_DEBUG_DESIRED_POS)
        &&(resp.get(2).asInt()==j))
    {
        ok=ok&&true;
        *t=resp.get(3).asDouble();
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getRotorPosition(int j, double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_ROTOR_POS);
    cmd.addInt(j);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
		&&(resp.get(1).asInt()==VOCAB_DEBUG_ROTOR_POS)
        &&(resp.get(2).asInt()==j))
    {
        this->lastStamp.update(); //BEWARE: This updates the stamps of the whole interface!
        ok=ok&&true;
        *t=resp.get(3).asDouble();
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getRotorPositions(double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_ROTOR_POSS);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
        &&(resp.get(1).asInt()==VOCAB_DEBUG_ROTOR_POSS))
    {
        this->lastStamp.update(); //BEWARE: This updates the stamps of the whole interface!
        ok=ok&&true;
        for(int i = 0; i < nj; i++)
        {
            t[i] = resp.get(2+i).asDouble();
        }
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getRotorSpeed(int j, double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_ROTOR_SPEED);
    cmd.addInt(j);
    
    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
		&&(resp.get(1).asInt()==VOCAB_DEBUG_ROTOR_SPEED)
        &&(resp.get(2).asInt()==j))
    {
        this->lastStamp.update(); //BEWARE: This updates the stamp of the whole interface!
        ok=ok&&true;
        *t=resp.get(3).asDouble();
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getRotorSpeeds(double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_ROTOR_SPEEDS);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
        &&(resp.get(1).asInt()==VOCAB_DEBUG_ROTOR_SPEEDS))
    {
        this->lastStamp.update(); //BEWARE: This updates the stamp of the whole interface!
        ok=ok&&true;
        for(int i = 0; i < nj; i++)
        {
            t[i] = resp.get(2+i).asDouble();
        }
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getRotorAcceleration(int j, double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_ROTOR_ACCEL);
    cmd.addInt(j);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
		&&(resp.get(1).asInt()==VOCAB_DEBUG_ROTOR_ACCEL)
        &&(resp.get(2).asInt()==j))
    {
        this->lastStamp.update(); //BEWARE: This updates the stamp of the whole interface!
        ok=ok&&true;
        *t=resp.get(3).asDouble();
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getRotorAccelerations(double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_ROTOR_ACCELS);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
        &&(resp.get(1).asInt()==VOCAB_DEBUG_ROTOR_ACCELS))
    {
        this->lastStamp.update(); //BEWARE: This updates the stamp of the whole interface!
        ok=ok&&true;
        for(int i = 0; i < nj; i++)
        {
            t[i] = resp.get(2+i).asDouble();
        }
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::setDebugReferencePosition(int j, double t)
{ 
    Bottle cmd, response;
    cmd.addVocab(VOCAB_SET);
    cmd.addVocab(VOCAB_DEBUG_DESIRED_POS);
    cmd.addInt(j);
	cmd.addDouble(t);

    bool ok = rpc_p.write(cmd, response);
    return CHECK_FAIL(ok, response);
}

bool yarp::dev::DebugInterfaceClient::getJointPosition(int j, double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_JOINT_POS);
    cmd.addInt(j);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
		&&(resp.get(1).asInt()==VOCAB_DEBUG_JOINT_POS)
        &&(resp.get(2).asInt()==j))
    {
        ok=ok&&true;
        *t=resp.get(3).asDouble();
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getJointPositions(double* t)
{ 
    Bottle cmd, resp;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_DEBUG_JOINT_POSS);

    bool ok = rpc_p.write(cmd, resp);

    if (  (resp.get(0).asVocab()==VOCAB_IS)
        &&(resp.get(1).asInt()==VOCAB_DEBUG_JOINT_POSS))
    {
        this->lastStamp.update(); //BEWARE: This updates the stamps of the whole interface!
        ok=ok&&true;
        for(int i = 0; i < nj; i++)
        {
            t[i] = resp.get(2+i).asDouble();
        }
    }
    return ok;
}

bool yarp::dev::DebugInterfaceClient::getTimeStamp(Bottle &bot, Stamp &st)
{
    if (bot.get(3).asVocab()==VOCAB_TIMESTAMP)
    {
        //yup! we have a timestamp
        int fr=bot.get(4).asInt();
        double ts=bot.get(5).asDouble();
        st=Stamp(fr,ts);
        return true;
    }
    return false;
}

Stamp yarp::dev::DebugInterfaceClient::getLastInputStamp()
{
    Stamp ret;
//        mutex.wait();
    ret = lastStamp;
//        mutex.post();
    return ret;
}

// implementation of CommandsHelper
yarp::dev::DriverCreator *createDebugInterfaceClient() {
    return new DriverCreatorOf<DebugInterfaceClient>("debugInterfaceClient", 
        "controlboard",
        "DebugInterfaceClient");
}

