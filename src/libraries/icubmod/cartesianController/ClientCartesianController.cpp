/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Developed by Ugo Pattacini

#include <stdio.h>
#include <algorithm>
#include <sstream>

#include "CommonCartesianController.h"
#include "ClientCartesianController.h"

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinInv.h>

#define CARTCTRL_CLIENT_VER     1.1
#define CARTCTRL_DEFAULT_TMO    0.1 // [s]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/************************************************************************/
void CartesianEventHandler::onRead(Bottle &event)
{
    if (interface!=NULL)
        interface->eventHandling(event);
}


/************************************************************************/
void CartesianEventHandler::setInterface(ClientCartesianController *interface)
{
    this->interface=interface;
    setStrict();
    useCallback();
}


/************************************************************************/
ClientCartesianController::ClientCartesianController()
{
    init();
}


/************************************************************************/
ClientCartesianController::ClientCartesianController(Searchable &config)
{
    init();
    open(config);
}


/************************************************************************/
void ClientCartesianController::init()
{
    connected=false;
    closed=false;

    timeout=CARTCTRL_DEFAULT_TMO;
    lastPoseMsgArrivalTime=0.0;

    pose.resize(7,0.0);

    portEvents.setInterface(this);
}


/************************************************************************/
bool ClientCartesianController::open(Searchable &config)
{
    ConstString remote, local, carrier;

    if (config.check("remote"))
        remote=config.find("remote").asString();
    else
        return false;

    if (config.check("local"))
        local=config.find("local").asString();
    else
        return false;
    
    carrier=config.check("carrier",Value("udp")).asString();

    if (config.check("timeout"))
        timeout=config.find("timeout").asDouble();

    portCmd.open((local+"/command:o").c_str());
    portState.open((local+"/state:i").c_str());
    portEvents.open((local+"/events:i").c_str());
    portRpc.open((local+"/rpc:o").c_str());

    bool ok=true;

    ok&=Network::connect(portRpc.getName().c_str(),(remote+"/rpc:i").c_str());
    if (ok)
    {
        Bottle info;
        getInfoHelper(info);
        if (info.check("server_version"))
        {
            double server_version=info.find("server_version").asDouble();
            if (server_version!=CARTCTRL_CLIENT_VER)
            {
                printf("Error: version mismatch => server(%g) != client(%g); please update accordingly\n",
                       server_version,CARTCTRL_CLIENT_VER);
                return false;
            }
        }
        else
            printf("Warning: unable to retrieve server version; please update the server\n");
    }
    else
    {
        printf("Error: unable to connect to the server rpc port!\n");
        return false;
    }

    ok&=Network::connect(portCmd.getName().c_str(),(remote+"/command:i").c_str(),carrier.c_str());
    ok&=Network::connect((remote+"/state:o").c_str(),portState.getName().c_str(),carrier.c_str());
    ok&=Network::connect((remote+"/events:o").c_str(),portEvents.getName().c_str(),carrier.c_str());    

    // check whether the solver is alive and connected
    if (ok)
    {
        Bottle command, reply;
    
        command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
        command.addVocab(IKINCARTCTRL_VOCAB_OPT_ISSOLVERON);
    
        if (!portRpc.write(command,reply))
        {
            printf("Error: unable to get reply from server!\n");
            close();

            return false;
        }

        if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
            if (reply.size()>1)
                if (reply.get(1).asVocab()==IKINCARTCTRL_VOCAB_VAL_TRUE)
                    return connected=true;

        printf("Error: unable to connect to solver!\n");
        close();

        return false;
    }
    else
    {
        printf("Error: unable to connect to server!\n");
        close();

        return false;
    }
}


/************************************************************************/
bool ClientCartesianController::close()
{
    if (closed)
        return true;

    deleteContexts();

    while (eventsMap.size()>0)
        unregisterEvent(*eventsMap.begin()->second);

    portCmd.interrupt();
    portState.interrupt();
    portEvents.interrupt();
    portRpc.interrupt();

    portCmd.close();
    portState.close();
    portEvents.close();
    portRpc.close();

    connected=false;

    return closed=true;
}


/************************************************************************/
bool ClientCartesianController::setTrackingMode(const bool f)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_MODE);

    command.addVocab(f?IKINCARTCTRL_VOCAB_VAL_MODE_TRACK:IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getTrackingMode(bool *f)
{
    if (!connected || (f==NULL))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_MODE);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        int mode=reply.get(1).asVocab();

        if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_TRACK)
            *f=true;
        else if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE)
            *f=false;
        else
            return false;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::setReferenceMode(const bool f)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_REFERENCE);

    command.addVocab(f?IKINCARTCTRL_VOCAB_VAL_TRUE:IKINCARTCTRL_VOCAB_VAL_FALSE);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getReferenceMode(bool *f)
{
    if (!connected || (f==NULL))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_REFERENCE);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        int mode=reply.get(1).asVocab();

        if (mode==IKINCARTCTRL_VOCAB_VAL_TRUE)
            *f=true;
        else if (mode==IKINCARTCTRL_VOCAB_VAL_FALSE)
            *f=false;
        else
            return false;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::setPosePriority(const ConstString &p)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_PRIO);
    command.addString(p);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getPosePriority(ConstString &p)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_PRIO);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        p=reply.get(1).asString();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getPose(Vector &x, Vector &o, Stamp *stamp)
{
    if (!connected)
        return false;

    double now=Time::now();

    // receive from network in streaming mode (non-blocking)
    if (Vector *v=portState.read(false))
    {
        pose=*v;
        portState.getEnvelope(poseStamp);
        lastPoseMsgArrivalTime=now;
    }

    x.resize(3);
    o.resize(pose.length()-x.length());

    for (size_t i=0; i<x.length(); i++)
        x[i]=pose[i];

    for (size_t i=0; i<o.length(); i++)
        o[i]=pose[x.length()+i];

    if (stamp!=NULL)
        *stamp=poseStamp;

    return (now-lastPoseMsgArrivalTime<timeout);
}


/************************************************************************/
bool ClientCartesianController::getPose(const int axis, Vector &x, Vector &o,
                                        Stamp *stamp)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_POSE);
    command.addInt(axis);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *posePart=reply.get(1).asList())
        {
            x.resize(3);
            o.resize(posePart->size()-x.length());

            for (size_t i=0; i<x.length(); i++)
                x[i]=posePart->get(i).asDouble();

            for (size_t i=0; i<o.length(); i++)
                o[i]=posePart->get(x.length()+i).asDouble();

            if ((reply.size()>2) && (stamp!=NULL))
            {
                if (Bottle *stampPart=reply.get(2).asList())
                {
                    Stamp tmpStamp(stampPart->get(0).asInt(),
                                   stampPart->get(1).asDouble());

                    *stamp=tmpStamp;
                }
            }

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::goToPose(const Vector &xd, const Vector &od, const double t)
{
    if (!connected || (xd.length()<3) || (od.length()<4))
        return false;

    Bottle &command=portCmd.prepare();
    command.clear();

    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_FULL);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);

    for (int i=0; i<4; i++)
        xdesPart.addDouble(od[i]);    

    // send command
    portCmd.writeStrict();
    return true;
}


/************************************************************************/
bool ClientCartesianController::goToPosition(const Vector &xd, const double t)
{
    if (!connected || (xd.length()<3))
        return false;

    Bottle &command=portCmd.prepare();
    command.clear();

    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_XYZ);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);    

    // send command
    portCmd.writeStrict();
    return true;
}


/************************************************************************/
bool ClientCartesianController::goToPoseSync(const Vector &xd, const Vector &od, const double t)
{
    if (!connected || (xd.length()<3) || (od.length()<4))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_FULL);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);

    for (int i=0; i<4; i++)
        xdesPart.addDouble(od[i]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::goToPositionSync(const Vector &xd, const double t)
{
    if (!connected || (xd.length()<3))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GO);
    command.addVocab(IKINCARTCTRL_VOCAB_VAL_POSE_XYZ);
    command.addDouble(t);
    Bottle &xdesPart=command.addList();

    for (int i=0; i<3; i++)
        xdesPart.addDouble(xd[i]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getDesired(Vector &xdhat, Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_DES);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ClientCartesianController::askForPose(const Vector &xd, const Vector &od,
                                           Vector &xdhat, Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;
    Vector tg(xd.length()+od.length());
    for (size_t i=0; i<xd.length(); i++)
        tg[i]=xd[i];

    for (size_t i=0; i<od.length(); i++)
        tg[xd.length()+i]=od[i];
    
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,tg);
    addPoseOption(command,IKINCTRL_POSE_FULL);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ClientCartesianController::askForPose(const Vector &q0, const Vector &xd,
                                           const Vector &od, Vector &xdhat,
                                           Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;
    Vector tg(xd.length()+od.length());
    for (size_t i=0; i<xd.length(); i++)
        tg[i]=xd[i];

    for (size_t i=0; i<od.length(); i++)
        tg[xd.length()+i]=od[i];

    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,tg);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_Q,q0);
    addPoseOption(command,IKINCTRL_POSE_FULL);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ClientCartesianController::askForPosition(const Vector &xd, Vector &xdhat,
                                               Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,xd);
    addPoseOption(command,IKINCTRL_POSE_XYZ);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ClientCartesianController::askForPosition(const Vector &q0, const Vector &xd,
                                               Vector &xdhat, Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,xd);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_Q,q0);
    addPoseOption(command,IKINCTRL_POSE_XYZ);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ClientCartesianController::getDOF(Vector &curDof)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_DOF);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *dofPart=reply.get(1).asList())
        {
            curDof.resize(dofPart->size());
            
            for (size_t i=0; i<curDof.length(); i++)
                curDof[i]=dofPart->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::setDOF(const Vector &newDof, Vector &curDof)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_DOF);
    Bottle &dofPart=command.addList();

    for (size_t i=0; i<newDof.length(); i++)
        dofPart.addInt((int)newDof[i]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {        
        if (Bottle *dofPart=reply.get(1).asList())
        {                        
            curDof.resize(dofPart->size());

            for (size_t i=0; i<curDof.length(); i++)
                curDof[i]=dofPart->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::getRestPos(Vector &curRestPos)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_REST_POS);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *restPart=reply.get(1).asList())
        {
            curRestPos.resize(restPart->size());
            
            for (size_t i=0; i<curRestPos.length(); i++)
                curRestPos[i]=restPart->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::setRestPos(const Vector &newRestPos, Vector &curRestPos)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_REST_POS);
    Bottle &restPart=command.addList();

    for (size_t i=0; i<newRestPos.length(); i++)
        restPart.addDouble(newRestPos[i]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {        
        if (Bottle *restPart=reply.get(1).asList())
        {                        
            curRestPos.resize(restPart->size());

            for (size_t i=0; i<curRestPos.length(); i++)
                curRestPos[i]=restPart->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::getRestWeights(Vector &curRestWeights)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_REST_WEIGHTS);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *restPart=reply.get(1).asList())
        {
            curRestWeights.resize(restPart->size());
            
            for (size_t i=0; i<curRestWeights.length(); i++)
                curRestWeights[i]=restPart->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::setRestWeights(const Vector &newRestWeights,
                                               Vector &curRestWeights)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_REST_WEIGHTS);
    Bottle &restPart=command.addList();

    for (size_t i=0; i<newRestWeights.length(); i++)
        restPart.addDouble(newRestWeights[i]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {        
        if (Bottle *restPart=reply.get(1).asList())
        {                        
            curRestWeights.resize(restPart->size());

            for (size_t i=0; i<curRestWeights.length(); i++)
                curRestWeights[i]=restPart->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::getLimits(const int axis, double *min, double *max)
{
    if (!connected || (min==NULL) || (max==NULL))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_LIM);
    command.addInt(axis);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>2)
        {
            *min=reply.get(1).asDouble();
            *max=reply.get(2).asDouble();
            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::setLimits(const int axis, const double min, const double max)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_LIM);
    command.addInt(axis);
    command.addDouble(min);
    command.addDouble(max);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
        return true;
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::getTrajTime(double *t)
{
    if (!connected || (t==NULL))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TIME);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            *t=reply.get(1).asDouble();
            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::setTrajTime(const double t)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TIME);
    command.addDouble(t);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getInTargetTol(double *tol)
{
    if (!connected || (tol==NULL))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TOL);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            *tol=reply.get(1).asDouble();
            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::setInTargetTol(const double tol)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TOL);
    command.addDouble(tol);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getJointsVelocities(Vector &qdot)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_QDOT);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *qdotPart=reply.get(1).asList())
        {
            qdot.resize(qdotPart->size());

            for (size_t i=0; i<qdot.length(); i++)
                qdot[i]=qdotPart->get(i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::getTaskVelocities(Vector &xdot, Vector &odot)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_XDOT);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *xdotPart=reply.get(1).asList())
        {
            xdot.resize(3);
            odot.resize(xdotPart->size()-xdot.length());

            for (size_t i=0; i<xdot.length(); i++)
                xdot[i]=xdotPart->get(i).asDouble();

            for (size_t i=0; i<odot.length(); i++)
                odot[i]=xdotPart->get(xdot.length()+i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::setTaskVelocities(const Vector &xdot, const Vector &odot)
{
    if (!connected || (xdot.length()<3) || (odot.length()<4))
        return false;

    Bottle &command=portCmd.prepare();
    command.clear();

    command.addVocab(IKINCARTCTRL_VOCAB_CMD_TASKVEL);
    Bottle &xdotPart=command.addList();

    for (int i=0; i<3; i++)
        xdotPart.addDouble(xdot[i]);

    for (int i=0; i<4; i++)
        xdotPart.addDouble(odot[i]);

    // send command
    portCmd.writeStrict();
    return true;
}


/************************************************************************/
bool ClientCartesianController::attachTipFrame(const Vector &x, const Vector &o)
{
    if (!connected || (x.length()<3) || (o.length()<4))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TIP_FRAME);
    Bottle &tipPart=command.addList();

    for (int i=0; i<3; i++)
        tipPart.addDouble(x[i]);

    for (int i=0; i<4; i++)
        tipPart.addDouble(o[i]);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getTipFrame(Vector &x, Vector &o)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TIP_FRAME);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (Bottle *tipPart=reply.get(1).asList())
        {
            x.resize(3);
            o.resize(tipPart->size()-x.length());

            for (size_t i=0; i<x.length(); i++)
                x[i]=tipPart->get(i).asDouble();

            for (size_t i=0; i<o.length(); i++)
                o[i]=tipPart->get(x.length()+i).asDouble();

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::removeTipFrame()
{
    return attachTipFrame(Vector(3,0.0),Vector(4,0.0));
}


/************************************************************************/
bool ClientCartesianController::checkMotionDone(bool *f)
{
    if (!connected || (f==NULL))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_MOTIONDONE);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            int flag=reply.get(1).asVocab();

            if (flag==IKINCARTCTRL_VOCAB_VAL_TRUE)
                *f=true;
            else if (flag==IKINCARTCTRL_VOCAB_VAL_FALSE)
                *f=false;

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::waitMotionDone(const double period, const double timeout)
{
    bool done=false;
    double t0=Time::now();

    while (!done)
    {
        Time::delay(period);

        if (!checkMotionDone(&done) || ((timeout>0.0) && ((Time::now()-t0)>timeout)))
            return false;
    }

    return true;
}


/************************************************************************/
bool ClientCartesianController::stopControl()
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_STOP);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::storeContext(int *id)
{
    if (!connected || (id==NULL))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_STORE);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            contextIdList.insert(*id=reply.get(1).asInt());
            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::restoreContext(const int id)
{
    if (!connected || ((contextIdList.find(id)==contextIdList.end()) && (id!=0)))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_RESTORE);
    command.addInt(id);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::deleteContext(const int id)
{
    if (!connected || ((contextIdList.find(id)==contextIdList.end()) && (id!=0)))
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_DELETE);
    command.addList().addInt(id);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        contextIdList.erase(id);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ClientCartesianController::deleteContexts()
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_DELETE);
    Bottle &ids=command.addList();
    for (set<int>::iterator itr=contextIdList.begin(); itr!=contextIdList.end(); itr++)
        ids.addInt(*itr);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    contextIdList.clear();

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::getInfoHelper(Bottle &info)
{
    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_INFO);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            if (Bottle *infoPart=reply.get(1).asList())
                info=*infoPart;

            return true;
        }
    }

    return false;
}


/************************************************************************/
bool ClientCartesianController::getInfo(Bottle &info)
{
    if (connected)
        return getInfoHelper(info);
    else
        return false;
}


/************************************************************************/
void ClientCartesianController::eventHandling(Bottle &event)
{
    string type=event.get(0).asString().c_str();
    double time=event.get(1).asDouble();
    double checkPoint=(type=="motion-ongoing")?event.get(2).asDouble():-1.0;
    map<string,CartesianEvent*>::iterator itr;

    // rise the all-events callback
    itr=eventsMap.find("*");
    if (itr!=eventsMap.end())
    {
        if (itr->second!=NULL)
        {
            CartesianEvent &Event=*itr->second;
            Event.cartesianEventVariables.type=ConstString(type.c_str());
            Event.cartesianEventVariables.time=time;

            if (checkPoint>=0.0)
                Event.cartesianEventVariables.motionOngoingCheckPoint=checkPoint;

            Event.cartesianEventCallback();
        }
    }

    string typeExtended=type;
    if (checkPoint>=0.0)
    {
        ostringstream ss;
        ss<<type<<"-"<<checkPoint;
        typeExtended=ss.str();
    }

    // rise the event specific callback
    itr=eventsMap.find(typeExtended);
    if (itr!=eventsMap.end())
    {
        if (itr->second!=NULL)
        {
            CartesianEvent &Event=*itr->second;
            Event.cartesianEventVariables.type=ConstString(type.c_str());
            Event.cartesianEventVariables.time=time;

            if (checkPoint>=0.0)
                Event.cartesianEventVariables.motionOngoingCheckPoint=checkPoint;

            Event.cartesianEventCallback();
        }
    }
}


/************************************************************************/
bool ClientCartesianController::registerEvent(CartesianEvent &event)
{
    if (!connected)
        return false;

    string type=event.cartesianEventParameters.type.c_str();
    if (type=="motion-ongoing")
    {
        double checkPoint=event.cartesianEventParameters.motionOngoingCheckPoint;

        Bottle command, reply;
        command.addVocab(IKINCARTCTRL_VOCAB_CMD_EVENT);
        command.addVocab(IKINCARTCTRL_VOCAB_OPT_REGISTER);
        command.addVocab(IKINCARTCTRL_VOCAB_VAL_EVENT_ONGOING);
        command.addDouble(checkPoint);

        if (!portRpc.write(command,reply))
        {
            printf("Error: unable to get reply from server!\n");
            return false;
        }

        if (reply.get(0).asVocab()!=IKINCARTCTRL_VOCAB_REP_ACK)
            return false;

        ostringstream ss;
        ss<<type<<"-"<<checkPoint;
        type=ss.str();
    }

    eventsMap[type]=&event;
    return true;
}


/************************************************************************/
bool ClientCartesianController::unregisterEvent(CartesianEvent &event)
{
    if (!connected)
        return false;

    string type=event.cartesianEventParameters.type.c_str();
    if (type=="motion-ongoing")
    {
        double checkPoint=event.cartesianEventParameters.motionOngoingCheckPoint;

        Bottle command, reply;
        command.addVocab(IKINCARTCTRL_VOCAB_CMD_EVENT);
        command.addVocab(IKINCARTCTRL_VOCAB_OPT_UNREGISTER);
        command.addVocab(IKINCARTCTRL_VOCAB_VAL_EVENT_ONGOING);
        command.addDouble(checkPoint);

        if (!portRpc.write(command,reply))
        {
            printf("Error: unable to get reply from server!\n");
            return false;
        }

        if (reply.get(0).asVocab()!=IKINCARTCTRL_VOCAB_REP_ACK)
            return false;

        ostringstream ss;
        ss<<type<<"-"<<checkPoint;
        type=ss.str();
    }

    eventsMap.erase(type);
    return true;
}


/************************************************************************/
bool ClientCartesianController::tweakSet(const Bottle &options)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_SET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TWEAK);
    command.addList()=options;

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    return (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK);
}


/************************************************************************/
bool ClientCartesianController::tweakGet(Bottle &options)
{
    if (!connected)
        return false;

    Bottle command, reply;
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_GET);
    command.addVocab(IKINCARTCTRL_VOCAB_OPT_TWEAK);

    if (!portRpc.write(command,reply))
    {
        printf("Error: unable to get reply from server!\n");
        return false;
    }

    if (reply.get(0).asVocab()==IKINCARTCTRL_VOCAB_REP_ACK)
    {
        if (reply.size()>1)
        {
            if (Bottle *optionsPart=reply.get(1).asList())
                options=*optionsPart;

            return true;
        }
    }

    return false;
}


/************************************************************************/
ClientCartesianController::~ClientCartesianController()
{
    close();
}


