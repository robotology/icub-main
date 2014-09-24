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
#include <stdlib.h>
#include <algorithm>
#include <sstream>

#include "CommonCartesianController.h"
#include "ServerCartesianController.h"

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinVocabs.h>

#define CARTCTRL_SERVER_VER                 1.1
#define CARTCTRL_DEFAULT_PER                10      // [ms]
#define CARTCTRL_DEFAULT_TASKVEL_PERFACTOR  4
#define CARTCTRL_DEFAULT_TOL                1e-2
#define CARTCTRL_DEFAULT_TRAJTIME           2.0     // [s]
#define CARTCTRL_DEFAULT_POSCTRL            "on"
#define CARTCTRL_DEFAULT_MULJNTCTRL         "on"
#define CARTCTRL_MAX_ACCEL                  1e9     // [deg/s^2]
#define CARTCTRL_CONNECT_TMO                5e3     // [ms]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
CartesianCtrlRpcProcessor::CartesianCtrlRpcProcessor(ServerCartesianController *server)
{
    this->server=server;
}


/************************************************************************/
bool CartesianCtrlRpcProcessor::read(ConnectionReader &connection)
{
    Bottle cmd, reply;

    if (!cmd.read(connection))
        return false;

    if (server->respond(cmd,reply))
        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);

    return true;
}


/************************************************************************/
CartesianCtrlCommandPort::CartesianCtrlCommandPort(ServerCartesianController *server)
{
    this->server=server;
    useCallback();
}


/************************************************************************/
void CartesianCtrlCommandPort::onRead(Bottle &command)
{
    if (command.size())
    {
        if ((command.get(0).asVocab()==IKINCARTCTRL_VOCAB_CMD_GO) && (command.size()>3))
        {
            int pose=command.get(1).asVocab();
            double t=command.get(2).asDouble();
            Bottle *v=command.get(3).asList();
        
            if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_FULL)
            {
                Vector xd(3);
                Vector od(v->size()-xd.length());

                for (size_t i=0; i<xd.length(); i++)
                    xd[i]=v->get(i).asDouble();

                for (size_t i=0; i<od.length(); i++)
                    od[i]=v->get(xd.length()+i).asDouble();

                server->goToPose(xd,od,t);
            }
            else if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_XYZ)
            {
                Vector xd(v->size());

                for (int i=0; i<v->size(); i++)
                    xd[i]=v->get(i).asDouble();

                server->goToPosition(xd,t);
            }
        }
        else if ((command.get(0).asVocab()==IKINCARTCTRL_VOCAB_CMD_TASKVEL) && (command.size()>1))
        {
            Bottle *v=command.get(1).asList();

            Vector xdot(3);
            Vector odot(v->size()-xdot.length());

            for (size_t i=0; i<xdot.length(); i++)
                xdot[i]=v->get(i).asDouble();

            for (size_t i=0; i<odot.length(); i++)
                odot[i]=v->get(xdot.length()+i).asDouble();

            server->setTaskVelocities(xdot,odot);
        }
    }
}


/************************************************************************/
ServerCartesianController::ServerCartesianController() :
                           RateThread(CARTCTRL_DEFAULT_PER)
{
    init();
}


/************************************************************************/
ServerCartesianController::ServerCartesianController(Searchable &config) :
                           RateThread(CARTCTRL_DEFAULT_PER)
{
    init();
    open(config);
}


/************************************************************************/
void ServerCartesianController::init()
{
    // initialization
    limbState=limbPlan=NULL;
    chainState=chainPlan=NULL;
    ctrl=NULL;

    portCmd     =NULL;
    rpcProcessor=NULL;

    attached     =false;
    connected    =false;
    closed       =false;
    trackingMode =false;
    executingTraj=false;
    taskVelModeOn=false;
    motionDone   =true;
    useReferences=false;
    jointsHealthy=true;

    connectCnt=0;
    ctrlPose=IKINCTRL_POSE_FULL;
    maxPartJoints=0;
    targetTol=CARTCTRL_DEFAULT_TOL;
    trajTime=CARTCTRL_DEFAULT_TRAJTIME;

    txToken=0.0;
    rxToken=0.0;
    txTokenLatchedStopControl=0.0;
    txTokenLatchedGoToRpc=0.0;
    skipSlvRes=false;
    syncEventEnabled=false;

    contextIdCnt=0;

    // request high resolution scheduling
    Time::turboBoost();
}


/************************************************************************/
void ServerCartesianController::openPorts()
{
    portCmd     =new CartesianCtrlCommandPort(this);
    rpcProcessor=new CartesianCtrlRpcProcessor(this);
    portRpc.setReader(*rpcProcessor);

    ConstString prefixName="/";
    prefixName=prefixName+ctrlName;

    portSlvIn.open((prefixName+"/"+slvName+"/in").c_str());
    portSlvOut.open((prefixName+"/"+slvName+"/out").c_str());
    portSlvRpc.open((prefixName+"/"+slvName+"/rpc").c_str());
    portCmd->open((prefixName+"/command:i").c_str());
    portState.open((prefixName+"/state:o").c_str());
    portEvent.open((prefixName+"/events:o").c_str());
    portRpc.open((prefixName+"/rpc:i").c_str());

    if (debugInfoEnabled)
        portDebugInfo.open((prefixName+"/dbg:o").c_str());
}


/************************************************************************/
void ServerCartesianController::closePorts()
{
    portSlvIn.interrupt();
    portSlvOut.interrupt();
    portSlvRpc.interrupt();
    portState.interrupt();
    portEvent.interrupt();
    portRpc.interrupt();

    portSlvIn.close();
    portSlvOut.close();
    portSlvRpc.close();
    portState.close();
    portEvent.close();
    portRpc.close();

    if (portCmd!=NULL)
    {
        portCmd->interrupt();
        portCmd->close();
        delete portCmd;
    }

    if (debugInfoEnabled)
    {
        portDebugInfo.interrupt();
        portDebugInfo.close();
    }

    delete rpcProcessor;
    connected=false;
}


/************************************************************************/
bool ServerCartesianController::respond(const Bottle &command, Bottle &reply)
{
    if (command.size())
        switch (command.get(0).asVocab())
        {
            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_STOP:
            {
                stopControl();
                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                break;
            }

            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_GO:
            {
                if (command.size()>3)
                {
                    int pose=command.get(1).asVocab();
                    double t=command.get(2).asDouble();
                    Bottle *v=command.get(3).asList();
                    Vector xd(v->size());
    
                    for (int i=0; i<v->size(); i++)
                        xd[i]=v->get(i).asDouble();

                    bool ret=false;

                    if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_FULL)
                    {
                        mutex.lock();
                        taskVelModeOn=false;
                        ret=goTo(IKINCTRL_POSE_FULL,xd,t,true);
                        mutex.unlock();
                    }
                    else if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_XYZ)
                    {
                        mutex.lock();
                        taskVelModeOn=false;
                        ret=goTo(IKINCTRL_POSE_XYZ,xd,t,true);
                        mutex.unlock();
                    }

                    if (ret)
                    {
                        // wait for the solver
                        syncEvent.reset();
                        syncEventEnabled=true;
                        syncEvent.wait();

                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                    }
                    else
                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                break;
            }

            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_ASK:
            {
                // just behave as a relay
                Bottle slvCommand=command;

                if (!portSlvRpc.write(slvCommand,reply))
                {
                    printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                }

                break;
            }

            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_STORE:
            {
                int id;
                if (storeContext(&id))
                {
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                    reply.addInt(id);
                }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_RESTORE:
            {
                if (command.size()>1)
                {
                    int id=command.get(1).asInt();
                    if (restoreContext(id))
                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                    else
                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                }
    
                break;
            }

            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_DELETE:
            {
                if (command.size()>1)
                {
                    Bottle *ids=command.get(1).asList();
                    if (deleteContexts(ids))
                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                    else
                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                }

                break;
            }

            //-----------------
            case IKINSLV_VOCAB_CMD_GET:
            {
                if (command.size()>1)
                    switch (command.get(1).asVocab())
                    {
                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_MODE:
                        {
                            bool flag;
                            if (getTrackingMode(&flag))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                 
                                if (flag)
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_MODE_TRACK);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_REFERENCE:
                        {
                            bool flag;
                            if (getReferenceMode(&flag))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                                if (flag)
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_TRUE);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_FALSE);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_PRIO:
                        {
                            ConstString priority;
                            if (getPosePriority(priority))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                reply.addString(priority);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TIME:
                        {
                            double time;
                            if (getTrajTime(&time))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                reply.addDouble(time);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TOL:
                        {
                            double tol;
                            if (getInTargetTol(&tol))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                reply.addDouble(tol);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_MOTIONDONE:
                        {
                            bool flag;
                            if (checkMotionDone(&flag))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                                if (flag)
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_TRUE);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_VAL_FALSE);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_ISSOLVERON:
                        {
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                            if (connected)
                                reply.addVocab(IKINCARTCTRL_VOCAB_VAL_TRUE);
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_VAL_FALSE);
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_LIM:
                        {
                            if (command.size()>2)
                            {
                                int axis=command.get(2).asInt();
                                double min, max;

                                if (getLimits(axis,&min,&max))
                                {
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    reply.addDouble(min);
                                    reply.addDouble(max);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_DOF:
                        {
                            Vector curDof;
                            if (getDOF(curDof))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &dofPart=reply.addList();
                                    
                                for (size_t i=0; i<curDof.length(); i++)
                                    dofPart.addInt((int)curDof[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_REST_POS:
                        {
                            Vector curRestPos;
                            if (getRestPos(curRestPos))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &restPart=reply.addList();
    
                                for (size_t i=0; i<curRestPos.length(); i++)
                                    restPart.addDouble(curRestPos[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }
    
                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_REST_WEIGHTS:
                        {
                            Vector curRestWeights;
                            if (getRestWeights(curRestWeights))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &restPart=reply.addList();
    
                                for (size_t i=0; i<curRestWeights.length(); i++)
                                    restPart.addDouble(curRestWeights[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_DES:
                        {
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                            Vector q(chainState->getN());
                            int cnt=0;

                            for (unsigned int i=0; i<chainState->getN(); i++)
                            {
                                if ((*chainState)[i].isBlocked())
                                    q[i]=CTRL_RAD2DEG*chainState->getAng(i);
                                else
                                    q[i]=CTRL_RAD2DEG*qdes[cnt++];
                            }

                            addVectorOption(reply,IKINCARTCTRL_VOCAB_OPT_X,xdes);
                            addVectorOption(reply,IKINCARTCTRL_VOCAB_OPT_Q,q);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_QDOT:
                        {
                            Vector qdot;
                            if (getJointsVelocities(qdot))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &qdotPart=reply.addList();
    
                                for (size_t i=0; i<qdot.length(); i++)
                                    qdotPart.addDouble(qdot[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }
    
                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_XDOT:
                        {
                            Vector xdot, odot;
                            if (getTaskVelocities(xdot,odot))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &xdotPart=reply.addList();
    
                                for (size_t i=0; i<xdot.length(); i++)
                                    xdotPart.addDouble(xdot[i]);
    
                                for (size_t i=0; i<odot.length(); i++)
                                    xdotPart.addDouble(odot[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_POSE:
                        {               
                            if (command.size()>2)
                            {
                                int axis=command.get(2).asInt();
                                Vector x,o;
                                Stamp stamp;

                                if (getPose(axis,x,o,&stamp))
                                {
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    Bottle &posePart=reply.addList();

                                    for (size_t i=0; i<x.length(); i++)
                                        posePart.addDouble(x[i]);

                                    for (size_t i=0; i<o.length(); i++)
                                        posePart.addDouble(o[i]);

                                    Bottle &stampPart=reply.addList();
                                    stampPart.addInt(stamp.getCount());
                                    stampPart.addDouble(stamp.getTime());
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TIP_FRAME:
                        {
                            Vector x,o;
                            if (getTipFrame(x,o))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &tipPart=reply.addList();

                                for (size_t i=0; i<x.length(); i++)
                                    tipPart.addDouble(x[i]);

                                for (size_t i=0; i<o.length(); i++)
                                    tipPart.addDouble(o[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_INFO:
                        {
                            Bottle info;
                            if (getInfo(info))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                reply.addList()=info;
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TWEAK:
                        {
                            Bottle options;
                            if (tweakGet(options))
                            {   
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                reply.addList()=options;
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        default:
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_SET:
            {
                if (command.size()>2)
                    switch (command.get(1).asVocab())
                    {
                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_MODE:
                        {                            
                            int mode=command.get(2).asVocab();
                            bool ret=false;

                            if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_TRACK)
                                ret=setTrackingMode(true);
                            else if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE)
                                ret=setTrackingMode(false);

                            if (ret)
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_REFERENCE:
                        {
                            int mode=command.get(2).asVocab();
                            bool ret=false;

                            if (mode==IKINCARTCTRL_VOCAB_VAL_TRUE)
                                ret=setReferenceMode(true);
                            else if (mode==IKINCARTCTRL_VOCAB_VAL_FALSE)
                                ret=setReferenceMode(false);

                            if (ret)
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_PRIO:
                        {
                            ConstString priority=command.get(2).asString();
                            if (setPosePriority(priority))
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TIME:
                        {
                            if (setTrajTime(command.get(2).asDouble()))
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TOL:
                        {
                            if (setInTargetTol(command.get(2).asDouble()))
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_LIM:
                        {                            
                            if (command.size()>4)
                            {
                                int axis=command.get(2).asInt();
                                double min=command.get(3).asDouble();
                                double max=command.get(4).asDouble();

                                if (setLimits(axis,min,max))
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_DOF:
                        {
                            if (Bottle *b=command.get(2).asList())
                            {
                                Vector newDof(b->size());
                                Vector curDof;

                                for (int i=0; i<b->size(); i++)
                                    newDof[i]=b->get(i).asDouble();

                                if (setDOF(newDof,curDof))
                                {                                                                        
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    Bottle &dofPart=reply.addList();

                                    for (size_t i=0; i<curDof.length(); i++)
                                        dofPart.addInt((int)curDof[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_REST_POS:
                        {
                            if (Bottle *b=command.get(2).asList())
                            {
                                Vector newRestPos(b->size());
                                Vector curRestPos;
    
                                for (int i=0; i<b->size(); i++)
                                    newRestPos[i]=b->get(i).asDouble();
    
                                if (setRestPos(newRestPos,curRestPos))
                                {                                                                        
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    Bottle &restPart=reply.addList();
    
                                    for (size_t i=0; i<curRestPos.length(); i++)
                                        restPart.addDouble(curRestPos[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_REST_WEIGHTS:
                        {
                            if (Bottle *b=command.get(2).asList())
                            {
                                Vector newRestWeights(b->size());
                                Vector curRestWeights;
    
                                for (int i=0; i<b->size(); i++)
                                    newRestWeights[i]=b->get(i).asDouble();
    
                                if (setRestWeights(newRestWeights,curRestWeights))
                                {                                                                        
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    Bottle &restPart=reply.addList();
    
                                    for (size_t i=0; i<curRestWeights.length(); i++)
                                        restPart.addDouble(curRestWeights[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            
    
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TIP_FRAME:
                        {
                            if (Bottle *b=command.get(2).asList())
                            {
                                if (b->size()>=7)
                                {
                                    Vector x(3);
                                    Vector o(b->size()-x.length());

                                    for (size_t i=0; i<x.length(); i++)
                                        x[i]=b->get(i).asDouble();

                                    for (size_t i=0; i<o.length(); i++)
                                        o[i]=b->get(i+x.length()).asDouble();

                                    if (attachTipFrame(x,o))
                                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    else
                                        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            

                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_TWEAK:
                        {
                            if (Bottle *options=command.get(2).asList())
                            {
                                if (tweakSet(*options))
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        default:
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            //-----------------
            case IKINCARTCTRL_VOCAB_CMD_EVENT:
            {
                if (command.size()>2)
                    switch (command.get(1).asVocab())
                    {
                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_REGISTER:
                        {
                            int mode=command.get(2).asVocab();
                            if ((mode==IKINCARTCTRL_VOCAB_VAL_EVENT_ONGOING) &&
                                (command.size()>3))
                            {
                                double checkPoint=command.get(3).asDouble();
                                if (registerMotionOngoingEvent(checkPoint))
                                {
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                    break;
                                }
                            }

                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_UNREGISTER:
                        {
                            int mode=command.get(2).asVocab();
                            if ((mode==IKINCARTCTRL_VOCAB_VAL_EVENT_ONGOING) &&
                                (command.size()>3))
                            {
                                double checkPoint=command.get(3).asDouble();
                                if (unregisterMotionOngoingEvent(checkPoint))
                                {
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                    break;
                                }
                            }

                            reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            break;
                        }

                        //-----------------
                        case IKINCARTCTRL_VOCAB_OPT_LIST:
                        {
                            int mode=command.get(2).asVocab();
                            if (mode==IKINCARTCTRL_VOCAB_VAL_EVENT_ONGOING)
                            {
                                reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                reply.addList()=listMotionOngoingEvents();
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);

                            break;
                        }

                        //-----------------
                        default:
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            //-----------------
            default:
                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
        }
    else
        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

    return true;
}


/************************************************************************/
void ServerCartesianController::alignJointsBounds()
{
    double min, max; 
    int cnt=0;

    printf("%s: aligning joints bounds ...\n",ctrlName.c_str());
    for (size_t i=0; i<lDsc.size(); i++)
    {
        printf("part #%lu: %s\n",(unsigned long)i,lDsc[i].key.c_str());
        for (int j=0; j<lJnt[i]; j++)
        {
            lLim[i]->getLimits(lRmp[i][j],&min,&max);
            printf("joint #%d: [%g, %g] deg\n",cnt,min,max);
            (*chainState)[cnt].setMin(CTRL_DEG2RAD*min);
            (*chainState)[cnt].setMax(CTRL_DEG2RAD*max);
            (*chainPlan)[cnt].setMin(CTRL_DEG2RAD*min);
            (*chainPlan)[cnt].setMax(CTRL_DEG2RAD*max);
            cnt++;
        }
    }
}


/************************************************************************/
double ServerCartesianController::getFeedback(Vector &_fb)
{
    Vector fbTmp(maxPartJoints);
    Vector stamps(maxPartJoints);
    int chainCnt=0;
    int _fbCnt=0;
    double timeStamp=-1.0;

    for (int i=0; i<numDrv; i++)
    {
        bool ok;

        if (useReferences)
            ok=lPid[i]->getReferences(fbTmp.data());
        else if (encTimedEnabled)
        {
            ok=lEnt[i]->getEncodersTimed(fbTmp.data(),stamps.data());
            timeStamp=std::max(timeStamp,findMax(stamps.subVector(0,lJnt[i]-1)));
        }
        else
            ok=lEnc[i]->getEncoders(fbTmp.data());

        if (ok)
        {
            for (int j=0; j<lJnt[i]; j++)
            {
                double tmp=CTRL_DEG2RAD*fbTmp[lRmp[i][j]];
            
                if ((*chainState)[chainCnt].isBlocked())
                {
                    chainState->setBlockingValue(chainCnt,tmp);
                    chainPlan->setBlockingValue(chainCnt,tmp);
                }
                else
                    _fb[_fbCnt++]=tmp;
            
                chainCnt++;
            }
        }
        else for (int j=0; j<lJnt[i]; j++)
        {
            if (!(*chainState)[chainCnt++].isBlocked())
                _fbCnt++;
        }
    }

    return timeStamp;
}


/************************************************************************/
void ServerCartesianController::createController()
{
    // guard
    if (chainState==NULL)
        return;

    stopControlHelper();

    // destroy old controller
    delete ctrl;

    // update quantities
    fb.resize(chainState->getDOF());
    getFeedback(fb);
    chainState->setAng(fb);
    chainPlan->setAng(fb);
    velCmd.resize(chainState->getDOF(),0.0);
    xdes=chainState->EndEffPose();
    qdes=chainState->getAng();
    q0=qdes;

    // instantiate new controller
    if (posDirectEnabled)
        ctrl=new MultiRefMinJerkCtrl(*chainPlan,ctrlPose,getRate()/1000.0);
    else if (plantModelProperties.check("plant_compensator",Value("off")).asString()=="on")
    {
        ctrl=new MultiRefMinJerkCtrl(*chainState,ctrlPose,getRate()/1000.0,true);
        ctrl->setPlantParameters(plantModelProperties,"joint");
    }
    else
        ctrl=new MultiRefMinJerkCtrl(*chainState,ctrlPose,getRate()/1000.0);

    // set tolerance
    ctrl->setInTargetTol(targetTol);

    // set task execution time
    trajTime=ctrl->set_execTime(trajTime,true);

    // configure the Smith Predictor
    if (!posDirectEnabled)
        smithPredictor.configure(plantModelProperties,*chainState); 
}


/************************************************************************/
bool ServerCartesianController::getNewTarget()
{
    if (Bottle *b1=portSlvIn.read(false))
    {
        bool tokened=getTokenOption(*b1,&rxToken);

        // token shall be not greater than the trasmitted one
        if (tokened && (rxToken>txToken))
        {
            printf("%s warning: skipped message from solver due to invalid token (rx=%g)>(thr=%g)\n",
                   ctrlName.c_str(),rxToken,txToken);

            return false;
        }

        // if we stopped the controller then we skip
        // any message with token smaller than the threshold
        if (skipSlvRes)
        {
            if (tokened && !trackingMode && (rxToken<=txTokenLatchedStopControl))
            {
                printf("%s warning: skipped message from solver since controller has been stopped (rx=%g)<=(thr=%g)\n",
                       ctrlName.c_str(),rxToken,txTokenLatchedStopControl);

                return false;
            }
            else
                skipSlvRes=false;
        }

        bool isNew=false;
        Vector _xdes, _qdes;

        if (b1->check(Vocab::decode(IKINSLV_VOCAB_OPT_X)))
        {
            Bottle *b2=getEndEffectorPoseOption(*b1);
            int l1=b2->size();
            int l2=7;
            int len=l1<l2 ? l1 : l2;
            _xdes.resize(len);

            for (int i=0; i<len; i++)
                _xdes[i]=b2->get(i).asDouble();

            if (!(_xdes==xdes))
                isNew=true;
        }

        if (b1->check(Vocab::decode(IKINSLV_VOCAB_OPT_Q)))
        {
            Bottle *b2=getJointsOption(*b1);
            int l1=b2->size();
            int l2=chainState->getDOF();
            int len=l1<l2 ? l1 : l2;
            _qdes.resize(len);

            for (int i=0; i<len; i++)
                _qdes[i]=CTRL_DEG2RAD*b2->get(i).asDouble();

            if (_qdes.length()!=ctrl->get_dim())
            {    
                printf("%s warning: skipped message from solver since does not match the controller dimension (qdes=%d)!=(ctrl=%d)\n",
                       ctrlName.c_str(),(int)_qdes.length(),ctrl->get_dim());

                return false;
            }
            else if (!(_qdes==qdes))
                isNew=true;
        }

        // update target
        if (isNew)
        {
            xdes=_xdes;
            qdes=_qdes;
        }

        // wake up rpc
        if (tokened && syncEventEnabled && (rxToken>=txTokenLatchedGoToRpc))
        {
            syncEventEnabled=false;
            syncEvent.signal();
        }

        return isNew;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::areJointsHealthyAndSet(VectorOf<int> &jointsToSet)
{    
    VectorOf<int> modes(maxPartJoints);
    int chainCnt=0;

    jointsToSet.clear();
    for (int i=0; i<numDrv; i++)
    {
        lMod[i]->getControlModes(modes.getFirst());
        for (int j=0; j<lJnt[i]; j++)
        {
            if (!(*chainState)[chainCnt].isBlocked())
            {
                if ((modes[j]==VOCAB_CM_HW_FAULT) || (modes[j]==VOCAB_CM_IDLE))
                    return false;
                else if (posDirectEnabled)
                {
                    if (modes[j]!=VOCAB_CM_POSITION_DIRECT)
                        jointsToSet.push_back(chainCnt);
                }
                else if (modes[j]!=VOCAB_CM_VELOCITY)
                    jointsToSet.push_back(chainCnt);
            }

            chainCnt++;
        }
    }

    return true;
}


/************************************************************************/
void ServerCartesianController::setJointsCtrlMode(const VectorOf<int> &jointsToSet)
{
    if (jointsToSet.size()==0)
        return;

    int chainCnt=0;
    int k=0;

    for (int i=0; i<numDrv; i++)
    {
        VectorOf<int> joints;
        VectorOf<int> modes;
        for (int j=0; j<lJnt[i]; j++)
        {
            if (chainCnt==jointsToSet[k])
            {       
                joints.push_back(j);
                modes.push_back(posDirectEnabled?VOCAB_CM_POSITION_DIRECT:
                                                 VOCAB_CM_VELOCITY);
                k++;
            }

            chainCnt++;
        }

        if (joints.size()>0)
            lMod[i]->setControlModes(joints.size(),joints.getFirst(),
                                     modes.getFirst());
    }
}


/************************************************************************/
Bottle ServerCartesianController::sendCtrlCmdMultipleJointsPosition()
{
    Bottle info;
    VectorOf<int> joints;
    Vector refs;
    int cnt=0;
    int j=0;
    int k=0;

    info.addString("position");
    info.addString("multiple");

    Vector q=CTRL_RAD2DEG*ctrl->get_q();
    velCmd=CTRL_RAD2DEG*ctrl->get_qdot();
    for (unsigned int i=0; i<chainState->getN(); i++)
    {
        if (!(*chainState)[i].isBlocked())
        {
            int joint=lRmp[j][k];
            double ref=q[cnt];

            joints.push_back(joint);
            refs.push_back(ref);

            ostringstream ss;
            ss<<lDsc[j].key.c_str()<<"_"<<joint;
            info.addString(ss.str().c_str());
            info.addDouble(ref);

            cnt++;
        }

        if (++k>=lJnt[j])
        {
            if (joints.size()>0)
                lPos[j]->setPositions(joints.size(),joints.getFirst(),refs.data());

            joints.clear();
            refs.clear();
            j++;
            k=0;
        }
    }

    return info;
}


/************************************************************************/
Bottle ServerCartesianController::sendCtrlCmdMultipleJointsVelocity()
{
    Bottle info;
    VectorOf<int> joints;
    Vector vels;
    int cnt=0;
    int j=0;
    int k=0;

    info.addString("velocity");
    info.addString("multiple");

    Vector v=CTRL_RAD2DEG*ctrl->get_qdot();
    for (unsigned int i=0; i<chainState->getN(); i++)
    {
        if (!(*chainState)[i].isBlocked())
        {
            int joint=lRmp[j][k];
            double vel=v[cnt];
            double thres=lDsc[j].minAbsVels[k];

            // apply bang-bang control to compensate for unachievable low velocities
            if ((vel!=0.0) && (fabs(vel)<thres))
                vel=iCub::ctrl::sign(qdes[cnt]-fb[cnt])*thres;

            joints.push_back(joint);
            vels.push_back(vel);

            ostringstream ss;
            ss<<lDsc[j].key.c_str()<<"_"<<joint;
            info.addString(ss.str().c_str());
            info.addDouble(vel);

            velCmd[cnt]=vel;
            cnt++;
        }

        if (++k>=lJnt[j])
        {
            if (joints.size()>0)
                lVel[j]->velocityMove(joints.size(),joints.getFirst(),vels.data());

            joints.clear();
            vels.clear();
            j++;
            k=0;
        }
    }

    return info;
}


/************************************************************************/
Bottle ServerCartesianController::sendCtrlCmdSingleJointPosition()
{
    Bottle info;
    int cnt=0;
    int j=0;
    int k=0;

    info.addString("position");
    info.addString("single");

    Vector q=CTRL_RAD2DEG*ctrl->get_q();
    velCmd=CTRL_RAD2DEG*ctrl->get_qdot();
    for (unsigned int i=0; i<chainState->getN(); i++)
    {
        if (!(*chainState)[i].isBlocked())
        {
            int joint=lRmp[j][k];
            double ref=q[cnt];
            lPos[j]->setPosition(joint,ref);

            ostringstream ss;
            ss<<lDsc[j].key.c_str()<<"_"<<joint;
            info.addString(ss.str().c_str());
            info.addDouble(ref);

            cnt++;
        }

        if (++k>=lJnt[j])
        {
            j++;
            k=0;
        }
    }

    return info;
}


/************************************************************************/
Bottle ServerCartesianController::sendCtrlCmdSingleJointVelocity()
{
    Bottle info;
    int cnt=0;
    int j=0;
    int k=0;

    info.addString("velocity");
    info.addString("single");

    Vector v=CTRL_RAD2DEG*ctrl->get_qdot();
    for (unsigned int i=0; i<chainState->getN(); i++)
    {
        if (!(*chainState)[i].isBlocked())
        {
            int joint=lRmp[j][k];
            double vel=v[cnt];
            double thres=lDsc[j].minAbsVels[k];

            // apply bang-bang control to compensate for unachievable low velocities
            if ((vel!=0.0) && (fabs(vel)<thres))
                vel=iCub::ctrl::sign(qdes[cnt]-fb[cnt])*thres;

            lVel[j]->velocityMove(joint,vel);

            ostringstream ss;
            ss<<lDsc[j].key.c_str()<<"_"<<joint;
            info.addString(ss.str().c_str());
            info.addDouble(vel);

            velCmd[cnt]=vel;
            cnt++;
        }

        if (++k>=lJnt[j])
        {
            j++;
            k=0;
        }
    }

    return info;
}


/************************************************************************/
void ServerCartesianController::stopLimb(const bool execStopPosition)
{
    if (!posDirectEnabled || execStopPosition)
    {
        int j=0; int k=0;
        for (unsigned int i=0; i<chainState->getN(); i++)
        {
            if (!(*chainState)[i].isBlocked())
                posDirectEnabled?
                lStp[j]->stop(lRmp[j][k]):
                lVel[j]->velocityMove(lRmp[j][k],0.0);  // vel==0.0 is always achievable

            if (++k>=lJnt[j])
            {
                j++;
                k=0;
            }
        }
    }

    velCmd=0.0;
    xdot_set=0.0;
}


/************************************************************************/
bool ServerCartesianController::threadInit()
{
    printf("Starting %s at %d ms\n",ctrlName.c_str(),(int)getRate());

    return true;
}


/************************************************************************/
void ServerCartesianController::afterStart(bool s)
{
    printf("%s %s\n",ctrlName.c_str(),s?"started successfully":"did not start!");
}


/************************************************************************/
void ServerCartesianController::run()
{    
    if (connected)
    {
        mutex.lock();

        VectorOf<int> jointsToSet;
        jointsHealthy=areJointsHealthyAndSet(jointsToSet);
        if (!jointsHealthy)
            stopControlHelper();        

        string event="none";

        // read the feedback
        double stamp=getFeedback(fb);

        // iff in position then update just chainState
        // and make the chainPlan evolve freely without
        // constraining it with the feedback
        if (posDirectEnabled)
            chainState->setAng(fb);
        else
            ctrl->set_q(fb);

        // manage the virtual target yielded by a
        // request for a task-space reference velocity
        if (jointsHealthy && taskVelModeOn && (++taskRefVelPeriodCnt>=taskRefVelPeriodFactor))
        {
            Vector xdot_set_int=taskRefVelTargetGen->integrate(xdot_set);
            goTo(IKINCTRL_POSE_FULL,xdot_set_int,0.0);
            taskRefVelPeriodCnt=0;
        }

        // get the current target pose
        if (getNewTarget())
        {
            if (jointsHealthy)
            {
                setJointsCtrlMode(jointsToSet); 
                if (!executingTraj)
                {
                    ctrl->restart(fb);
                    smithPredictor.restart(fb);
                }

                // onset of new trajectory
                executingTraj=true;
                event="motion-onset";

                motionOngoingEventsCurrent=motionOngoingEvents;
                q0=fb;
            }
        }

        // update the stamp anyway
        if (stamp>=0.0)
            txInfo.update(stamp);
        else
            txInfo.update();

        if (executingTraj)
        {
            // add the contribution of the Smith Predictor block
            ctrl->add_compensation(-1.0*smithPredictor.computeCmd(ctrl->get_qdot()));

            // limb control loop
            if (taskVelModeOn)
                ctrl->iterate(xdes,qdes,xdot_set);
            else
                ctrl->iterate(xdes,qdes);

            // handle the end-trajectory event
            if (ctrl->isInTarget() && !taskVelModeOn)
            {
                executingTraj=false;
                motionDone   =true;

                stopLimb(false);
                event="motion-done";

                // switch the solver status to one shot mode
                // if that's the case
                if (!trackingMode && (rxToken==txToken))
                    setTrackingModeHelper(false);
            }            
            else
            {
                // send commands to the robot                
                if (debugInfoEnabled && (portDebugInfo.getOutputCount()>0))
                {
                    portDebugInfo.prepare()=(this->*sendCtrlCmd)();
                    debugInfo.update(txInfo.getTime());
                    portDebugInfo.setEnvelope(debugInfo);
                    portDebugInfo.write();
                }
                else
                    (this->*sendCtrlCmd)();
            }
        }        

        // stream out the end-effector pose
        if (portState.getOutputCount()>0)
        {
            portState.prepare()=chainState->EndEffPose();
            portState.setEnvelope(txInfo);
            portState.write();
        }

        if (event=="motion-onset")
            notifyEvent(event);

        motionOngoingEventsHandling();

        if (event=="motion-done")
        {
            motionOngoingEventsFlush();
            notifyEvent(event);
        }

        mutex.unlock();
    }
    else if ((++connectCnt)*getRate()>CARTCTRL_CONNECT_TMO)
    {
        if (connectToSolver())
        {
            createController();

            // reserve id==0 for start-up context
            int id0;
            storeContext(&id0);
        }

        connectCnt=0;
    }
}


/************************************************************************/
void ServerCartesianController::threadRelease()
{
    printf("Stopping %s\n",ctrlName.c_str());

    if (connected)
        stopLimb();

    notifyEvent("closing");
}


/************************************************************************/
bool ServerCartesianController::open(Searchable &config)
{
    printf("***** Configuring cartesian controller *****\n");

    // GENERAL group
    Bottle &optGeneral=config.findGroup("GENERAL");
    if (optGeneral.isNull())
    {
        printf("GENERAL group is missing\n");
        close();

        return false;
    }

    printf("Acquiring options for group GENERAL...\n");

    // scan for required params
    if (optGeneral.check("SolverNameToConnect"))
        slvName=optGeneral.find("SolverNameToConnect").asString();
    else
    {
        printf("SolverNameToConnect option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("KinematicPart"))
    {
        kinPart=optGeneral.find("KinematicPart").asString();
        if ((kinPart!="arm") && (kinPart!="leg") && (kinPart!="custom"))
        {
            printf("Attempt to instantiate an unknown kinematic part\n");
            printf("Available parts are: arm, leg, custom\n");
            close();

            return false;
        }
    }
    else
    {
        printf("KinematicPart option is missing\n");
        close();

        return false;
    }

    string _partKinType;
    if (optGeneral.check("KinematicType"))
    {
        kinType=optGeneral.find("KinematicType").asString();
        string _kinType=kinType.c_str();
        _partKinType=_kinType.substr(0,_kinType.find("_"));

        if ((_partKinType!="left") && (_partKinType!="right"))
        {
            printf("Attempt to instantiate unknown kinematic type\n");
            printf("Available types are: left, right, left_v*, right_v*\n");
            close();

            return false;
        }
    }
    else if (kinPart!="custom")
    {
        printf("KinematicType option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("NumberOfDrivers"))
    {
        numDrv=optGeneral.find("NumberOfDrivers").asInt();
        if (numDrv<=0)
        {
            printf("NumberOfDrivers shall be positive\n");
            close();

            return false;
        }
    }
    else
    {
        printf("NumberOfDrivers option is missing\n");
        close();

        return false;
    }

    // optional params
    if (optGeneral.check("ControllerName"))
        ctrlName=optGeneral.find("ControllerName").asString();
    else
    {
        ctrlName="cartController";
        ctrlName=ctrlName+"/";
        ctrlName=ctrlName+kinPart;
        ctrlName=ctrlName+_partKinType.c_str();
        printf("default ControllerName assumed: %s",ctrlName.c_str());
    }

    if (optGeneral.check("ControllerPeriod"))
        setRate(optGeneral.find("ControllerPeriod").asInt());

    taskRefVelPeriodFactor=optGeneral.check("TaskRefVelPeriodFactor",
                                            Value(CARTCTRL_DEFAULT_TASKVEL_PERFACTOR)).asInt();

    posDirectEnabled=optGeneral.check("PositionControl",
                                      Value(CARTCTRL_DEFAULT_POSCTRL)).asString()=="on";

    multipleJointsControlEnabled=optGeneral.check("MultipleJointsControl",
                                                  Value(CARTCTRL_DEFAULT_MULJNTCTRL)).asString()=="on";

    debugInfoEnabled=optGeneral.check("DebugInfo",Value("off")).asString()=="on";

    // scan DRIVER groups
    for (int i=0; i<numDrv; i++)
    {
        char entry[255];
        sprintf(entry,"DRIVER_%d",i);

        Bottle &optDrv=config.findGroup(entry);
        if (optDrv.isNull())
        {
            printf("%s group is missing\n",entry);
            close();

            return false;
        }

        printf("Acquiring options for group %s...\n",entry);

        DriverDescriptor desc;

        if (optDrv.check("Key"))
            desc.key=optDrv.find("Key").asString();
        else
        {
            printf("Key option is missing\n");
            close();

            return false;
        }

        if (optDrv.check("JointsOrder"))
        {
            ConstString jointsOrderType=optDrv.find("JointsOrder").asString();

            if (jointsOrderType=="direct")
                desc.jointsDirectOrder=true;
            else if (jointsOrderType=="reversed")
                desc.jointsDirectOrder=false;
            else
            {
                printf("Attempt to select an unknown mapping order\n");
                printf("Available orders are: direct, reversed\n");
                close();

                return false;
            }
        }
        else
        {
            printf("JointsOrder option is missing\n");
            close();

            return false;
        }

        if (Bottle *pMinAbsVelsBottle=optDrv.find("MinAbsVels").asList())
        {
            desc.useDefaultMinAbsVel=false;
            desc.minAbsVels.resize(pMinAbsVelsBottle->size());

            for (int j=0; j<pMinAbsVelsBottle->size(); j++)
            {
                desc.minAbsVels[j]=pMinAbsVelsBottle->get(j).asDouble();

                if (desc.minAbsVels[j]<0.0)
                    desc.minAbsVels[j]=-desc.minAbsVels[j];
            }
        }
        else
        {
            printf("MinAbsVels option is missing ... using default values\n");
            desc.useDefaultMinAbsVel=true;
        }

        lDsc.push_back(desc);
    }

    // acquire options for Plant modeling
    Bottle &optPlantModel=config.findGroup("PLANT_MODEL");
    if (!optPlantModel.isNull())
    {
        printf("PLANT_MODEL group detected\n");
        plantModelProperties.fromString(optPlantModel.toString().c_str());

        // append information about the predictor's period,
        // that must match the controller's period
        plantModelProperties.unput("Ts");
        plantModelProperties.put("Ts",getRate()/1000.0);
    }
    else
        plantModelProperties.clear();

    // instantiate kinematic object
    if (kinPart=="arm")
        limbState=new iCubArm(kinType.c_str());
    else if (kinPart=="leg")
        limbState=new iCubLeg(kinType.c_str());
    else if (optGeneral.check("customKinFile"))     // custom kinematics case
    {
        Property linksOptions;
        linksOptions.fromConfigFile(optGeneral.find("customKinFile").asString().c_str());

        limbState=new iKinLimb(linksOptions);
        if (!limbState->isValid())
        {
            printf("Invalid links parameters\n");
            close();

            return false;
        }
    }
    else
    {
        printf("customKinFile option is missing\n");
        close();

        return false;
    }

    Property DHTable; limbState->toLinksProperties(DHTable);
    printf("DH Table: %s\n",DHTable.toString().c_str());

    // duplicate the limb for planning purpose
    limbPlan=new iKinLimb(*limbState);

    chainState=limbState->asChain();
    chainPlan=limbPlan->asChain();

    openPorts();

    return true;
}


/************************************************************************/
bool ServerCartesianController::close()
{
    if (closed)
        return true;

    detachAll();

    for (unsigned int i=0; i<lRmp.size(); i++)
        delete[] lRmp[i];

    lDsc.clear();
    lMod.clear();
    lEnc.clear();
    lEnt.clear();
    lPid.clear();
    lLim.clear();
    lVel.clear();
    lPos.clear();
    lStp.clear();
    lJnt.clear();
    lRmp.clear();

    delete limbState;
    delete limbPlan;
    delete ctrl;

    while (eventsMap.size()>0)
        unregisterEvent(*eventsMap.begin()->second);

    closePorts();

    contextMap.clear();

    return closed=true;
}


/************************************************************************/
bool ServerCartesianController::attachAll(const PolyDriverList &p)
{
    drivers=p;
    int nd=drivers.size();

    printf("***** Attaching drivers to cartesian controller %s *****\n",ctrlName.c_str());
    printf("Received list of %d driver(s)\n",nd);

    if (nd!=numDrv)
    {
        printf("Expected list of %d driver(s)\n",numDrv);
        return false;
    }

    int remainingJoints=chainState->getN();

    for (int i=0; i<numDrv; i++)
    {
        printf("Acquiring info on driver %s... ",lDsc[i].key.c_str());

        // check if what we require is present within the given list
        int j;
        for (j=0; j<drivers.size(); j++)
            if (lDsc[i].key==drivers[j]->key)
                break;

        if (j>=drivers.size())
        {
            printf("None of provided drivers is of type %s\n",lDsc[i].key.c_str());
            return false;
        }

        // acquire interfaces and driver's info
        encTimedEnabled=pidAvailable=posDirectAvailable=true;
        if (drivers[j]->poly->isValid())
        {
            printf("ok\n");

            IControlMode2     *mod;
            IEncoders         *enc;
            IEncodersTimed    *ent;
            IPidControl       *pid;
            IControlLimits    *lim;
            IVelocityControl2 *vel;
            IPositionDirect   *pos;
            IPositionControl  *stp;
            int                joints;

            drivers[j]->poly->view(mod);
            drivers[j]->poly->view(enc);
            drivers[j]->poly->view(lim);
            drivers[j]->poly->view(vel);
            drivers[j]->poly->view(stp);
            encTimedEnabled&=drivers[j]->poly->view(ent);
            pidAvailable&=drivers[j]->poly->view(pid);
            posDirectAvailable&=drivers[j]->poly->view(pos);

            enc->getAxes(&joints);

            // this is for allocating vector
            // to read data from the interface
            if (joints>maxPartJoints)
                maxPartJoints=joints;

            // handle chain's bounds
            if (joints>remainingJoints)
                joints=remainingJoints;

            remainingJoints-=joints;

            int *rmpTmp=new int[joints];
            for (int k=0; k<joints; k++)
                rmpTmp[k]=lDsc[i].jointsDirectOrder ? k : (joints-k-1);

            // prepare references for minimum achievable absolute velocities
            if (lDsc[i].useDefaultMinAbsVel)
                lDsc[i].minAbsVels.resize(joints,0.0);
            else if (lDsc[i].minAbsVels.length()<(size_t)joints)
            {
                Vector tmp=lDsc[i].minAbsVels;
                lDsc[i].minAbsVels.resize(joints,0.0);

                for (size_t k=0; k<tmp.length(); k++)
                    lDsc[i].minAbsVels[k]=tmp[k];
            }

            lMod.push_back(mod);
            lEnc.push_back(enc);
            lEnt.push_back(ent);
            lPid.push_back(pid);
            lLim.push_back(lim);
            lVel.push_back(vel);
            lPos.push_back(pos);
            lStp.push_back(stp);
            lJnt.push_back(joints);
            lRmp.push_back(rmpTmp);
        }
        else
        {
            printf("error\n");
            return false;
        }
    }

    printf("%s: %s interface will be used\n",ctrlName.c_str(),
           encTimedEnabled?"IEncodersTimed":"IEncoders");

    printf("%s: IPidControl %s\n",ctrlName.c_str(),
           pidAvailable?"available":"not available");

    printf("%s: IPositionDirect %s\n",ctrlName.c_str(),
           posDirectAvailable?"available":"not available");

    posDirectEnabled&=posDirectAvailable;
    printf("%s: %s interface will be used\n",ctrlName.c_str(),
           posDirectEnabled?"IPositionDirect":"IVelocityControl");

    printf("%s: %s control will be used\n",ctrlName.c_str(),
           multipleJointsControlEnabled?"multiple joints":"single joint");

    if (multipleJointsControlEnabled)
    {
        if (posDirectEnabled)
            sendCtrlCmd=&ServerCartesianController::sendCtrlCmdMultipleJointsPosition;
        else
            sendCtrlCmd=&ServerCartesianController::sendCtrlCmdMultipleJointsVelocity;
    }
    else if (posDirectEnabled)
        sendCtrlCmd=&ServerCartesianController::sendCtrlCmdSingleJointPosition;
    else
        sendCtrlCmd=&ServerCartesianController::sendCtrlCmdSingleJointVelocity;

    alignJointsBounds();

    // exclude acceleration constraints by fixing
    // thresholds at high values
    for (int i=0; i<numDrv; i++)
        for (int j=0; j<lJnt[i]; j++)
            lVel[i]->setRefAcceleration(j,CARTCTRL_MAX_ACCEL);

    // init task-space reference velocity
    xdot_set.resize(7,0.0);

    // this line shall be put before any
    // call to attached-dependent methods
    attached=true;

    connectToSolver();

    // create controller
    createController();

    // reserve id==0 for start-up context
    int id0;
    storeContext(&id0);

    // create the target generator for
    // task-space reference velocity
    taskRefVelTargetGen=new Integrator(taskRefVelPeriodFactor*(getRate()/1000.0),ctrl->get_x());
    taskRefVelPeriodCnt=0;

    start();

    return true;
}


/************************************************************************/
bool ServerCartesianController::detachAll()
{
    if (isRunning())
        stop();

    return true;
}


/************************************************************************/
bool ServerCartesianController::pingSolver()
{    
    ConstString portSlvName="/";
    portSlvName=portSlvName+slvName+"/in";

    printf("%s: Checking if cartesian solver %s is alive... ",ctrlName.c_str(),slvName.c_str());

    bool ok=Network::exists(portSlvName.c_str(),true);

    printf("%s\n",ok?"ready":"not yet");

    return ok;
}


/************************************************************************/
bool ServerCartesianController::connectToSolver()
{
    if (attached && !connected && pingSolver())
    {        
        printf("%s: Connecting to cartesian solver %s...\n",ctrlName.c_str(),slvName.c_str());

        ConstString portSlvName="/";
        portSlvName=portSlvName+slvName;

        bool ok=true;

        ok&=Network::connect((portSlvName+"/out").c_str(),portSlvIn.getName().c_str(),"udp");
        ok&=Network::connect(portSlvOut.getName().c_str(),(portSlvName+"/in").c_str(),"udp");
        ok&=Network::connect(portSlvRpc.getName().c_str(),(portSlvName+"/rpc").c_str());

        if (ok)
            printf("%s: Connections established with %s\n",ctrlName.c_str(),slvName.c_str());
        else
        {
            printf("%s: Problems detected while connecting to %s\n",ctrlName.c_str(),slvName.c_str());
            return false;
        }

        // this line shall be put before any
        // call to connected-dependent methods
        connected=true;

        // keep solver and controller status aligned at start-up
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_DOF);

        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
            return false;
        }

        // update chain's links
        // skip the first ack/nack vocab
        Bottle *rxDofPart=reply.get(1).asList();
        for (int i=0; i<rxDofPart->size(); i++)
        {
            if (rxDofPart->get(i).asInt()!=0)
            {
                chainState->releaseLink(i);
                chainPlan->releaseLink(i);
            }
            else
            {
                chainState->blockLink(i);
                chainPlan->blockLink(i);
            }
        }

        setTrackingMode(trackingMode);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goTo(unsigned int _ctrlPose, const Vector &xd,
                                     const double t, const bool latchToken)
{    
    if (connected && (ctrl->get_dim()!=0) && jointsHealthy)
    {
        motionDone=false;
        
        ctrl->set_ctrlPose(ctrlPose=_ctrlPose);

        // update trajectory execution time just if required
        if (t>0.0)
            setTrajTimeHelper(t);

        Bottle &b=portSlvOut.prepare();
        b.clear();
    
        // xd part
        addTargetOption(b,xd);
        // pose part
        addPoseOption(b,ctrlPose);
        // always put solver in continuous mode
        // before commanding a new desired pose
        // in order to compensate for movements
        // of uncontrolled joints
        // correct solver status will be reinstated
        // accordingly at the end of trajectory
        addModeOption(b,true);
        // token part
        addTokenOption(b,txToken=Time::now());

        skipSlvRes=false;
        if (latchToken)
            txTokenLatchedGoToRpc=txToken;

        portSlvOut.writeStrict();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTrackingModeHelper(const bool f)
{
    if (connected)
    {
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_MODE);
        if (f)
            command.addInt(IKINSLV_VOCAB_VAL_MODE_TRACK);
        else
            command.addInt(IKINSLV_VOCAB_VAL_MODE_SINGLE);

        // send command to solver and wait for reply
        bool ret=false;
        if (!portSlvRpc.write(command,reply))
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
        else if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
        {
            trackingMode=f;
            ret=true;
        }

        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTrackingMode(const bool f)
{
    mutex.lock();
    bool ret=setTrackingModeHelper(f);
    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::getTrackingMode(bool *f)
{
    if (attached && (f!=NULL))
    {
        *f=trackingMode;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setReferenceMode(const bool f)
{
    if (attached && pidAvailable)
    {
        useReferences=f;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getReferenceMode(bool *f)
{
    if (attached && (f!=NULL))
    {
        *f=useReferences;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setPosePriority(const ConstString &p)
{
    bool ret=false;
    if (connected && ((p=="position") || (p=="orientation")))
    {
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_PRIO);
        command.add(p=="position"?IKINSLV_VOCAB_VAL_PRIO_XYZ:IKINSLV_VOCAB_VAL_PRIO_ANG);

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::getPosePriority(ConstString &p)
{
    bool ret=false;
    if (connected)
    {
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_PRIO);

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
        {
            if (ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK))
                p=(reply.get(1).asVocab()==IKINSLV_VOCAB_VAL_PRIO_XYZ)?
                  "position":"orientation";
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::getPose(Vector &x, Vector &o, Stamp *stamp)
{
    if (attached)
    {
        mutex.lock();
        Vector pose=chainState->EndEffPose();
    
        x.resize(3);
        o.resize(pose.length()-x.length());
    
        for (size_t i=0; i<x.length(); i++)
            x[i]=pose[i];
    
        for (size_t i=0; i<o.length(); i++)
            o[i]=pose[x.length()+i];

        if (stamp!=NULL)
            *stamp=txInfo;
    
        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getPose(const int axis, Vector &x,
                                        Vector &o, Stamp *stamp)
{
    if (attached)
    {
        mutex.lock();

        bool ret=false;
        if (axis<(int)chainState->getN())
        {
            Matrix H=chainState->getH(axis,true);
    
            x.resize(3);
            for (size_t i=0; i<x.length(); i++)
                x[i]=H(i,3);
    
            o=dcm2axis(H);

            poseInfo.update(txInfo.getTime());
            if (stamp!=NULL)
                *stamp=poseInfo;

            ret=true;
        }

        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPose(const Vector &xd, const Vector &od,
                                         const double t)
{
    if (connected)
    {
        mutex.lock();

        Vector _xd(xd.length()+od.length());
        for (size_t i=0; i<xd.length(); i++)
            _xd[i]=xd[i];
    
        for (size_t i=0; i<od.length(); i++)
            _xd[xd.length()+i]=od[i];

        taskVelModeOn=false;
        bool ret=goTo(IKINCTRL_POSE_FULL,_xd,t);

        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPosition(const Vector &xd, const double t)
{
    if (connected)
    {
        mutex.lock();

        taskVelModeOn=false;
        bool ret=goTo(IKINCTRL_POSE_XYZ,xd,t);

        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPoseSync(const Vector &xd, const Vector &od,
                                             const double t)
{
    return goToPose(xd,od,t);
}


/************************************************************************/
bool ServerCartesianController::goToPositionSync(const Vector &xd, const double t)
{
    return goToPosition(xd,t);
}


/************************************************************************/
bool ServerCartesianController::getDesired(Vector &xdhat, Vector &odhat,
                                           Vector &qdhat)
{
    if (connected)
    {
        mutex.lock();

        xdhat.resize(3);
        odhat.resize(xdes.length()-3);

        for (size_t i=0; i<xdhat.length(); i++)
            xdhat[i]=xdes[i];

        for (size_t i=0; i<odhat.length(); i++)
            odhat[i]=xdes[xdhat.length()+i];

        qdhat.resize(chainState->getN());
        int cnt=0;

        for (unsigned int i=0; i<chainState->getN(); i++)
        {
            if ((*chainState)[i].isBlocked())
                qdhat[i]=CTRL_RAD2DEG*chainState->getAng(i);
            else
                qdhat[i]=CTRL_RAD2DEG*qdes[cnt++];
        }

        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::askForPose(const Vector &xd, const Vector &od,
                                           Vector &xdhat, Vector &odhat,
                                           Vector &qdhat)
{
    if (!connected)
        return false;

    mutex.lock();

    Bottle command, reply;
    Vector tg(xd.length()+od.length());
    for (size_t i=0; i<xd.length(); i++)
        tg[i]=xd[i];

    for (size_t i=0; i<od.length(); i++)
        tg[xd.length()+i]=od[i];
    
    command.addVocab(IKINSLV_VOCAB_CMD_ASK);
    addVectorOption(command,IKINSLV_VOCAB_OPT_XD,tg);
    addPoseOption(command,IKINCTRL_POSE_FULL);

    // send command and wait for reply
    bool ret=false;
    if (portSlvRpc.write(command,reply))
        ret=getDesiredOption(reply,xdhat,odhat,qdhat);
    else
        printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());

    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::askForPose(const Vector &q0, const Vector &xd,
                                           const Vector &od, Vector &xdhat,
                                           Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    mutex.lock();

    Bottle command, reply;
    Vector tg(xd.length()+od.length());
    for (size_t i=0; i<xd.length(); i++)
        tg[i]=xd[i];

    for (size_t i=0; i<od.length(); i++)
        tg[xd.length()+i]=od[i];

    command.addVocab(IKINSLV_VOCAB_CMD_ASK);
    addVectorOption(command,IKINSLV_VOCAB_OPT_XD,tg);
    addVectorOption(command,IKINSLV_VOCAB_OPT_Q,q0);
    addPoseOption(command,IKINCTRL_POSE_FULL);

    // send command and wait for reply
    bool ret=false;
    if (portSlvRpc.write(command,reply))
        ret=getDesiredOption(reply,xdhat,odhat,qdhat);
    else
        printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());

    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::askForPosition(const Vector &xd, Vector &xdhat,
                                               Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    mutex.lock();

    Bottle command, reply;
    command.addVocab(IKINSLV_VOCAB_CMD_ASK);
    addVectorOption(command,IKINSLV_VOCAB_OPT_XD,xd);
    addPoseOption(command,IKINCTRL_POSE_XYZ);

    // send command and wait for reply
    bool ret=false;
    if (portSlvRpc.write(command,reply))
        ret=getDesiredOption(reply,xdhat,odhat,qdhat);
    else
        printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());

    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::askForPosition(const Vector &q0, const Vector &xd,
                                               Vector &xdhat, Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    mutex.lock();

    Bottle command, reply;
    command.addVocab(IKINSLV_VOCAB_CMD_ASK);
    addVectorOption(command,IKINSLV_VOCAB_OPT_XD,xd);
    addVectorOption(command,IKINSLV_VOCAB_OPT_Q,q0);
    addPoseOption(command,IKINCTRL_POSE_XYZ);

    // send command and wait for reply
    bool ret=false;
    if (portSlvRpc.write(command,reply))
        ret=getDesiredOption(reply,xdhat,odhat,qdhat);
    else
        printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());

    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::getDOF(Vector &curDof)
{
    if (connected)
    {
        mutex.lock();

        curDof.resize(chainState->getN());
        for (unsigned int i=0; i<chainState->getN(); i++)
            curDof[i]=!(*chainState)[i].isBlocked();
    
        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setDOF(const Vector &newDof, Vector &curDof)
{
    if (connected)
    {
        mutex.lock();

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_DOF);
        Bottle &txDofPart=command.addList();
        for (size_t i=0; i<newDof.length(); i++)
            txDofPart.addInt((int)newDof[i]);
    
        // send command to solver and wait for reply
        bool ret=false;
        if (portSlvRpc.write(command,reply))
        {
            // update chain's links
            // skip the first ack/nack vocab
            Bottle *rxDofPart=reply.get(1).asList();
            curDof.resize((size_t)rxDofPart->size());
            for (int i=0; i<rxDofPart->size(); i++)
            {
                curDof[i]=rxDofPart->get(i).asInt();
                if (curDof[i]!=0.0)
                {
                    chainState->releaseLink(i);
                    chainPlan->releaseLink(i);
                }
                else
                {
                    chainState->blockLink(i);
                    chainPlan->blockLink(i);
                }
            }

            // update controller
            createController();
            ret=true;
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());

        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getRestPos(Vector &curRestPos)
{
    if (connected)
    {
        mutex.lock();

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_POS);
    
        // send command to solver and wait for reply
        bool ret=false;
        if (portSlvRpc.write(command,reply))
        {
            Bottle *rxRestPart=reply.get(1).asList();
            curRestPos.resize(rxRestPart->size());
            for (int i=0; i<rxRestPart->size(); i++)
                curRestPos[i]=rxRestPart->get(i).asDouble();

            ret=true;            
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());

        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setRestPos(const Vector &newRestPos, Vector &curRestPos)
{
    if (connected)
    {
        mutex.lock();

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_POS);
        Bottle &txRestPart=command.addList();
        for (size_t i=0; i<newRestPos.length(); i++)
            txRestPart.addDouble(newRestPos[i]);
    
        // send command to solver and wait for reply
        bool ret=false;
        if (portSlvRpc.write(command,reply))
        {
            Bottle *rxRestPart=reply.get(1).asList();
            curRestPos.resize(rxRestPart->size());
            for (int i=0; i<rxRestPart->size(); i++)
                curRestPos[i]=rxRestPart->get(i).asDouble();

            ret=true;            
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
            
        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getRestWeights(Vector &curRestWeights)
{
    if (connected)
    {
        mutex.lock();

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_WEIGHTS);
    
        // send command to solver and wait for reply
        bool ret=false;
        if (portSlvRpc.write(command,reply))
        {
            Bottle *rxRestPart=reply.get(1).asList();
            curRestWeights.resize(rxRestPart->size());
            for (int i=0; i<rxRestPart->size(); i++)
                curRestWeights[i]=rxRestPart->get(i).asDouble();

            ret=true;            
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
            
        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setRestWeights(const Vector &newRestWeights,
                                               Vector &curRestWeights)
{
    if (connected)
    {
        mutex.lock();

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_WEIGHTS);
        Bottle &txRestPart=command.addList();
        for (size_t i=0; i<newRestWeights.length(); i++)
            txRestPart.addDouble(newRestWeights[i]);
    
        // send command to solver and wait for reply
        bool ret=false;
        if (portSlvRpc.write(command,reply))
        {
            Bottle *rxRestPart=reply.get(1).asList();
            curRestWeights.resize(rxRestPart->size());
            for (int i=0; i<rxRestPart->size(); i++)
                curRestWeights[i]=rxRestPart->get(i).asDouble();

            ret=true;
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
            
        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getLimits(const int axis, double *min, double *max)
{
    bool ret=false;
    if (connected && (min!=NULL) && (max!=NULL))
    {
        mutex.lock();
        if (axis<(int)chainState->getN())
        {
            Bottle command, reply;
            command.addVocab(IKINSLV_VOCAB_CMD_GET);
            command.addVocab(IKINSLV_VOCAB_OPT_LIM);
            command.addInt(axis);

            // send command to solver and wait for reply
            if (!portSlvRpc.write(command,reply))
                printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
            else if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
            {
                *min=reply.get(1).asDouble();
                *max=reply.get(2).asDouble();                        
                ret=true;
            }
        }
        mutex.unlock();
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::setLimits(const int axis, const double min,
                                          const double max)
{
    bool ret=false;
    if (connected)
    {
        mutex.lock();

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_LIM);
        command.addInt(axis);
        command.addDouble(min);
        command.addDouble(max);

        // send command to solver and wait for reply        
        if (!portSlvRpc.write(command,reply))
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
        else
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);

        mutex.unlock();
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::getTrajTime(double *t)
{
    if (attached && (t!=NULL))
    {
        *t=trajTime;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTrajTimeHelper(const double t)
{
    if (attached)
    {
        trajTime=ctrl->set_execTime(t,true);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTrajTime(const double t)
{
    mutex.lock();
    bool ret=setTrajTimeHelper(t);
    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::getInTargetTol(double *tol)
{
    if (attached && (tol!=NULL))
    {
        *tol=targetTol;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setInTargetTolHelper(const double tol)
{
    if (attached)
    {
        ctrl->setInTargetTol(tol);
        targetTol=ctrl->getInTargetTol();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setInTargetTol(const double tol)
{
    mutex.lock();
    bool ret=setInTargetTolHelper(tol);
    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::getJointsVelocities(Vector &qdot)
{
    if (connected)
    {
        mutex.lock();
        qdot=velCmd;
        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getTaskVelocities(Vector &xdot, Vector &odot)
{
    if (connected)
    {
        mutex.lock();

        Matrix J=ctrl->get_J();
        Vector taskVel(7,0.0);

        if ((J.rows()>0) && (J.cols()==velCmd.length()))
        {
            taskVel=J*(CTRL_DEG2RAD*velCmd);

            Vector _odot=taskVel.subVector(3,taskVel.length()-1);
            double thetadot=norm(_odot);
            if (thetadot>0.0)
                _odot=(1.0/thetadot)*_odot;

            taskVel[3]=_odot[0];
            taskVel[4]=_odot[1];
            taskVel[5]=_odot[2];
            taskVel.push_back(thetadot);
        }

        xdot.resize(3);
        odot.resize(taskVel.length()-xdot.length());
    
        for (size_t i=0; i<xdot.length(); i++)
            xdot[i]=taskVel[i];
    
        for (size_t i=0; i<odot.length(); i++)
            odot[i]=taskVel[xdot.length()+i];

        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTaskVelocities(const Vector &xdot,
                                                  const Vector &odot)
{
    if (connected)
    {
        mutex.lock();

        for (int i=0; i<3; i++)
            xdot_set[i]=xdot[i];

        for (size_t i=3; i<xdot_set.length(); i++)
            xdot_set[i]=odot[i-3];

        if (norm(xdot_set)>0.0)
        {
            if (!taskVelModeOn)
            {
                taskRefVelTargetGen->reset(chainState->EndEffPose());
                taskRefVelPeriodCnt=0;
            }

            taskVelModeOn=true;
        }
        else
            stopControlHelper();
        
        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::attachTipFrame(const Vector &x, const Vector &o)
{
    if (connected && (x.length()>=3) || (o.length()>=4))
    {
        mutex.lock();

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_TIP_FRAME);
        Bottle &txTipPart=command.addList();

        for (int i=0; i<3; i++)
            txTipPart.addDouble(x[i]);

        for (int i=0; i<4; i++)
            txTipPart.addDouble(o[i]);

        // send command to solver and wait for reply
        bool ret=false;
        if (portSlvRpc.write(command,reply))
        {
            if (ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK))
            {
                Matrix HN=axis2dcm(o);
                HN(0,3)=x[0];
                HN(1,3)=x[1];
                HN(2,3)=x[2];
                chainState->setHN(HN);
                chainPlan->setHN(HN);
            }
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());

        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getTipFrame(Vector &x, Vector &o)
{
    if (connected)
    {
        mutex.lock();

        Matrix HN=chainState->getHN();

        x=HN.getCol(3);
        x.pop_back();

        o=dcm2axis(HN);

        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::removeTipFrame()
{
    return attachTipFrame(Vector(3,0.0),Vector(4,0.0));
}


/************************************************************************/
bool ServerCartesianController::checkMotionDone(bool *f)
{
    if (attached && (f!=NULL))
    {
        *f=motionDone;
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::waitMotionDone(const double period,
                                               const double timeout)
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
bool ServerCartesianController::stopControlHelper()
{
    if (connected)
    {
        if (executingTraj)
        {
            executingTraj=false;
            taskVelModeOn=false;
            motionDone   =true;

            stopLimb();

            txTokenLatchedStopControl=txToken;
            skipSlvRes=true;

            notifyEvent("motion-done");

            // switch the solver status to one shot mode
            // if that's the case
            if (!trackingMode)
                setTrackingModeHelper(false);
        }

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::stopControl()
{
    mutex.lock();
    bool ret=stopControlHelper();
    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::storeContext(int *id)
{
    if (connected && (id!=NULL))
    {
        mutex.lock();
        unsigned int N=chainState->getN();
        mutex.unlock();

        Vector _dof,_restPos,_restWeights;
        Vector _tip_x,_tip_o;

        getDOF(_dof);
        getRestPos(_restPos);
        getRestWeights(_restWeights);
        getTipFrame(_tip_x,_tip_o);

        Matrix _limits(N,2);
        for (unsigned int axis=0; axis<N; axis++)
        {
            double min,max;
            getLimits(axis,&min,&max);
            _limits(axis,0)=min;
            _limits(axis,1)=max;
        }

        double _trajTime,_tol;
        bool _mode,_useReference;

        getTrajTime(&_trajTime);
        getInTargetTol(&_tol);
        getTrackingMode(&_mode);
        getReferenceMode(&_useReference);

        mutex.lock();

        Context &context=contextMap[contextIdCnt];
        context.dof=_dof;
        context.restPos=_restPos;
        context.restWeights=_restWeights;
        context.limits=_limits;
        context.tip_x=_tip_x;
        context.tip_o=_tip_o;
        context.trajTime=_trajTime;
        context.tol=_tol;
        context.mode=_mode;
        context.useReferences=_useReference;
        context.straightness=ctrl->get_gamma();
        getPosePriority(context.posePriority);
        getTask2ndOptions(context.task_2);
        getSolverConvergenceOptions(context.solverConvergence);

        *id=contextIdCnt++;

        mutex.unlock();
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::restoreContext(const int id)
{
    if (attached)
    {
        mutex.lock();

        map<int,Context>::iterator itr=contextMap.find(id);
        bool valid=(itr!=contextMap.end());
        Context context;
        if (valid)
            context=itr->second;

        mutex.unlock();

        if (valid)
        {
            setDOF(context.dof,context.dof);
            setRestPos(context.restPos,context.restPos);
            setRestWeights(context.restWeights,context.restWeights);
            attachTipFrame(context.tip_x,context.tip_o);

            for (unsigned int axis=0; axis<chainState->getN(); axis++)
                setLimits(axis,context.limits(axis,0),context.limits(axis,1));

            setTrackingMode(context.mode);
            setReferenceMode(context.useReferences);
            setTrajTime(context.trajTime);
            setInTargetTol(context.tol);
            ctrl->set_gamma(context.straightness);
            setPosePriority(context.posePriority);
            setTask2ndOptions(context.task_2);
            setSolverConvergenceOptions(context.solverConvergence);

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::deleteContext(const int id)
{
    map<int,Context>::iterator itr=contextMap.find(id);
    if (itr!=contextMap.end())
    {
        contextMap.erase(itr);
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::deleteContexts(Bottle *contextIdList)
{
    if (contextIdList!=NULL)
    {
        for (int i=0; i<contextIdList->size(); i++)
        {
            int id=contextIdList->get(i).asInt();
            map<int,Context>::iterator itr=contextMap.find(id);
            if (itr!=contextMap.end())
                contextMap.erase(itr);
        }

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getInfo(Bottle &info)
{
    if (attached)
    {
        info.clear();

        Bottle &serverVer=info.addList();
        serverVer.addString("server_version");
        serverVer.addDouble(CARTCTRL_SERVER_VER);

        string type=limbState->getType();
        size_t pos=type.find("_v");
        double hwVer=1.0;
        if (pos!=string::npos)
            hwVer=strtod(type.substr(pos+2).c_str(),NULL);

        Bottle &partVer=info.addList();
        partVer.addString((string(kinPart.c_str())+"_version").c_str());
        partVer.addDouble(hwVer);

        Bottle &events=info.addList();
        events.addString("events");
        Bottle &eventsList=events.addList();
        eventsList.addString("motion-onset");
        eventsList.addString("motion-done");
        eventsList.addString("motion-ongoing");
        eventsList.addString("closing");
        eventsList.addString("*");

        return true;
    }
    else
        return false;
}


/************************************************************************/
void ServerCartesianController::notifyEvent(const string &event,
                                            const double checkPoint)
{
    double time=txInfo.getTime();
    map<string,CartesianEvent*>::iterator itr;

    if (portEvent.getOutputCount()>0)
    {
        Bottle &ev=portEvent.prepare();
        ev.clear();

        ev.addString(event.c_str());
        ev.addDouble(time);
        if (checkPoint>=0.0)
            ev.addDouble(checkPoint);

        portEvent.writeStrict();
    }

    // rise the all-events callback
    itr=eventsMap.find("*");
    if (itr!=eventsMap.end())
    {
        if (itr->second!=NULL)
        {
            CartesianEvent &Event=*itr->second;
            Event.cartesianEventVariables.type=ConstString(event.c_str());
            Event.cartesianEventVariables.time=time;

            if (checkPoint>=0.0)
                Event.cartesianEventVariables.motionOngoingCheckPoint=checkPoint;

            Event.cartesianEventCallback();
        }
    }

    string typeExtended=event;
    if (checkPoint>=0.0)
    {
        ostringstream ss;
        ss<<event<<"-"<<checkPoint;
        typeExtended=ss.str();
    }

    // rise the event specific callback
    itr=eventsMap.find(typeExtended);
    if (itr!=eventsMap.end())
    {
        if (itr->second!=NULL)
        {
            CartesianEvent &Event=*itr->second;
            Event.cartesianEventVariables.type=ConstString(event.c_str());
            Event.cartesianEventVariables.time=time;

            if (checkPoint>=0.0)
                Event.cartesianEventVariables.motionOngoingCheckPoint=checkPoint;

            Event.cartesianEventCallback();
        }
    }
}


/************************************************************************/
bool ServerCartesianController::registerEvent(CartesianEvent &event)
{
    if (!connected)
        return false;

    string type=event.cartesianEventParameters.type.c_str();
    if (type=="motion-ongoing")
    {
        double checkPoint=event.cartesianEventParameters.motionOngoingCheckPoint;
        registerMotionOngoingEvent(checkPoint);

        ostringstream ss;
        ss<<type<<"-"<<checkPoint;
        type=ss.str();
    }

    eventsMap[type]=&event;
    return true;
}


/************************************************************************/
bool ServerCartesianController::unregisterEvent(CartesianEvent &event)
{
    if (!connected)
        return false;

    string type=event.cartesianEventParameters.type.c_str();
    if (type=="motion-ongoing")
    {
        double checkPoint=event.cartesianEventParameters.motionOngoingCheckPoint;
        unregisterMotionOngoingEvent(checkPoint);

        ostringstream ss;
        ss<<type<<"-"<<event.cartesianEventParameters.motionOngoingCheckPoint;
        type=ss.str();
    }

    eventsMap.erase(type);
    return true;
}


/************************************************************************/
void ServerCartesianController::motionOngoingEventsHandling()
{
    if (motionOngoingEventsCurrent.size()!=0)
    {
        double curCheckPoint=*motionOngoingEventsCurrent.begin();
        double dist=norm(qdes-q0);
        double checkPoint=(dist>1e-6)?norm(fb-q0)/dist:1.0;
        checkPoint=std::min(std::max(checkPoint,0.0),1.0);

        if (checkPoint>=curCheckPoint)
        {            
            notifyEvent("motion-ongoing",curCheckPoint);
            motionOngoingEventsCurrent.erase(curCheckPoint);
        }
    }
}


/************************************************************************/
void ServerCartesianController::motionOngoingEventsFlush()
{
    while (motionOngoingEventsCurrent.size()!=0)
    {
        double curCheckPoint=*motionOngoingEventsCurrent.begin();
        notifyEvent("motion-ongoing",curCheckPoint);
        motionOngoingEventsCurrent.erase(curCheckPoint);
    }
}


/************************************************************************/
bool ServerCartesianController::registerMotionOngoingEvent(const double checkPoint)
{
    if ((checkPoint>=0.0) && (checkPoint<=1.0))
    {
        mutex.lock();
        motionOngoingEvents.insert(checkPoint);
        mutex.unlock();

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::unregisterMotionOngoingEvent(const double checkPoint)
{
    bool ret=false;
    if ((checkPoint>=0.0) && (checkPoint<=1.0))
    {
        mutex.lock();
        multiset<double>::iterator itr=motionOngoingEvents.find(checkPoint);
        if (itr!=motionOngoingEvents.end())
        {
            motionOngoingEvents.erase(itr);
            ret=true;
        }
        mutex.unlock();
    }

    return ret;
}


/************************************************************************/
Bottle ServerCartesianController::listMotionOngoingEvents()
{
    Bottle events;

    mutex.lock();
    for (multiset<double>::iterator itr=motionOngoingEvents.begin(); itr!=motionOngoingEvents.end(); itr++)
        events.addDouble(*itr);
    mutex.unlock();

    return events;
}


/************************************************************************/
bool ServerCartesianController::getTask2ndOptions(Value &v)
{
    bool ret=false;
    if (connected)
    {
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_TASK2);

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
        {
            if (ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK))
                v=reply.get(1);
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::setTask2ndOptions(const Value &v)
{
    bool ret=false;
    if (connected)
    {
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_TASK2);
        command.add(v);

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::getSolverConvergenceOptions(Bottle &options)
{
    bool ret=false;
    if (connected)
    {
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_CONVERGENCE);

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
        {
            if (ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK))
                options=*reply.get(1).asList();
        }
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::setSolverConvergenceOptions(const Bottle &options)
{
    bool ret=false;
    if (connected)
    {
        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_CONVERGENCE);
        command.addList()=options;

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);
        else
            printf("%s error: unable to get reply from solver!\n",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::tweakSet(const Bottle &options)
{
    Bottle &opt=const_cast<Bottle&>(options);
    mutex.lock();

    bool ret=true;

    // straightness
    if (opt.check("straightness"))
        ctrl->set_gamma(opt.find("straightness").asDouble());

    // secondary task
    if (opt.check("task_2"))
        ret&=setTask2ndOptions(opt.find("task_2"));

    // solver convergence options
    if (opt.check("max_iter") || opt.check("tol") || opt.check("translationalTol"))
        ret&=setSolverConvergenceOptions(options);

    mutex.unlock();
    return ret;
}


/************************************************************************/
bool ServerCartesianController::tweakGet(Bottle &options)
{
    if (attached)
    {
        mutex.lock();
        options.clear();

        bool ret=true;

        // straightness
        Bottle &straightness=options.addList();
        straightness.addString("straightness");
        straightness.addDouble(ctrl->get_gamma());

        // secondary task
        Value v;
        ret&=getTask2ndOptions(v);
        if (ret)
        {
            Bottle &task2=options.addList(); 
            task2.addString("task_2");
            task2.add(v);
        }

        // solver convergence options
        Bottle b;
        ret&=getSolverConvergenceOptions(b);
        if (ret)
            options.append(b);

        mutex.unlock();
        return ret;
    }
    else
        return false;
}


/************************************************************************/
ServerCartesianController::~ServerCartesianController()
{
    close();
}



