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

#include <sstream>
#include <limits>
#include <algorithm>

#include "CommonCartesianController.h"
#include "ServerCartesianController.h"

#include <yarp/math/Math.h>

#include <iCub/iKin/iKinVocabs.h>

#define CARTCTRL_SERVER_VER                 1.1
#define CARTCTRL_DEFAULT_PER                0.01    // [s]
#define CARTCTRL_DEFAULT_TASKVEL_PERFACTOR  4
#define CARTCTRL_DEFAULT_TOL                1e-2
#define CARTCTRL_DEFAULT_TRAJTIME           2.0     // [s]
#define CARTCTRL_DEFAULT_POSCTRL            "on"
#define CARTCTRL_DEFAULT_MULJNTCTRL         "on"
#define CARTCTRL_CONNECT_SOLVER_PING        1.0     // [s]

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
TaskRefVelTargetGenerator::TaskRefVelTargetGenerator(const double Ts,
                                                     const Vector &x0)
{
    yAssert(x0.length()>=7);
    I=new Integrator(Ts,x0.subVector(0,2));
    R=axis2dcm(x0.subVector(3,6));
}


/************************************************************************/
void TaskRefVelTargetGenerator::reset(const Vector &x0)
{
    yAssert(x0.length()>=7);
    I->reset(x0.subVector(0,2));
    R=axis2dcm(x0.subVector(3,6));
}


/************************************************************************/
Vector TaskRefVelTargetGenerator::integrate(const Vector &vel)
{
    yAssert(vel.length()>=7);
    Vector w=vel.subVector(3,6);
    w[3]*=I->getTs();
    R=axis2dcm(w)*R;
    return cat(I->integrate(vel.subVector(0,2)),dcm2axis(R));
}


/************************************************************************/
TaskRefVelTargetGenerator::~TaskRefVelTargetGenerator()
{
    delete I;
}


/************************************************************************/
ServerCartesianController::ServerCartesianController() :
                           PeriodicThread(CARTCTRL_DEFAULT_PER)
{
    init();
}


/************************************************************************/
ServerCartesianController::ServerCartesianController(Searchable &config) :
                           PeriodicThread(CARTCTRL_DEFAULT_PER)
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
    closed       =true;
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
    pathPerc=0.0;

    txToken=0.0;
    rxToken=0.0;
    txTokenLatchedStopControl=0.0;
    txTokenLatchedGoToRpc=0.0;
    skipSlvRes=false;
    syncEventEnabled=false;

    contextIdCnt=0;
}


/************************************************************************/
void ServerCartesianController::openPorts()
{
    portCmd     =new CartesianCtrlCommandPort(this);
    rpcProcessor=new CartesianCtrlRpcProcessor(this);
    portRpc.setReader(*rpcProcessor);

    string prefixName="/";
    prefixName=prefixName+ctrlName;

    portSlvIn.open(prefixName+"/"+slvName+"/in");
    portSlvOut.open(prefixName+"/"+slvName+"/out");
    portSlvRpc.open(prefixName+"/"+slvName+"/rpc");
    portCmd->open(prefixName+"/command:i");
    portState.open(prefixName+"/state:o");
    portEvent.open(prefixName+"/events:o");
    portRpc.open(prefixName+"/rpc:i");

    if (debugInfoEnabled)
        portDebugInfo.open(prefixName+"/dbg:o");
}


/************************************************************************/
void ServerCartesianController::closePorts()
{
    // close this first since the
    // callback does quite a lot
    if (portCmd!=NULL)
    {
        portCmd->disableCallback();
        portCmd->interrupt();
        portCmd->close();
        delete portCmd;
    }

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
                        lock_guard<mutex> lck(mtx);
                        taskVelModeOn=false;
                        ret=goTo(IKINCTRL_POSE_FULL,xd,t,true);
                    }
                    else if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_XYZ)
                    {
                        lock_guard<mutex> lck(mtx);
                        taskVelModeOn=false;
                        ret=goTo(IKINCTRL_POSE_XYZ,xd,t,true);
                    }

                    if (ret)
                    {
                        // wait for the solver
                        syncEventEnabled=true;
                        unique_lock<mutex> lck(mtx_syncEvent);
                        cv_syncEvent.wait(lck);

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
                    yError("%s: unable to get reply from solver!",ctrlName.c_str());
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
                            string priority;
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
                                reply.addList().read(curRestPos);
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
                                reply.addList().read(curRestWeights);
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
                                reply.addList().read(qdot);
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
                            string priority=command.get(2).asString();
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
                                    reply.addList().read(curRestPos);
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
                                    reply.addList().read(curRestWeights);
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
bool ServerCartesianController::alignJointsBounds()
{
    double min, max; 
    int cnt=0;

    yInfo("%s: aligning joints bounds ...",ctrlName.c_str());
    for (size_t i=0; i<lDsc.size(); i++)
    {
        yInfo("part #%lu: %s",(unsigned long)i,lDsc[i].key.c_str());
        for (int j=0; j<lJnt[i]; j++)
        {
            if (!lLim[i]->getLimits(lRmp[i][j],&min,&max))
            {
                yError("joint #%d: failed getting limits!",cnt);
                return false;
            }

            yInfo("joint #%d: [%g, %g] deg",cnt,min,max);
            (*chainState)[cnt].setMin(CTRL_DEG2RAD*min);
            (*chainState)[cnt].setMax(CTRL_DEG2RAD*max);
            (*chainPlan)[cnt].setMin(CTRL_DEG2RAD*min);
            (*chainPlan)[cnt].setMax(CTRL_DEG2RAD*max);
            cnt++;
        }
    }

    return true;
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
            ok=lPid[i]->getPidReferences(VOCAB_PIDTYPE_POSITION,fbTmp.data());
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
        ctrl=new MultiRefMinJerkCtrl(*chainPlan,ctrlPose,getPeriod());
    else if (plantModelProperties.check("plant_compensator",Value("off")).asString()=="on")
    {
        ctrl=new MultiRefMinJerkCtrl(*chainState,ctrlPose,getPeriod(),true);
        ctrl->setPlantParameters(plantModelProperties,"joint");
    }
    else
        ctrl=new MultiRefMinJerkCtrl(*chainState,ctrlPose,getPeriod());

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
            yWarning("%s: skipped message from solver due to invalid token (rx=%g)>(thr=%g)",
                     ctrlName.c_str(),rxToken,txToken);

            return false;
        }

        // if we stopped the controller then we skip
        // any message with token smaller than the threshold
        if (skipSlvRes)
        {
            if (tokened && !trackingMode && (rxToken<=txTokenLatchedStopControl))
            {
                yWarning("%s: skipped message from solver since controller has been stopped (rx=%g)<=(thr=%g)",
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
            int l1=(int)b2->size();
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
            int l1=(int)b2->size();
            int l2=chainState->getDOF();
            int len=l1<l2 ? l1 : l2;
            _qdes.resize(len);

            for (int i=0; i<len; i++)
                _qdes[i]=CTRL_DEG2RAD*b2->get(i).asDouble();

            if (_qdes.length()!=ctrl->get_dim())
            {    
                yWarning("%s: skipped message from solver since does not match the controller dimension (qdes=%d)!=(ctrl=%d)",
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
            cv_syncEvent.notify_all();
        }

        return isNew;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::areJointsHealthyAndSet(vector<int> &jointsToSet)
{    
    vector<int> modes(maxPartJoints);
    int chainCnt=0;

    jointsToSet.clear();
    for (int i=0; (i<numDrv) && ctrlModeAvailable; i++)
    {
        lMod[i]->getControlModes(modes.data());
        for (int j=0; j<lJnt[i]; j++)
        {
            if (!(*chainState)[chainCnt].isBlocked())
            {
                int mode=modes[lRmp[i][j]];
                if ((mode==VOCAB_CM_HW_FAULT) || (mode==VOCAB_CM_IDLE))
                    return false;
                else if (posDirectEnabled)
                {
                    if (mode!=VOCAB_CM_POSITION_DIRECT)
                        jointsToSet.push_back(chainCnt);
                }
                else if (mode!=VOCAB_CM_VELOCITY)
                    jointsToSet.push_back(chainCnt);
            }

            chainCnt++;
        }
    }

    return true;
}


/************************************************************************/
void ServerCartesianController::setJointsCtrlMode(const vector<int> &jointsToSet)
{
    if (jointsToSet.size()==0)
        return;

    int chainCnt=0;
    int k=0;

    for (int i=0; i<numDrv; i++)
    {
        vector<int> joints;
        vector<int> modes;
        for (int j=0; j<lJnt[i]; j++)
        {
            if (chainCnt==jointsToSet[k])
            {
                joints.push_back(lRmp[i][j]);
                modes.push_back(posDirectEnabled?VOCAB_CM_POSITION_DIRECT:
                                                 VOCAB_CM_VELOCITY);
                k++;
            }

            chainCnt++;
        }

        if (joints.size()>0)
            lMod[i]->setControlModes((int)joints.size(),joints.data(),modes.data());
    }
}


/************************************************************************/
Bottle ServerCartesianController::sendCtrlCmdMultipleJointsPosition()
{
    Bottle info;
    vector<int> joints;
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
            ss<<lDsc[j].key<<"_"<<joint;
            info.addString(ss.str());
            info.addDouble(ref);

            cnt++;
        }

        if (++k>=lJnt[j])
        {
            if (joints.size()>0)
                lPos[j]->setPositions((int)joints.size(),joints.data(),refs.data());

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
    vector<int> joints;
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
                vel=yarp::math::sign(qdes[cnt]-fb[cnt])*thres;

            joints.push_back(joint);
            vels.push_back(vel);

            ostringstream ss;
            ss<<lDsc[j].key<<"_"<<joint;
            info.addString(ss.str());
            info.addDouble(vel);

            velCmd[cnt]=vel;
            cnt++;
        }

        if (++k>=lJnt[j])
        {
            if (joints.size()>0)
                lVel[j]->velocityMove((int)joints.size(),joints.data(),vels.data());

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
            ss<<lDsc[j].key<<"_"<<joint;
            info.addString(ss.str());
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
                vel=yarp::math::sign(qdes[cnt]-fb[cnt])*thres;

            lVel[j]->velocityMove(joint,vel);

            ostringstream ss;
            ss<<lDsc[j].key<<"_"<<joint;
            info.addString(ss.str());
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
        Bottle info;
        info.addString(posDirectEnabled?"position":"velocity");
        info.addString("single");

        int j=0; int k=0;
        for (unsigned int i=0; i<chainState->getN(); i++)
        {
            if (!(*chainState)[i].isBlocked())
            {
                int joint=lRmp[j][k];
                ostringstream ss;
                ss<<lDsc[j].key<<"_"<<joint;
                info.addString(ss.str());

                if (posDirectEnabled)
                {
                    lStp[j]->stop(joint);
                    info.addString("stop");
                }
                else
                {
                    // vel==0.0 is always achievable
                    lVel[j]->velocityMove(joint,0.0);
                    info.addDouble(0.0); 
                }
            }

            if (++k>=lJnt[j])
            {
                j++;
                k=0;
            }
        }

        if (debugInfoEnabled && (portDebugInfo.getOutputCount()>0))
        {
            portDebugInfo.prepare()=info;
            debugInfo.update(txInfo.getTime());
            portDebugInfo.setEnvelope(debugInfo);
            portDebugInfo.writeStrict();
        }
    }

    velCmd=0.0;
    xdot_set=0.0;
}


/************************************************************************/
bool ServerCartesianController::threadInit()
{
    yInfo("Starting %s at %d ms",ctrlName.c_str(),(int)(1000.0*getPeriod()));
    return true;
}


/************************************************************************/
void ServerCartesianController::afterStart(bool s)
{
    if (s)
        yInfo("%s started successfully",ctrlName.c_str());
    else
        yError("%s did not start!",ctrlName.c_str());
}


/************************************************************************/
void ServerCartesianController::run()
{    
    if (connected)
    {
        lock_guard<mutex> lck(mtx);

        // read the feedback
        double stamp=getFeedback(fb);

        // update the stamp anyway
        if (stamp>=0.0)
            txInfo.update(stamp);
        else
            txInfo.update();

        vector<int> jointsToSet;
        jointsHealthy=areJointsHealthyAndSet(jointsToSet);
        if (!jointsHealthy)
            stopControlHelper();

        string event="none";

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

        // compute current point [%] in the path
        double dist=norm(qdes-q0);
        pathPerc=(dist>1e-6)?norm(fb-q0)/dist:1.0;
        pathPerc=std::min(std::max(pathPerc,0.0),1.0);

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
            bool inTarget=ctrl->isInTarget();
            if (inTarget && posDirectEnabled)
                inTarget=isInTargetHelper();
            
            if (inTarget && !taskVelModeOn)
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
                    portDebugInfo.writeStrict();
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
    }
    else if ((++connectCnt)*getPeriod()>CARTCTRL_CONNECT_SOLVER_PING)
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
    yInfo("Stopping %s",ctrlName.c_str());

    if (connected)
        stopLimb();

    notifyEvent("closing");
}


/************************************************************************/
bool ServerCartesianController::open(Searchable &config)
{
    yInfo("***** Configuring cartesian controller *****");
    closed=false;

    // GENERAL group
    Bottle &optGeneral=config.findGroup("GENERAL");
    if (optGeneral.isNull())
    {
        yError("GENERAL group is missing");
        close();
        return false;
    }

    yInfo("Acquiring options for group GENERAL...");

    // scan for required params
    if (optGeneral.check("SolverNameToConnect"))
        slvName=optGeneral.find("SolverNameToConnect").asString();
    else
    {
        yError("SolverNameToConnect option is missing");
        close();
        return false;
    }

    if (optGeneral.check("KinematicPart"))
    {
        kinPart=optGeneral.find("KinematicPart").asString();
        if ((kinPart!="arm") && (kinPart!="leg") && (kinPart!="custom"))
        {
            yError("Attempt to instantiate an unknown kinematic part");
            yError("Available parts are: arm, leg, custom");
            close();
            return false;
        }
    }
    else
    {
        yError("KinematicPart option is missing");
        close();
        return false;
    }

    string _partKinType;
    if (optGeneral.check("KinematicType"))
    {
        kinType=optGeneral.find("KinematicType").asString();
        string _kinType=kinType;
        _partKinType=_kinType.substr(0,_kinType.find("_"));

        if ((_partKinType!="left") && (_partKinType!="right"))
        {
            yError("Attempt to instantiate unknown kinematic type");
            yError("Available types are: left, right, left_v*, right_v*");
            close();
            return false;
        }
    }
    else if (kinPart!="custom")
    {
        yError("KinematicType option is missing");
        close();
        return false;
    }

    if (optGeneral.check("NumberOfDrivers"))
    {
        numDrv=optGeneral.find("NumberOfDrivers").asInt();
        if (numDrv<=0)
        {
            yError("NumberOfDrivers shall be positive");
            close();
            return false;
        }
    }
    else
    {
        yError("NumberOfDrivers option is missing");
        close();
        return false;
    }

    // optional params
    if (optGeneral.check("ControllerName"))
        ctrlName=optGeneral.find("ControllerName").asString();
    else
    {
        ctrlName="cartController/"+kinPart+_partKinType;
        yWarning("default ControllerName assumed: %s",ctrlName.c_str());
    }

    if (optGeneral.check("ControllerPeriod"))
        setPeriod((double)optGeneral.find("ControllerPeriod").asInt()/1000.0);

    taskRefVelPeriodFactor=optGeneral.check("TaskRefVelPeriodFactor",
                                            Value(CARTCTRL_DEFAULT_TASKVEL_PERFACTOR)).asInt();

    posDirectEnabled=optGeneral.check("PositionControl",
                                      Value(CARTCTRL_DEFAULT_POSCTRL)).asString()=="on";

    multipleJointsControlEnabled=optGeneral.check("MultipleJointsControl",
                                                  Value(CARTCTRL_DEFAULT_MULJNTCTRL)).asString()=="on";

    debugInfoEnabled=optGeneral.check("DebugInfo",Value("off")).asString()=="on";
    if (debugInfoEnabled)
        yDebug("Commands to robot will be also streamed out on debug port");

    // scan DRIVER groups
    for (int i=0; i<numDrv; i++)
    {
        ostringstream entry;
        entry<<"DRIVER_"<<i;
        string entry_str(entry.str());

        Bottle &optDrv=config.findGroup(entry_str);
        if (optDrv.isNull())
        {
            yError("%s group is missing",entry_str.c_str());
            close();
            return false;
        }

        yInfo("Acquiring options for group %s...",entry_str.c_str());

        DriverDescriptor desc;

        if (optDrv.check("Key"))
            desc.key=optDrv.find("Key").asString();
        else
        {
            yError("Key option is missing");
            close();
            return false;
        }

        if (optDrv.check("JointsOrder"))
        {
            string jointsOrderType=optDrv.find("JointsOrder").asString();

            if (jointsOrderType=="direct")
                desc.jointsDirectOrder=true;
            else if (jointsOrderType=="reversed")
                desc.jointsDirectOrder=false;
            else
            {
                yError("Attempt to select an unknown mapping order");
                yError("Available orders are: direct, reversed");
                close();
                return false;
            }
        }
        else
        {
            yError("JointsOrder option is missing");
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
            yWarning("MinAbsVels option is missing ... using default values");
            desc.useDefaultMinAbsVel=true;
        }

        lDsc.push_back(desc);
    }

    // acquire options for Plant modeling
    Bottle &optPlantModel=config.findGroup("PLANT_MODEL");
    if (!optPlantModel.isNull())
    {
        yInfo("PLANT_MODEL group detected");
        plantModelProperties.fromString(optPlantModel.toString());

        // append information about the predictor's period,
        // that must match the controller's period
        plantModelProperties.unput("Ts");
        plantModelProperties.put("Ts",getPeriod());
    }
    else
        plantModelProperties.clear();

    // instantiate kinematic object
    if (kinPart=="arm")
        limbState=new iCubArm(kinType);
    else if (kinPart=="leg")
        limbState=new iCubLeg(kinType);
    else if (optGeneral.check("CustomKinFile"))     // custom kinematics case
    {
        ResourceFinder rf_kin;
        if (optGeneral.check("CustomKinContext"))
            rf_kin.setDefaultContext(optGeneral.find("CustomKinContext").asString()); 
        rf_kin.configure(0,NULL);
        string pathToCustomKinFile=rf_kin.findFileByName(optGeneral.find("CustomKinFile").asString());

        Property linksOptions;
        linksOptions.fromConfigFile(pathToCustomKinFile);

        limbState=new iKinLimb(linksOptions);
        if (!limbState->isValid())
        {
            yError("Invalid links parameters");
            close();
            return false;
        }
    }
    else
    {
        yError("CustomKinFile option is missing");
        close();
        return false;
    }

    Property DHTable; limbState->toLinksProperties(DHTable);
    yInfo()<<"DH Table: "<<DHTable.toString();  // stream version to prevent long strings truncation

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

    delete limbState;
    delete limbPlan;

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

    yInfo("***** Attaching drivers to cartesian controller %s *****",ctrlName.c_str());
    yInfo("Received list of %d driver(s)",nd);

    if (nd!=numDrv)
    {
        yError("Expected list of %d driver(s)",numDrv);
        return false;
    }

    int remainingJoints=chainState->getN();

    ctrlModeAvailable=true;
    encTimedEnabled=true;
    pidAvailable=true;
    posDirectAvailable=true;

    for (int i=0; i<numDrv; i++)
    {
        yInfo("Acquiring info on driver %s...",lDsc[i].key.c_str());

        // check if what we require is present within the given list
        int j;
        for (j=0; j<drivers.size(); j++)
            if (lDsc[i].key==drivers[j]->key)
                break;

        if (j>=drivers.size())
        {
            yError("None of provided drivers is of type %s",lDsc[i].key.c_str());
            return false;
        }

        // acquire interfaces and driver's info
        if (drivers[j]->poly->isValid())
        {
            yInfo("driver %s successfully open",lDsc[i].key.c_str());

            IControlMode     *mod;
            IEncoders        *enc;
            IEncodersTimed   *ent;
            IPidControl      *pid;
            IControlLimits   *lim;
            IVelocityControl *vel;
            IPositionDirect  *pos;
            IPositionControl *stp;
            int               joints;
            
            drivers[j]->poly->view(enc);
            drivers[j]->poly->view(lim);

            ctrlModeAvailable&=drivers[j]->poly->view(mod);
            drivers[j]->poly->view(vel);

            encTimedEnabled&=drivers[j]->poly->view(ent);
            pidAvailable&=drivers[j]->poly->view(pid);
            posDirectAvailable&=drivers[j]->poly->view(pos);
            posDirectAvailable&=drivers[j]->poly->view(stp);

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
            lPos.push_back(pos);
            lStp.push_back(stp);
            lJnt.push_back(joints);
            lRmp.push_back(rmpTmp);
            lVel.push_back(vel);
        }
        else
        {
            yError("unable to open driver %s",lDsc[i].key.c_str());
            return false;
        }
    }

    yInfo("%s: IControlMode %s",ctrlName.c_str(),
          ctrlModeAvailable?"available":"not available");

    yInfo("%s: %s interface will be used",ctrlName.c_str(),
          encTimedEnabled?"IEncodersTimed":"IEncoders");

    yInfo("%s: IPidControl %s",ctrlName.c_str(),
          pidAvailable?"available":"not available");

    yInfo("%s: IPositionDirect %s",ctrlName.c_str(),
          posDirectAvailable?"available":"not available");

    posDirectEnabled&=posDirectAvailable;
    yInfo("%s: %s interface will be used",ctrlName.c_str(),
          posDirectEnabled?"IPositionDirect":"IVelocityControl");

    yInfo("%s: %s control will be used",ctrlName.c_str(),
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

    if (!alignJointsBounds())
    {
        yError("unable to retrieve joints limits");
        return false;
    }

    // exclude acceleration constraints by fixing
    // thresholds at high values
    for (int i=0; i<numDrv; i++)
        for (int j=0; j<lJnt[i]; j++)
            lVel[i]->setRefAcceleration(j,std::numeric_limits<double>::max());

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
    taskRefVelTargetGen=new TaskRefVelTargetGenerator(taskRefVelPeriodFactor*getPeriod(),ctrl->get_x());
    taskRefVelPeriodCnt=0;

    start();

    return true;
}


/************************************************************************/
bool ServerCartesianController::detachAll()
{
    if (isRunning())
        stop();

    delete taskRefVelTargetGen;
    taskRefVelTargetGen=NULL;

    for (unsigned int i=0; i<lRmp.size(); i++)
    {
        delete[] lRmp[i];
        lRmp[i]=NULL;
    }

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

    delete ctrl;
    ctrl=NULL;

    attached=false;
    return true;
}


/************************************************************************/
bool ServerCartesianController::pingSolver()
{    
    string portSlvName="/";
    portSlvName=portSlvName+slvName+"/in";    

    bool ok=Network::exists(portSlvName,true);
    yInfo("%s: Checking if cartesian solver %s is alive... %s",
          ctrlName.c_str(),slvName.c_str(),ok?"ready":"not yet");

    return ok;
}


/************************************************************************/
bool ServerCartesianController::connectToSolver()
{
    if (attached && !connected && pingSolver())
    {        
        yInfo("%s: Connecting to cartesian solver %s...",ctrlName.c_str(),slvName.c_str());

        string portSlvName="/";
        portSlvName=portSlvName+slvName;

        bool ok=true;

        ok&=Network::connect(portSlvName+"/out",portSlvIn.getName(),"udp");
        ok&=Network::connect(portSlvOut.getName(),portSlvName+"/in","udp");
        ok&=Network::connect(portSlvRpc.getName(),portSlvName+"/rpc");

        if (ok)
            yInfo("%s: Connections established with %s",ctrlName.c_str(),slvName.c_str());
        else
        {
            yError("%s: Problems detected while connecting to %s",ctrlName.c_str(),slvName.c_str());
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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());         
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
        command.addVocab(f?IKINSLV_VOCAB_VAL_MODE_TRACK:IKINSLV_VOCAB_VAL_MODE_SINGLE);

        // send command to solver and wait for reply
        bool ret=false;
        if (!portSlvRpc.write(command,reply))
            yError("%s: unable to get reply from solver!",ctrlName.c_str());         
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
    lock_guard<mutex> lck(mtx);
    return setTrackingModeHelper(f);
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
bool ServerCartesianController::setPosePriority(const string &p)
{
    bool ret=false;
    if (connected && ((p=="position") || (p=="orientation")))
    {
        lock_guard<mutex> lck(mtx);

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_PRIO);
        command.addVocab(p=="position"?IKINSLV_VOCAB_VAL_PRIO_XYZ:IKINSLV_VOCAB_VAL_PRIO_ANG);

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);
        else
            yError("%s: unable to get reply from solver!",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::getPosePriority(string &p)
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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::getPose(Vector &x, Vector &o, Stamp *stamp)
{
    if (attached)
    {
        lock_guard<mutex> lck(mtx);
        Vector pose=chainState->EndEffPose();
    
        x.resize(3);
        o.resize(pose.length()-x.length());
    
        for (size_t i=0; i<x.length(); i++)
            x[i]=pose[i];
    
        for (size_t i=0; i<o.length(); i++)
            o[i]=pose[x.length()+i];

        if (stamp!=NULL)
            *stamp=txInfo;

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
        lock_guard<mutex> lck(mtx);

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
        lock_guard<mutex> lck(mtx);

        Vector _xd(xd.length()+od.length());
        for (size_t i=0; i<xd.length(); i++)
            _xd[i]=xd[i];
    
        for (size_t i=0; i<od.length(); i++)
            _xd[xd.length()+i]=od[i];

        taskVelModeOn=false;
        return goTo(IKINCTRL_POSE_FULL,_xd,t);
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPosition(const Vector &xd, const double t)
{
    if (connected)
    {
        lock_guard<mutex> lck(mtx);

        taskVelModeOn=false;
        return goTo(IKINCTRL_POSE_XYZ,xd,t);
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
        lock_guard<mutex> lck(mtx);

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

    lock_guard<mutex> lck(mtx);

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
        yError("%s: unable to get reply from solver!",ctrlName.c_str());         

    return ret;
}


/************************************************************************/
bool ServerCartesianController::askForPose(const Vector &q0, const Vector &xd,
                                           const Vector &od, Vector &xdhat,
                                           Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    lock_guard<mutex> lck(mtx);

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
        yError("%s: unable to get reply from solver!",ctrlName.c_str());         

    return ret;
}


/************************************************************************/
bool ServerCartesianController::askForPosition(const Vector &xd, Vector &xdhat,
                                               Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    lock_guard<mutex> lck(mtx);

    Bottle command, reply;
    command.addVocab(IKINSLV_VOCAB_CMD_ASK);
    addVectorOption(command,IKINSLV_VOCAB_OPT_XD,xd);
    addPoseOption(command,IKINCTRL_POSE_XYZ);

    // send command and wait for reply
    bool ret=false;
    if (portSlvRpc.write(command,reply))
        ret=getDesiredOption(reply,xdhat,odhat,qdhat);
    else
        yError("%s: unable to get reply from solver!",ctrlName.c_str());         

    return ret;
}


/************************************************************************/
bool ServerCartesianController::askForPosition(const Vector &q0, const Vector &xd,
                                               Vector &xdhat, Vector &odhat,
                                               Vector &qdhat)
{
    if (!connected)
        return false;

    lock_guard<mutex> lck(mtx);

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
        yError("%s: unable to get reply from solver!",ctrlName.c_str());         

    return ret;
}


/************************************************************************/
bool ServerCartesianController::getDOF(Vector &curDof)
{
    if (connected)
    {
        lock_guard<mutex> lck(mtx);

        curDof.resize(chainState->getN());
        for (unsigned int i=0; i<chainState->getN(); i++)
            curDof[i]=!(*chainState)[i].isBlocked();
    
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
        lock_guard<mutex> lck(mtx);

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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());         
        
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
        lock_guard<mutex> lck(mtx);

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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());
                
        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setRestPos(const Vector &newRestPos,
                                           Vector &curRestPos)
{
    if (connected)
    {
        lock_guard<mutex> lck(mtx);

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_POS);
        command.addList().read(newRestPos);
    
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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());

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
        lock_guard<mutex> lck(mtx);

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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());

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
        lock_guard<mutex> lck(mtx);

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_WEIGHTS);
        command.addList().read(newRestWeights);
    
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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());

        return ret;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getLimits(const int axis, double *min,
                                          double *max)
{
    bool ret=false;
    if (connected && (min!=NULL) && (max!=NULL))
    {
        lock_guard<mutex> lck(mtx);
        if (axis<(int)chainState->getN())
        {
            Bottle command, reply;
            command.addVocab(IKINSLV_VOCAB_CMD_GET);
            command.addVocab(IKINSLV_VOCAB_OPT_LIM);
            command.addInt(axis);

            // send command to solver and wait for reply
            if (!portSlvRpc.write(command,reply))
                yError("%s: unable to get reply from solver!",ctrlName.c_str());         
            else if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
            {
                *min=reply.get(1).asDouble();
                *max=reply.get(2).asDouble();                        
                ret=true;
            }
        }        
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
        lock_guard<mutex> lck(mtx);

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_LIM);
        command.addInt(axis);
        command.addDouble(min);
        command.addDouble(max);

        // send command to solver and wait for reply        
        if (portSlvRpc.write(command,reply))
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);
        else
            yError("%s: unable to get reply from solver!",ctrlName.c_str());
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
    lock_guard<mutex> lck(mtx);
    return setTrajTimeHelper(t);
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
bool ServerCartesianController::isInTargetHelper()
{
    Matrix H=chainState->getH();
    Vector e(6,0.0);

    if (ctrlPose!=IKINCTRL_POSE_ANG)
    {
        e[0]=xdes[0]-H(0,3);
        e[1]=xdes[1]-H(1,3);
        e[2]=xdes[2]-H(2,3);
    }

    if (ctrlPose!=IKINCTRL_POSE_XYZ)
    {
        Matrix Des=axis2dcm(xdes.subVector(3,6));
        Vector ax=dcm2axis(Des*SE3inv(H));
        e[3]=ax[3]*ax[0];
        e[4]=ax[3]*ax[1];
        e[5]=ax[3]*ax[2];
    }

    return (norm(e)<targetTol);
}


/************************************************************************/
bool ServerCartesianController::setInTargetTol(const double tol)
{
    lock_guard<mutex> lck(mtx);
    return setInTargetTolHelper(tol);
}


/************************************************************************/
bool ServerCartesianController::getJointsVelocities(Vector &qdot)
{
    if (connected)
    {
        lock_guard<mutex> lck(mtx);
        qdot=velCmd;
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
        lock_guard<mutex> lck(mtx);

        Matrix J=ctrl->get_J();
        Vector taskVel(7,0.0);

        if ((J.rows()>0) && (J.cols()==velCmd.length()))
        {
            taskVel=J*(CTRL_DEG2RAD*velCmd);

            Vector _odot=taskVel.subVector(3,(unsigned int)taskVel.length()-1);
            double thetadot=norm(_odot);
            if (thetadot>0.0)
                _odot/=thetadot;

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
        lock_guard<mutex> lck(mtx);

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
        lock_guard<mutex> lck(mtx);

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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());         
        
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
        lock_guard<mutex> lck(mtx);
        Matrix HN=chainState->getHN();

        x=HN.getCol(3);
        x.pop_back();

        o=dcm2axis(HN);        
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
    lock_guard<mutex> lck(mtx);
    return stopControlHelper();
}


/************************************************************************/
bool ServerCartesianController::storeContext(int *id)
{
    if (connected && (id!=NULL))
    {
        mtx.lock();
        unsigned int N=chainState->getN();
        mtx.unlock();

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

        lock_guard<mutex> lck(mtx);

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
        mtx.lock();

        map<int,Context>::iterator itr=contextMap.find(id);
        bool valid=(itr!=contextMap.end());
        Context context;
        if (valid)
            context=itr->second;

        mtx.unlock();

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
        lock_guard<mutex> lck(mtx);

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

        string kinPartStr(kinPart);
        string type=limbState->getType();

        size_t pos=type.find("_v");
        double hwVer=1.0;
        if (pos!=string::npos)
            hwVer=strtod(type.substr(pos+2).c_str(),NULL);        

        Bottle &partVer=info.addList();
        partVer.addString(kinPartStr+"_version");
        partVer.addDouble(hwVer);

        Bottle &partType=info.addList();
        partType.addString(kinPartStr+"_type");
        partType.addString(type);

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

        ev.addString(event);
        ev.addDouble(time);
        if (checkPoint>=0.0)
            ev.addDouble(checkPoint);

        eventInfo.update(time);
        portEvent.setEnvelope(eventInfo);
        portEvent.writeStrict();
    }

    // rise the all-events callback
    itr=eventsMap.find("*");
    if (itr!=eventsMap.end())
    {
        if (itr->second!=NULL)
        {
            CartesianEvent &Event=*itr->second;
            Event.cartesianEventVariables.type=event;
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
            Event.cartesianEventVariables.type=event;
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

    string type=event.cartesianEventParameters.type;
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

    string type=event.cartesianEventParameters.type;
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
        if (pathPerc>=curCheckPoint)
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
        lock_guard<mutex> lck(mtx);
        motionOngoingEvents.insert(checkPoint);
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
        lock_guard<mutex> lck(mtx);
        multiset<double>::iterator itr=motionOngoingEvents.find(checkPoint);
        if (itr!=motionOngoingEvents.end())
        {
            motionOngoingEvents.erase(itr);
            ret=true;
        }
    }

    return ret;
}


/************************************************************************/
Bottle ServerCartesianController::listMotionOngoingEvents()
{
    Bottle events;

    lock_guard<mutex> lck(mtx);
    for (multiset<double>::iterator itr=motionOngoingEvents.begin(); itr!=motionOngoingEvents.end(); itr++)
        events.addDouble(*itr);

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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());         
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::setTask2ndOptions(const Value &v)
{
    bool ret=false;
    if (connected)
    {
        lock_guard<mutex> lck(mtx);

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_TASK2);
        command.add(v);

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);
        else
            yError("%s: unable to get reply from solver!",ctrlName.c_str());
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
            yError("%s: unable to get reply from solver!",ctrlName.c_str());         
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::setSolverConvergenceOptions(const Bottle &options)
{
    bool ret=false;
    if (connected)
    {
        lock_guard<mutex> lck(mtx);

        Bottle command, reply;
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_CONVERGENCE);
        command.addList()=options;

        // send command to solver and wait for reply
        if (portSlvRpc.write(command,reply))
            ret=(reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK);
        else
            yError("%s: unable to get reply from solver!",ctrlName.c_str());        
    }

    return ret;
}


/************************************************************************/
bool ServerCartesianController::tweakSet(const Bottle &options)
{
    bool ret=true;

    // straightness
    if (options.check("straightness"))
        ctrl->set_gamma(options.find("straightness").asDouble());

    // secondary task
    if (options.check("task_2"))
        ret&=setTask2ndOptions(options.find("task_2"));

    // solver convergence options
    if (options.check("tol") || options.check("constr_tol") ||
        options.check("max_iter"))
        ret&=setSolverConvergenceOptions(options);

    return ret;
}


/************************************************************************/
bool ServerCartesianController::tweakGet(Bottle &options)
{
    if (attached)
    {
        lock_guard<mutex> lck(mtx);
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



