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

#include <yarp/os/Time.h>
#include <yarp/os/Network.h>

#include <iCub/iKin/iKinVocabs.h>

#include <stdio.h>

#include "CommonCartesianController.h"
#include "ServerCartesianController.h"

#define CARTCTRL_DEFAULT_PER                10      // [ms]
#define CARTCTRL_DEFAULT_TASKVEL_PERFACTOR  4
#define CARTCTRL_DEFAULT_TASKVEL_BALLPOS    0.2     // [m]
#define CARTCTRL_DEFAULT_TASKVEL_BALLORIEN  45.0    // [deg]
#define CARTCTRL_DEFAULT_TOL                1e-2
#define CARTCTRL_DEFAULT_TRAJTIME           2.0     // [s]
#define CARTCTRL_MAX_ACCEL                  1e9     // [deg/s^2]
#define CARTCTRL_CONNECT_TMO                5e3     // [ms]

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
CartesianCtrlRpcProcessor::CartesianCtrlRpcProcessor(ServerCartesianController *_ctrl)
{
    ctrl=_ctrl;
}


/************************************************************************/
bool CartesianCtrlRpcProcessor::read(ConnectionReader &connection)
{
    Bottle cmd, reply;

    if (!cmd.read(connection))
        return false;

    if (ctrl->respond(cmd,reply))
        if (ConnectionWriter *writer=connection.getWriter())
            reply.write(*writer);

    return true;
}


/************************************************************************/
CartesianCtrlCommandPort::CartesianCtrlCommandPort(ServerCartesianController *_ctrl)
{
    ctrl=_ctrl;
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

                for (int i=0; i<xd.length(); i++)
                    xd[i]=v->get(i).asDouble();

                for (int i=0; i<od.length(); i++)
                    od[i]=v->get(xd.length()+i).asDouble();

                ctrl->goToPose(xd,od,t);
            }
            else if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_XYZ)
            {
                Vector xd(v->size());

                for (int i=0; i<v->size(); i++)
                    xd[i]=v->get(i).asDouble();

                ctrl->goToPosition(xd,t);
            }
        }
        else if ((command.get(0).asVocab()==IKINCARTCTRL_VOCAB_CMD_TASKVEL) && (command.size()>1))
        {
            Bottle *v=command.get(1).asList();

            Vector xdot(3);
            Vector odot(v->size()-xdot.length());

            for (int i=0; i<xdot.length(); i++)
                xdot[i]=v->get(i).asDouble();

            for (int i=0; i<odot.length(); i++)
                odot[i]=v->get(xdot.length()+i).asDouble();

            ctrl->setTaskVelocities(xdot,odot);
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
    limb =NULL;
    chain=NULL;
    ctrl =NULL;

    portCmd     =NULL;
    rpcProcessor=NULL;

    attached     =false;
    connected    =false;
    closed       =false;
    trackingMode =false;
    executingTraj=false;
    taskVelModeOn=false;
    motionDone   =true;    

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

    portCmd->useCallback();
    portRpc.setReader(*rpcProcessor);

    ConstString prefixName="/";
    prefixName=prefixName+ctrlName;

    portSlvIn.open((prefixName+"/"+slvName+"/in").c_str());
    portSlvOut.open((prefixName+"/"+slvName+"/out").c_str());
    portSlvRpc.open((prefixName+"/"+slvName+"/rpc").c_str());
    portCmd->open((prefixName+"/command:i").c_str());
    portState.open((prefixName+"/state:o").c_str());
    portRpc.open((prefixName+"/rpc:i").c_str());
}


/************************************************************************/
void ServerCartesianController::closePorts()
{
    portSlvIn.interrupt();
    portSlvOut.interrupt();
    portSlvRpc.interrupt();
    portState.interrupt();
    portRpc.interrupt();

    portSlvIn.close();
    portSlvOut.close();
    portSlvRpc.close();
    portState.close();
    portRpc.close();

    if (portCmd!=NULL)
    {
        portCmd->interrupt();
        portCmd->close();
        delete portCmd;
    }

    if (rpcProcessor!=NULL)
        delete rpcProcessor;
}


/************************************************************************/
bool ServerCartesianController::respond(const Bottle &command, Bottle &reply)
{
    if (command.size())
        switch (command.get(0).asVocab())
        {
            case IKINCARTCTRL_VOCAB_CMD_STOP:
            {   
                // begin of critical code
                mutex.wait();

                stopControl();

                // end of critical code
                mutex.post();

                break;
            }

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
                        taskVelModeOn=false;
                        ret=goTo(IKINCTRL_POSE_FULL,xd,t,true);
                    }
                    else if (pose==IKINCARTCTRL_VOCAB_VAL_POSE_XYZ)
                    {
                        taskVelModeOn=false;
                        ret=goTo(IKINCTRL_POSE_XYZ,xd,t,true);
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

            case IKINCARTCTRL_VOCAB_CMD_ASK:
            {
                // just behave as a relay
                Bottle slvCommand=command;

                if (!portSlvRpc.write(slvCommand,reply))
                {
                    fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                }

                break;
            }

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

            case IKINSLV_VOCAB_CMD_GET:
            {
                if (command.size()>1)
                    switch (command.get(1).asVocab())
                    {
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

                        case IKINCARTCTRL_VOCAB_OPT_ISSOLVERON:
                        {
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                            if (connected)
                                reply.addVocab(IKINCARTCTRL_VOCAB_VAL_TRUE);
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_VAL_FALSE);
    
                            break;
                        }

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

                        case IKINCARTCTRL_VOCAB_OPT_DOF:
                        {
                            Vector curDof;

                            if (getDOF(curDof))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &dofPart=reply.addList();
                                    
                                for (int i=0; i<curDof.length(); i++)
                                    dofPart.addInt((int)curDof[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_REST_POS:
                        {
                            Vector curRestPos;
    
                            if (getRestPos(curRestPos))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &restPart=reply.addList();
    
                                for (int i=0; i<curRestPos.length(); i++)
                                    restPart.addDouble(curRestPos[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }
    
                        case IKINCARTCTRL_VOCAB_OPT_REST_WEIGHTS:
                        {
                            Vector curRestWeights;
    
                            if (getRestWeights(curRestWeights))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &restPart=reply.addList();
    
                                for (int i=0; i<curRestWeights.length(); i++)
                                    restPart.addDouble(curRestWeights[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_DES:
                        {
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);

                            Vector q(chain->getN());
                            int cnt=0;

                            for (unsigned int i=0; i<chain->getN(); i++)
                                if ((*chain)[i].isBlocked())
                                    q[i]=CTRL_RAD2DEG*chain->getAng(i);
                                else
                                    q[i]=CTRL_RAD2DEG*qdes[cnt++];

                            addVectorOption(reply,IKINCARTCTRL_VOCAB_OPT_X,xdes);
                            addVectorOption(reply,IKINCARTCTRL_VOCAB_OPT_Q,q);

                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_QDOT:
                        {
                            Vector qdot;
    
                            if (getJointsVelocities(qdot))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &qdotPart=reply.addList();
    
                                for (int i=0; i<qdot.length(); i++)
                                    qdotPart.addDouble(qdot[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }
    
                        case IKINCARTCTRL_VOCAB_OPT_XDOT:
                        {
                            Vector xdot, odot;

                            if (getTaskVelocities(xdot,odot))
                            {
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                Bottle &xdotPart=reply.addList();
    
                                for (int i=0; i<xdot.length(); i++)
                                    xdotPart.addDouble(xdot[i]);
    
                                for (int i=0; i<odot.length(); i++)
                                    xdotPart.addDouble(odot[i]);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_POSE:
                        {               
                            if (command.size()>2)
                            {
                                int axis=command.get(2).asInt();
                                Vector x,o;

                                if (getPose(axis,x,o))
                                {
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_ACK);
                                    Bottle &posePart=reply.addList();

                                    for (int i=0; i<x.length(); i++)
                                        posePart.addDouble(x[i]);

                                    for (int i=0; i<o.length(); i++)
                                        posePart.addDouble(o[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                            break;
                        }

                        default:
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            case IKINCARTCTRL_VOCAB_CMD_SET:
            {
                if (command.size()>2)
                    switch (command.get(1).asVocab())
                    {
                        case IKINCARTCTRL_VOCAB_OPT_MODE:
                        {
                            int mode=command.get(2).asVocab();
    
                            if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_TRACK)
                            {    
                                if (setTrackingMode(true))
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                else
                                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            }
                            else if (mode==IKINCARTCTRL_VOCAB_VAL_MODE_SINGLE)
                            {    
                                if (setTrackingMode(false))
                                    reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                                else
                                    reply.addVocab(IKINSLV_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINSLV_VOCAB_REP_NACK);
    
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_TIME:
                        {
                            setTrajTime(command.get(2).asDouble());
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            break;
                        }

                        case IKINCARTCTRL_VOCAB_OPT_TOL:
                        {
                            setInTargetTol(command.get(2).asDouble());
                            reply.addVocab(IKINSLV_VOCAB_REP_ACK);
                            break;
                        }

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

                                    for (int i=0; i<curDof.length(); i++)
                                        dofPart.addInt((int)curDof[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            

                            break;
                        }

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
    
                                    for (int i=0; i<curRestPos.length(); i++)
                                        restPart.addDouble(curRestPos[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            
    
                            break;
                        }

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
    
                                    for (int i=0; i<curRestWeights.length(); i++)
                                        restPart.addDouble(curRestWeights[i]);
                                }
                                else
                                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                            }
                            else
                                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);                            
    
                            break;
                        }

                        default:
                            reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
                    }
                else
                    reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

                break;
            }

            default:
                reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);
        }
    else
        reply.addVocab(IKINCARTCTRL_VOCAB_REP_NACK);

    return true;
}


/************************************************************************/
void ServerCartesianController::stopLimbVel()
{
    int j=0;
    int k=0;

    // this timeout prevents the stop() from
    // being overwritten by the last velocityMove()
    // which travels on a different connection.
    Time::delay((2.0/1000.0)*getRate());

    for (unsigned int i=0; i<chain->getN(); i++)
    {
        if (!(*chain)[i].isBlocked())
            lVel[j]->stop(lRmp[j][k]);

        if (++k>=lJnt[j])
        {
            j++;
            k=0;
        }
    }
}


/************************************************************************/
void ServerCartesianController::alignJointsBounds()
{
    if (connected)
    {           
        fprintf(stdout,"Getting joints bounds from cartesian solver %s ...\n",slvName.c_str());
    
        for (unsigned int i=0; i<chain->getN(); i++)
        {
            Bottle command, reply;
            double min, max;

            fprintf(stdout,"joint #%d: ... ",i);

            // prepare command
            command.addVocab(IKINSLV_VOCAB_CMD_GET);
            command.addVocab(IKINSLV_VOCAB_OPT_LIM);
            command.addInt(i);

            // send command to solver and wait for reply
            if (!portSlvRpc.write(command,reply))
            {
                fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
                return;
            }

            if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
            {
                min=reply.get(1).asDouble();
                max=reply.get(2).asDouble();                        
        
                // align local joint's bounds
                (*chain)[i].setMin(CTRL_DEG2RAD*min);
                (*chain)[i].setMax(CTRL_DEG2RAD*max);

                fprintf(stdout,"[%.1f, %.1f] deg\n",min,max);
            }
            else
                fprintf(stdout,"failed\n");
        }
    }
}


/************************************************************************/
void ServerCartesianController::getFeedback(Vector &_fb)
{
    Vector fbTmp(maxPartJoints);
    int chainCnt=0;
    int _fbCnt=0;

    for (int i=0; i<numDrv; i++)
    {
        if (lEnc[i]->getEncoders(fbTmp.data()))
        {
            for (int j=0; j<lJnt[i]; j++)
            {
                double tmp=CTRL_DEG2RAD*fbTmp[lRmp[i][j]];
            
                if ((*chain)[chainCnt].isBlocked())
                    chain->setBlockingValue(chainCnt,tmp);
                else
                    _fb[_fbCnt++]=tmp;
            
                chainCnt++;
            }
        }
        else for (int j=0; j<lJnt[i]; j++)
        {
            if (!(*chain)[chainCnt++].isBlocked())
                _fbCnt++;
        }
    }
}


/************************************************************************/
void ServerCartesianController::newController()
{
    // guard
    if (!chain)
        return;

    stopControl();

    // if it already exists, destroy old controller
    if (ctrl!=NULL)
        delete ctrl;

    // update quantities
    fb.resize(chain->getDOF());
    getFeedback(fb);
    chain->setAng(fb);
    velCmd.resize(chain->getDOF(),0.0);
    xdes=chain->EndEffPose();
    qdes=chain->getAng();

    // instantiate new controller
    ctrl=new MultiRefMinJerkCtrl(*chain,ctrlPose,getRate()/1000.0);

    // set tolerance
    ctrl->setInTargetTol(targetTol);

    // set task execution time
    trajTime=ctrl->set_execTime(trajTime,true);
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
            fprintf(stdout,"%s warning: skipped message from solver due to invalid token (rx=%g)>(thr=%g)\n",
                    ctrlName.c_str(),rxToken,txToken);

            return false;
        }

        // if we stopped the controller then we skip
        // any message with token smaller than the threshold
        if (skipSlvRes)
        {
            if (tokened && !trackingMode && (rxToken<=txTokenLatchedStopControl))
            {
                fprintf(stdout,"%s warning: skipped message from solver since controller has been stopped (rx=%g)<=(thr=%g)\n",
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
            int l2=chain->getDOF();
            int len=l1<l2 ? l1 : l2;
            _qdes.resize(len);

            for (int i=0; i<len; i++)
                _qdes[i]=CTRL_DEG2RAD*b2->get(i).asDouble();

            if (_qdes.length()!=ctrl->get_dim())
            {    
                fprintf(stdout,"%s warning: skipped message from solver since does not match the controller dimension (qdes=%d)!=(ctrl=%d)\n",
                        ctrlName.c_str(),_qdes.length(),ctrl->get_dim());

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
void ServerCartesianController::sendVelocity(const Vector &v)
{
    int j=0;
    int k=0;
    int cnt=0;

    for (unsigned int i=0; i<chain->getN(); i++)
    {
        if (!(*chain)[i].isBlocked())
        {    
            double v_cnt=v[cnt];

            // send only if changed
            if (v_cnt!=velCmd[cnt])
            {
                double thres=lDsc[j].minAbsVels[k];

                // apply bang-bang control to compensate for unachievable low velocities
                if ((v_cnt>-thres) && (v_cnt<thres) && (v_cnt!=0.0))
                {
                    // current error in the joint space
                    double e=qdes[cnt]-fb[cnt];

                    if (e>0.0)
                        v_cnt=thres;
                    else if (e<0.0)
                        v_cnt=-thres;
                    else
                        v_cnt=0.0;
                }

                lVel[j]->velocityMove(lRmp[j][k],velCmd[cnt]=v_cnt);
            }

            cnt++;
        }

        if (++k>=lJnt[j])
        {
            j++;
            k=0;
        }
    }
}


/************************************************************************/
bool ServerCartesianController::threadInit()
{
    fprintf(stdout,"Starting %s at %d ms\n",ctrlName.c_str(),(int)getRate());

    return true;
}


/************************************************************************/
void ServerCartesianController::afterStart(bool s)
{
    fprintf(stdout,"%s ",ctrlName.c_str());

    if (s)
        fprintf(stdout," started successfully\n");
    else
        fprintf(stdout," did not start!\n");
}


/************************************************************************/
void ServerCartesianController::run()
{    
    if (connected)
    {
        // begin of critical code
        mutex.wait();

        // read the feedback
        getFeedback(fb);
        ctrl->set_q(fb);

        // manage the virtual target yielded by a
        // request for a task-space reference velocity
        if (taskVelModeOn && (++taskRefVelPeriodCnt>=taskRefVelPeriodFactor))
        {
            Vector xdot_set_int=taskRefVelTargetGen->integrate(xdot_set);

            Vector xdes_x(3), xdot_set_int_x(3);
            xdes_x[0]=xdes[0]; xdot_set_int_x[0]=xdot_set_int[0];
            xdes_x[1]=xdes[1]; xdot_set_int_x[1]=xdot_set_int[1];
            xdes_x[2]=xdes[2]; xdot_set_int_x[2]=xdot_set_int[2];

            Vector xdes_o(3), xdot_set_int_o(3);
            xdes_o[0]=xdes[6]*xdes[3]; xdot_set_int_o[0]=xdot_set_int[6]*xdot_set_int[3];
            xdes_o[1]=xdes[6]*xdes[4]; xdot_set_int_o[1]=xdot_set_int[6]*xdot_set_int[4];
            xdes_o[2]=xdes[6]*xdes[5]; xdot_set_int_o[2]=xdot_set_int[6]*xdot_set_int[5];

            // prevent the virtual target from getting too far
            if ((norm(xdot_set_int_x-xdes_x)>CARTCTRL_DEFAULT_TASKVEL_BALLPOS) ||
                (norm(xdot_set_int_o-xdes_o)>CTRL_DEG2RAD*CARTCTRL_DEFAULT_TASKVEL_BALLORIEN))
            {
                taskRefVelTargetGen->reset(xdes);
                xdot_set_int=taskRefVelTargetGen->integrate(xdot_set);
            }

            goTo(IKINCTRL_POSE_FULL,xdot_set_int,0.0);
            taskRefVelPeriodCnt=0;
        }

        // get the current target pose
        if (getNewTarget())
        {    
            if (!executingTraj)
                ctrl->restart(fb);

            executingTraj=true; // onset of new trajectory
        }
            
        if (executingTraj)
        {
            // limb control loop
            if (taskVelModeOn)
                ctrl->iterate(xdes,qdes,xdot_set);
            else
                ctrl->iterate(xdes,qdes);

            // send joints velocities to the robot [deg/s]
            sendVelocity(CTRL_RAD2DEG*ctrl->get_qdot());

            // handle the end-trajectory event
            if (ctrl->isInTarget())
            {
                executingTraj=false;
                motionDone   =true;

                stopLimbVel();

                // switch the solver status to one shot mode
                // if it is the case
                if (!trackingMode && (rxToken==txToken))
                    setTrackingMode(false);
            }
        }
    
        // update the stamp anyway
        txInfo.update();

        // streams out the end-effector pose
        if (portState.getOutputCount()>0)
        {
            portState.prepare()=chain->EndEffPose();
            portState.setEnvelope(txInfo);
            portState.write();
        }
    
        // end of critical code
        mutex.post();
    }
    else if ((++connectCnt)*getRate()>CARTCTRL_CONNECT_TMO)
    {
        connectToSolver();
        connectCnt=0;
    }
}


/************************************************************************/
void ServerCartesianController::threadRelease()
{
    fprintf(stdout,"Stopping %s\n",ctrlName.c_str());

    if (connected)
        stopLimbVel();
}


/************************************************************************/
bool ServerCartesianController::open(Searchable &config)
{
    fprintf(stdout,"***** Configuring cartesian controller *****\n");

    // GENERAL group
    Bottle &optGeneral=config.findGroup("GENERAL");
    if (optGeneral.isNull())
    {
        fprintf(stdout,"GENERAL group is missing\n");
        close();

        return false;
    }

    fprintf(stdout,"Acquiring options for group GENERAL...\n");

    // scan for required params
    if (optGeneral.check("SolverNameToConnect"))
        slvName=optGeneral.find("SolverNameToConnect").asString();
    else
    {
        fprintf(stdout,"SolverNameToConnect option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("KinematicPart"))
    {
        kinPart=optGeneral.find("KinematicPart").asString();

        if (kinPart!="arm" && kinPart!="leg")
        {
            fprintf(stdout,"Attempt to instantiate an unknown kinematic part\n");
            fprintf(stdout,"Available parts are: arm, leg\n");
            close();

            return false;
        }
    }
    else
    {
        fprintf(stdout,"KinematicPart option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("KinematicType"))
    {
        kinType=optGeneral.find("KinematicType").asString();

        if ((kinType!="left") && (kinType!="right"))
        {
            fprintf(stdout,"Attempt to instantiate an unknown kinematic type\n");
            fprintf(stdout,"Available types are: left, right\n");
            close();

            return false;
        }
    }
    else
    {
        fprintf(stdout,"KinematicType option is missing\n");
        close();

        return false;
    }

    if (optGeneral.check("NumberOfDrivers"))
    {
        if (!(numDrv=optGeneral.find("NumberOfDrivers").asInt()))
        {
            fprintf(stdout,"NumberOfDrivers shall be positive\n");
            close();

            return false;
        }
    }
    else
    {
        fprintf(stdout,"NumberOfDrivers option is missing\n");
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
        ctrlName=ctrlName+kinPart+kinType;
        fprintf(stdout,"default ControllerName assumed: %s",ctrlName.c_str());
    }

    if (optGeneral.check("ControllerPeriod"))
        setRate(optGeneral.find("ControllerPeriod").asInt());

    taskRefVelPeriodFactor=optGeneral.check("TaskRefVelPeriodFactor",
                                            Value(CARTCTRL_DEFAULT_TASKVEL_PERFACTOR)).asInt();

    // scan DRIVER groups
    for (int i=0; i<numDrv; i++)
    {
        char entry[255];
        sprintf(entry,"DRIVER_%d",i);

        Bottle &optDrv=config.findGroup(entry);
        if (optDrv.isNull())
        {
            fprintf(stdout,"%s group is missing\n",entry);
            close();

            return false;
        }

        fprintf(stdout,"Acquiring options for group %s...\n",entry);

        DriverDescriptor desc;

        if (optDrv.check("Key"))
            desc.key=optDrv.find("Key").asString();
        else
        {
            fprintf(stdout,"Key option is missing\n");
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
                fprintf(stdout,"Attempt to select an unknown mapping order\n");
                fprintf(stdout,"Available orders are: direct, reversed\n");
                close();

                return false;
            }
        }
        else
        {
            fprintf(stdout,"JointsOrder option is missing\n");
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
            fprintf(stdout,"MinAbsVels option is missing ... using default values\n");
            desc.useDefaultMinAbsVel=true;
        }

        lDsc.push_back(desc);
    }

    // instantiate kinematic object
    if (kinPart=="arm")
        limb=new iCubArm(kinType.c_str());
    else if (kinPart=="leg")
        limb=new iCubLeg(kinType.c_str());

    chain=limb->asChain();

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
    lLim.clear();
    lEnc.clear();
    lVel.clear();
    lJnt.clear();
    lRmp.clear();

    if (limb!=NULL)
        delete limb;

    if (ctrl!=NULL)
        delete ctrl;

    closePorts();

    contextMap.clear();

    return closed=true;
}


/************************************************************************/
bool ServerCartesianController::attachAll(const PolyDriverList &p)
{
    drivers=p;
    int nd=drivers.size();

    fprintf(stdout,"***** Attaching drivers to cartesian controller %s *****\n",ctrlName.c_str());
    fprintf(stdout,"Received list of %d driver(s)\n",nd);

    if (nd!=numDrv)
    {
        fprintf(stdout,"Expected list of %d driver(s)\n",numDrv);
        return false;
    }

    int remainingJoints=chain->getN();

    for (int i=0; i<numDrv; i++)
    {
        fprintf(stdout,"Acquiring info on driver %s... ",lDsc[i].key.c_str());

        // check if what we require is present within the given list
        int j;
        for (j=0; j<drivers.size(); j++)
            if (lDsc[i].key==drivers[j]->key)
                break;

        if (j>=drivers.size())
        {
            fprintf(stdout,"None of provided drivers is of type %s\n",lDsc[i].key.c_str());
            return false;
        }

        // acquire interfaces and driver's info
        if (drivers[j]->poly->isValid())
        {
            fprintf(stdout,"ok\n");

            IControlLimits   *lim;
            IEncoders        *enc;
            IVelocityControl *vel;
            int               joints;

            drivers[j]->poly->view(lim);
            drivers[j]->poly->view(enc);
            drivers[j]->poly->view(vel);

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
            else if (lDsc[i].minAbsVels.length()<joints)
            {
                Vector tmp=lDsc[i].minAbsVels;
                lDsc[i].minAbsVels.resize(joints,0.0);

                for (int k=0; k<tmp.length(); k++)
                    lDsc[i].minAbsVels[k]=tmp[k];
            }

            lLim.push_back(lim);
            lEnc.push_back(enc);
            lVel.push_back(vel);
            lJnt.push_back(joints);
            lRmp.push_back(rmpTmp);
        }
        else
        {
            fprintf(stdout,"error\n");
            return false;
        }
    }

    // exclude acceleration constraints by fixing
    // thresholds at high values
    for (int i=0; i<numDrv; i++)
        for (int j=0; j<lJnt[i]; j++)
            lVel[i]->setRefAcceleration(j,CARTCTRL_MAX_ACCEL);

    // create controller
    newController();

    // init task-space reference velocity
    xdot_set.resize(7,0.0);

    // create the target generator for
    // task-space reference velocity
    taskRefVelTargetGen=new Integrator(taskRefVelPeriodFactor*(getRate()/1000.0),ctrl->get_x());
    taskRefVelPeriodCnt=0;

    // this line shall be put before any
    // call to attached-dependent methods
    attached=true;

    connectToSolver();

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

    fprintf(stdout,"%s: Checking if cartesian solver %s is alive... ",ctrlName.c_str(),slvName.c_str());

    bool ok=Network::exists(portSlvName.c_str(),true);

    fprintf(stdout,"%s\n",ok?"ok":"failed");

    return ok;
}


/************************************************************************/
bool ServerCartesianController::connectToSolver()
{
    if (attached && !connected && pingSolver())
    {        
        fprintf(stdout,"%s: Connecting to cartesian solver %s...\n",ctrlName.c_str(),slvName.c_str());

        ConstString portSlvName="/";
        portSlvName=portSlvName+slvName;

        bool ok=true;

        ok&=Network::connect((portSlvName+"/out").c_str(),portSlvIn.getName().c_str());
        ok&=Network::connect(portSlvOut.getName().c_str(),(portSlvName+"/in").c_str());
        ok&=Network::connect(portSlvRpc.getName().c_str(),(portSlvName+"/rpc").c_str());

        if (!ok)
            return false;

        // this line shall be put before any
        // call to connected-dependent methods
        connected=true;

        // keep solver and controller status aligned at start-up
        Vector curDof, tmpDof;
        getDOF(curDof);
        setDOF(curDof,tmpDof);
        setTrackingMode(trackingMode);
        alignJointsBounds();
        
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goTo(unsigned int _ctrlPose, const Vector &xd,
                                     const double t, const bool latchToken)
{    
    if (connected && (ctrl->get_dim()!=0))
    {
        motionDone=false;
        
        ctrl->set_ctrlPose(ctrlPose=_ctrlPose);

        // update trajectory execution time just if required
        if (t>0.0)
            setTrajTime(t);

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
bool ServerCartesianController::setTrackingMode(const bool f)
{
    if (connected)
    {
        Bottle command, reply;

        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_MODE);
        if (f)
            command.addInt(IKINSLV_VOCAB_VAL_MODE_TRACK);
        else
            command.addInt(IKINSLV_VOCAB_VAL_MODE_SINGLE);

        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            return false;
        }

        if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
        {
            trackingMode=f;
            return true;
        }
        else
            return false;        
    }
    else
        return false;
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
bool ServerCartesianController::getPose(Vector &x, Vector &o)
{
    if (attached)
    {
        Vector pose=chain->EndEffPose();
    
        x.resize(3);
        o.resize(pose.length()-x.length());
    
        for (int i=0; i<x.length(); i++)
            x[i]=pose[i];
    
        for (int i=0; i<o.length(); i++)
            o[i]=pose[x.length()+i];
    
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getPose(const int axis, Vector &x, Vector &o)
{
    if (attached)
    {
        if (axis<(int)chain->getN())
        {
            Matrix H=chain->getH(axis,true);
    
            x.resize(3);
    
            for (int i=0; i<x.length(); i++)
                x[i]=H(i,3);
    
            o=dcm2axis(H);
    
            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPose(const Vector &xd, const Vector &od, const double t)
{
    if (connected)
    {
        Vector _xd(xd.length()+od.length());
    
        for (int i=0; i<xd.length(); i++)
            _xd[i]=xd[i];
    
        for (int i=0; i<od.length(); i++)
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
        taskVelModeOn=false;
        return goTo(IKINCTRL_POSE_XYZ,xd,t);
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::goToPoseSync(const Vector &xd, const Vector &od, const double t)
{
    return goToPose(xd,od,t);
}


/************************************************************************/
bool ServerCartesianController::goToPositionSync(const Vector &xd, const double t)
{
    return goToPosition(xd,t);
}


/************************************************************************/
bool ServerCartesianController::getDesired(Vector &xdhat, Vector &odhat, Vector &qdhat)
{
    if (connected)
    {
        xdhat.resize(3);
        odhat.resize(xdes.length()-3);

        for (int i=0; i<xdhat.length(); i++)
            xdhat[i]=xdes[i];

        for (int i=0; i<odhat.length(); i++)
            odhat[i]=xdes[xdhat.length()+i];

        qdhat.resize(chain->getN());
        int cnt=0;

        for (unsigned int i=0; i<chain->getN(); i++)
            if ((*chain)[i].isBlocked())
                qdhat[i]=CTRL_RAD2DEG*chain->getAng(i);
            else
                qdhat[i]=CTRL_RAD2DEG*qdes[cnt++];

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::askForPose(const Vector &xd, const Vector &od,
                                           Vector &xdhat, Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;

    // prepare command
    Vector tg(xd.length()+od.length());
    for (int i=0; i<xd.length(); i++)
        tg[i]=xd[i];

    for (int i=0; i<od.length(); i++)
        tg[xd.length()+i]=od[i];
    
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,tg);
    addPoseOption(command,IKINCTRL_POSE_FULL);

    // send command and wait for reply
    if (!portSlvRpc.write(command,reply))
    {
        fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ServerCartesianController::askForPose(const Vector &q0, const Vector &xd,
                                           const Vector &od, Vector &xdhat,
                                           Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;

    // prepare command
    Vector tg(xd.length()+od.length());
    for (int i=0; i<xd.length(); i++)
        tg[i]=xd[i];

    for (int i=0; i<od.length(); i++)
        tg[xd.length()+i]=od[i];

    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,tg);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_Q,q0);
    addPoseOption(command,IKINCTRL_POSE_FULL);

    // send command and wait for reply
    if (!portSlvRpc.write(command,reply))
    {
        fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ServerCartesianController::askForPosition(const Vector &xd, Vector &xdhat,
                                               Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,xd);
    addPoseOption(command,IKINCTRL_POSE_XYZ);

    // send command and wait for reply
    if (!portSlvRpc.write(command,reply))
    {
        fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ServerCartesianController::askForPosition(const Vector &q0, const Vector &xd,
                                               Vector &xdhat, Vector &odhat, Vector &qdhat)
{
    if (!connected)
        return false;

    Bottle command, reply;

    // prepare command
    command.addVocab(IKINCARTCTRL_VOCAB_CMD_ASK);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_XD,xd);
    addVectorOption(command,IKINCARTCTRL_VOCAB_OPT_Q,q0);
    addPoseOption(command,IKINCTRL_POSE_XYZ);

    // send command and wait for reply
    if (!portSlvRpc.write(command,reply))
    {
        fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
        return false;
    }

    return getDesiredOption(reply,xdhat,odhat,qdhat);
}


/************************************************************************/
bool ServerCartesianController::getDOF(Vector &curDof)
{
    if (connected)
    {
        curDof.resize(chain->getN());
    
        for (unsigned int i=0; i<chain->getN(); i++)
            curDof[i]=!(*chain)[i].isBlocked();
    
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
        Bottle command, reply;
    
        // begin of critical code
        mutex.wait();
    
        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_DOF);
        Bottle &txDofPart=command.addList();
        for (int i=0; i<newDof.length(); i++)
            txDofPart.addInt((int)newDof[i]);
    
        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());

            // end of critical code
            mutex.post();
            return false;
        }

        // update chain's links
        // skip the first ack/nack vocab
        Bottle *rxDofPart=reply.get(1).asList();
        curDof.resize(rxDofPart->size());
        for (int i=0; i<rxDofPart->size(); i++)
            if (curDof[i]=rxDofPart->get(i).asInt())
                chain->releaseLink(i);
            else
                chain->blockLink(i);
            
        // update controller
        newController();

        // end of critical code
        mutex.post();
        
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getRestPos(Vector &curRestPos)
{
    if (connected)
    {
        Bottle command, reply;
    
        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_POS);
    
        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            return false;
        }

        Bottle *rxRestPart=reply.get(1).asList();
        curRestPos.resize(rxRestPart->size());
        for (int i=0; i<rxRestPart->size(); i++)
            curRestPos[i]=rxRestPart->get(i).asDouble();
            
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setRestPos(const Vector &newRestPos, Vector &curRestPos)
{
    if (connected)
    {
        Bottle command, reply;
    
        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_POS);
        Bottle &txRestPart=command.addList();
        for (int i=0; i<newRestPos.length(); i++)
            txRestPart.addDouble(newRestPos[i]);
    
        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            return false;
        }

        Bottle *rxRestPart=reply.get(1).asList();
        curRestPos.resize(rxRestPart->size());
        for (int i=0; i<rxRestPart->size(); i++)
            curRestPos[i]=rxRestPart->get(i).asDouble();
            
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getRestWeights(Vector &curRestWeights)
{
    if (connected)
    {
        Bottle command, reply;
    
        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_GET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_WEIGHTS);
    
        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            return false;
        }

        Bottle *rxRestPart=reply.get(1).asList();
        curRestWeights.resize(rxRestPart->size());
        for (int i=0; i<rxRestPart->size(); i++)
            curRestWeights[i]=rxRestPart->get(i).asDouble();
            
        return true;
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
        Bottle command, reply;
    
        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_REST_WEIGHTS);
        Bottle &txRestPart=command.addList();
        for (int i=0; i<newRestWeights.length(); i++)
            txRestPart.addDouble(newRestWeights[i]);
    
        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            return false;
        }

        Bottle *rxRestPart=reply.get(1).asList();
        curRestWeights.resize(rxRestPart->size());
        for (int i=0; i<rxRestPart->size(); i++)
            curRestWeights[i]=rxRestPart->get(i).asDouble();
            
        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::getLimits(const int axis, double *min, double *max)
{
    if (connected && (min!=NULL) && (max!=NULL))
    {
        if (axis<(int)chain->getN())
        {
            *min=CTRL_RAD2DEG*(*chain)[axis].getMin();
            *max=CTRL_RAD2DEG*(*chain)[axis].getMax();

            return true;
        }
        else
            return false;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setLimits(const int axis, const double min, const double max)
{
    if (connected)
    {
        Bottle command, reply;

        // prepare command
        command.addVocab(IKINSLV_VOCAB_CMD_SET);
        command.addVocab(IKINSLV_VOCAB_OPT_LIM);
        command.addInt(axis);
        command.addDouble(min);
        command.addDouble(max);

        // send command to solver and wait for reply
        if (!portSlvRpc.write(command,reply))
        {
            fprintf(stdout,"%s error: unable to get reply from solver!\n",slvName.c_str());
            return false;
        }

        if (reply.get(0).asVocab()==IKINSLV_VOCAB_REP_ACK)
        {
            // align local joint's limits
            (*chain)[axis].setMin(CTRL_DEG2RAD*min);
            (*chain)[axis].setMax(CTRL_DEG2RAD*max);

            return true;
        }
        else
            return false;        
    }
    else
        return false;
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
bool ServerCartesianController::setTrajTime(const double t)
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
bool ServerCartesianController::setInTargetTol(const double tol)
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
bool ServerCartesianController::getJointsVelocities(Vector &qdot)
{
    if (connected)
    {
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
        Matrix J=ctrl->get_J();
        Vector taskVel;

        if (!J.rows() || (J.cols()!=velCmd.length()))
            taskVel.resize(7,0.0);
        else
            taskVel=J*(CTRL_DEG2RAD*velCmd);

        xdot.resize(3);
        odot.resize(taskVel.length()-xdot.length());
    
        for (int i=0; i<xdot.length(); i++)
            xdot[i]=taskVel[i];
    
        for (int i=0; i<odot.length(); i++)
            odot[i]=taskVel[xdot.length()+i];

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::setTaskVelocities(const Vector &xdot, const Vector &odot)
{
    if (connected)
    {
        for (int i=0; i<3; i++)
            xdot_set[i]=xdot[i];

        for (int i=3; i<xdot_set.length(); i++)
            xdot_set[i]=odot[i-3];

        if (norm(xdot_set)==0.0)
        {
            stopControl();
            return true;
        }

        if (!taskVelModeOn)
        {
            taskRefVelTargetGen->reset(chain->EndEffPose());
            taskRefVelPeriodCnt=0;
        }

        return taskVelModeOn=true;
    }
    else
        return false;
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
bool ServerCartesianController::waitMotionDone(const double period, const double timeout)
{
    bool done=false;
    double t0=Time::now();

    while (!done)
    {
        Time::delay(period);

        if (!checkMotionDone(&done) || (timeout>0.0) && ((Time::now()-t0)>timeout))
            return false;
    }

    return true;
}


/************************************************************************/
bool ServerCartesianController::stopControl()
{
    if (connected)
    {
        executingTraj=false;
        taskVelModeOn=false;
        motionDone   =true;
        
        stopLimbVel();
        velCmd=0.0;
        xdot_set=0.0;

        txTokenLatchedStopControl=txToken;
        skipSlvRes=true;

        return true;
    }
    else
        return false;
}


/************************************************************************/
bool ServerCartesianController::storeContext(int *id)
{
    if (attached && (id!=NULL))
    {
        Context &context=contextMap[contextIdCnt];

        getDOF(context.dof);
        getRestPos(context.restPos);
        getRestWeights(context.restWeights);

        context.limits.resize(chain->getN(),2);
        for (unsigned int axis=0; axis<chain->getN(); axis++)
        {
            double min,max;
            getLimits(axis,&min,&max);
            context.limits(axis,0)=min;
            context.limits(axis,1)=max;
        }

        getTrajTime(&context.trajTime);
        getInTargetTol(&context.tol);
        getTrackingMode(&context.mode);

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
        map<int,Context>::iterator itr=contextMap.find(id);

        if (itr!=contextMap.end())
        {
            Context &context=itr->second;

            setDOF(context.dof,context.dof);
            setRestPos(context.restPos,context.restPos);
            setRestWeights(context.restWeights,context.restWeights);

            for (unsigned int axis=0; axis<chain->getN(); axis++)
                setLimits(axis,context.limits(axis,0),context.limits(axis,1));

            setTrajTime(context.trajTime);
            setInTargetTol(context.tol);
            setTrackingMode(context.mode);

            return true;
        }
        else
            return false;
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
Stamp ServerCartesianController::getLastInputStamp()
{
    return txInfo;
}


/************************************************************************/
ServerCartesianController::~ServerCartesianController()
{
    close();
}



