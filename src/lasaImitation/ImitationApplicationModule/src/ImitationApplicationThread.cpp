// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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

 
#include "ImitationApplicationThread.h"

#include <string.h>
#include <iostream>
using namespace std;

#include <yarp/os/Network.h>
using namespace yarp::os;

#include "StdTools/Various.h"

//#define FAKE_NETWORK


ImitationApplicationThread::ImitationApplicationThread(int period, const char* baseName)
:RateThread(period)
{
    mPeriod = period;
    strncpy(mBaseName,baseName,256);
}

ImitationApplicationThread::~ImitationApplicationThread()
{}

bool ImitationApplicationThread::threadInit()
{
    mMode = MODE_BASICCOMMAND;
    mMode = MODE_STATEMACHINE;
    
    char portname[256];
    snprintf(portname,        256,"/%s/Wiimote",mBaseName);
    mWiimotePort.open(portname);
    mWiimotePort.setStrict();
    
    snprintf(mSrcCtrlPortName[PID_Velocity],        256,"/%s/VC",mBaseName);
    snprintf(mSrcCtrlPortName[PID_Robot],           256,"/%s/RC",mBaseName);
    snprintf(mSrcCtrlPortName[PID_3DMouse],         256,"/%s/TC3D",mBaseName);
    snprintf(mSrcCtrlPortName[PID_Touchpad],        256,"/%s/TCTP",mBaseName);
    snprintf(mSrcCtrlPortName[PID_GMMRight],        256,"/%s/GMMR",mBaseName);
    snprintf(mSrcCtrlPortName[PID_GMMLeft],         256,"/%s/GMML",mBaseName);
    snprintf(mSrcCtrlPortName[PID_RefTransRight],   256,"/%s/RefR",mBaseName);
    snprintf(mSrcCtrlPortName[PID_RefTransLeft],    256,"/%s/RefL",mBaseName);
    
    
    
    snprintf(mDstCtrlPortName[PID_Velocity],        256,"/VelocityController/rpc");
    snprintf(mDstCtrlPortName[PID_Robot],           256,"/RobotController/rpc");
    snprintf(mDstCtrlPortName[PID_3DMouse],         256,"/TouchController/3DMouse/rpc");
    snprintf(mDstCtrlPortName[PID_Touchpad],        256,"/TouchController/Touchpad/rpc");
    snprintf(mDstCtrlPortName[PID_GMMRight],        256,"/GaussianMixtureModel/Right/rpc");
    snprintf(mDstCtrlPortName[PID_GMMLeft],         256,"/GaussianMixtureModel/Left/rpc");
    snprintf(mDstCtrlPortName[PID_RefTransRight],   256,"/RefTransform/Right/rpc");
    snprintf(mDstCtrlPortName[PID_RefTransLeft],    256,"/RefTransform/Left/rpc");

    //for(int i=0;i<PID_SIZE;i++){
    //    cout << mSrcCtrlPortName[i] << " "<<mDstCtrlPortName[i]<<endl;
    //}
    
    mVelocityControllerPort.open(   mSrcCtrlPortName[PID_Velocity]);
    mRobotControllerPort.open(      mSrcCtrlPortName[PID_Robot]);
    m3DMouseControllerPort.open(    mSrcCtrlPortName[PID_3DMouse]);
    mTouchpadControllerPort.open(   mSrcCtrlPortName[PID_Touchpad]);
    mGMMRightPort.open(             mSrcCtrlPortName[PID_GMMRight]);
    mGMMLeftPort.open(              mSrcCtrlPortName[PID_GMMLeft]);
    mRefTransRightPort.open(        mSrcCtrlPortName[PID_RefTransRight]);
    mRefTransLeftPort.open(         mSrcCtrlPortName[PID_RefTransLeft]);

    mPorts[PID_Velocity]        = &mVelocityControllerPort;
    mPorts[PID_Robot]           = &mRobotControllerPort;
    mPorts[PID_3DMouse]         = &m3DMouseControllerPort;
    mPorts[PID_Touchpad]        = &mTouchpadControllerPort;
    mPorts[PID_GMMRight]        = &mGMMRightPort;
    mPorts[PID_GMMLeft]         = &mGMMLeftPort;
    mPorts[PID_RefTransRight]   = &mRefTransRightPort;
    mPorts[PID_RefTransLeft]    = &mRefTransLeftPort;

    
    snprintf(mSrcPortName[SPID_Touchpad],           256,"/TouchController/Touchpad/velocity");
    snprintf(mSrcPortName[SPID_3DMouse],            256,"/TouchController/3DMouse/velocity");
    snprintf(mSrcPortName[SPID_TouchpadSignal],     256,"/TouchController/Touchpad/signals");
    snprintf(mSrcPortName[SPID_3DMouseSignal],      256,"/TouchController/3DMouse/signals");
    snprintf(mSrcPortName[SPID_RWristRef],          256,"/RobotController/currentWristRefR");
    snprintf(mSrcPortName[SPID_LWristRef],          256,"/RobotController/currentWristRefL");
    snprintf(mSrcPortName[SPID_RArmCartPos],        256,"/RobotController/currentCartPositionR");
    snprintf(mSrcPortName[SPID_LArmCartPos],        256,"/RobotController/currentCartPositionL");
    snprintf(mSrcPortName[SPID_EyeCartPos],         256,"/RobotController/currentCartEyeTargetPosition");
    snprintf(mSrcPortName[SPID_GMMRightCartPos],    256,"/GaussianMixtureModel/Right/output");
    snprintf(mSrcPortName[SPID_GMMLeftCartPos],     256,"/GaussianMixtureModel/Left/output");
    snprintf(mSrcPortName[SPID_RRefTransOutput],    256,"/RefTransform/Right/outputPose");
    snprintf(mSrcPortName[SPID_LRefTransOutput],    256,"/RefTransform/Left/outputPose");
    snprintf(mSrcPortName[SPID_MotionSensorsArm],   256,"/MotionSensors/left_arm");
    snprintf(mSrcPortName[SPID_MotionSensorsHand],  256,"/MotionSensors/left_glove");
    snprintf(mSrcPortName[SPID_Vision0],            256,"/micha/vision0:o");
    snprintf(mSrcPortName[SPID_Vision1],            256,"/micha/vision1:o");
        

    
    snprintf(mDstPortName[DPID_Touchpad],           256,"/TouchController/Touchpad/frameOfRef");
    snprintf(mDstPortName[DPID_3DMouse],            256,"/TouchController/3DMouse/frameOfRef");
    snprintf(mDstPortName[DPID_RArmDesCartVel],     256,"/RobotController/desiredCartVelocityR");
    snprintf(mDstPortName[DPID_LArmDesCartVel],     256,"/RobotController/desiredCartVelocityL");
    snprintf(mDstPortName[DPID_RArmDesCartPos],     256,"/RobotController/desiredCartPositionR");
    snprintf(mDstPortName[DPID_LArmDesCartPos],     256,"/RobotController/desiredCartPositionL");
    snprintf(mDstPortName[DPID_RWristDesCartVel],   256,"/RobotController/desiredCartWristVelocityR");
    snprintf(mDstPortName[DPID_LWristDesCartVel],   256,"/RobotController/desiredCartWristVelocityL");
    snprintf(mDstPortName[DPID_RArmHandPos],        256,"/RobotController/desiredHandPosR");
    snprintf(mDstPortName[DPID_LArmHandPos],        256,"/RobotController/desiredHandPosL");
    snprintf(mDstPortName[DPID_RArmJointsPos],      256,"/RobotController/desiredArmJointsR");
    snprintf(mDstPortName[DPID_LArmJointsPos],      256,"/RobotController/desiredArmJointsL");
    snprintf(mDstPortName[DPID_EyeInEyeDesCartPos], 256,"/RobotController/desiredCartEyeInEyePosition");
    snprintf(mDstPortName[DPID_EyeDesCartPos],      256,"/RobotController/desiredCartEyePosition");
    snprintf(mDstPortName[DPID_GMMRightSignal],     256,"/GaussianMixtureModel/Right/signals");
    snprintf(mDstPortName[DPID_GMMLeftSignal],      256,"/GaussianMixtureModel/Left/signals");
    snprintf(mDstPortName[DPID_RRefTransRef],       256,"/RefTransform/Right/inputRef");
    snprintf(mDstPortName[DPID_LRefTransRef],       256,"/RefTransform/Left/inputRef");

    InitStateMachine();

    mBasicCommand = BC_NONE;
    
    mWiimoteEvent.resize(11);
    mWiimoteEvent = 0;

    return true;
}
void ImitationApplicationThread::PrepareToStop(){
    /*mMutex.wait();
    mNextState = IA_STOP;
    mMutex.post();
    while(mState != IA_IDLE){
        Time::delay(double(mPeriod)*0.001);
        if(mState != IA_IDLE)
            cerr<<"Waiting to quit"<<endl;
    }*/
}

void ImitationApplicationThread::threadRelease()
{
    ConnectToNetwork(false);
    
    mWiimotePort.close();

    mVelocityControllerPort.close();
    mRobotControllerPort.close();
    m3DMouseControllerPort.close();
    mTouchpadControllerPort.close();
    mGMMRightPort.close();
    mGMMLeftPort.close();
    mRefTransRightPort.close();
    mRefTransLeftPort.close();

}

void ImitationApplicationThread::run()
{
    mMutex.wait();
    ClearCommands();

    ProcessWiimote();

    if(mMode == MODE_BASICCOMMAND){
        ProcessBasicCommand();
    }else{
        ProcessStateMachine();
    }
    
    SendCommands();
    mMutex.post();
    
}

bool ImitationApplicationThread::ProcessWiimote(bool flush){
    // Read data from input port
    bool res = false;
    mWiimoteEvent = 0;
    Vector *inputVec = mWiimotePort.read(false);
    while(inputVec!=NULL){
        mWiimoteEvent = *inputVec;
        //cout << "Wiimote says: "<<mWiimoteEvent.toString()<<endl;
        res = true;
        if(!flush) break;
        inputVec = mWiimotePort.read(false);
    }
    if(res){
        if(mMode == MODE_STATEMACHINE){
            bool bCmd = false;
            Bottle cmd; 
            if(mWiimoteEvent[0]>0.0){ cmd.fromString("ok");   bCmd = true;}
            if(mWiimoteEvent[8]>0.0){ cmd.fromString("abrt"); bCmd = true;}
            if(mWiimoteEvent[4]>0.0){ cmd.fromString("prev"); bCmd = true;}
            if(mWiimoteEvent[5]>0.0){ cmd.fromString("next"); bCmd = true;}
            if(bCmd){
                Bottle rep;
                respondToStateMachine(cmd,rep);
            }
        }
        
    }
    
    return res;
}

void ImitationApplicationThread::ProcessBasicCommand(){
    switch(mBasicCommand){
    case BC_NONE:
        break;
    case BC_INIT:
        ConnectToNetwork(true);
        AddCommand(PID_Velocity,"kd 0.1");
        AddCommand(PID_Robot,"iks None");
        break;
    case BC_CLEAR:
        RemAllConnexions();
        ConnectToNetwork(false);
        break;
    case BC_RUN:
        //AddCommand(PID_Robot,   "iks None");
        AddCommand(PID_Robot,   "run");
        AddCommand(PID_Velocity,"run");
        break;
    case BC_STOP:
        AddCommand(PID_Robot,   "iks None");
        AddCommand(PID_Velocity,"susp");
        AddCommand(PID_Robot,   "susp");
        break;
    case BC_REST:
        AddCommand(PID_Velocity,"rest");
        AddCommand(PID_Robot,   "susp");
        break;
    case BC_3DMOUSE_TO_NONE:
        RemConnexion(SPID_3DMouse, DPID_RArmDesCartVel);
        RemConnexion(SPID_3DMouse, DPID_LArmDesCartVel);
        break;
    case BC_3DMOUSE_TO_RIGHTARM_NONE:
        RemConnexion(SPID_3DMouse, DPID_RArmDesCartVel);
        AddCommand(PID_Robot,"iku RightArm");
        break;
    case BC_3DMOUSE_TO_LEFTARM_NONE:
        RemConnexion(SPID_3DMouse, DPID_LArmDesCartVel);
        AddCommand(PID_Robot,"iku LeftArm");
        break;
    case BC_3DMOUSE_TO_RIGHTARM:
        RemAllSrcConnexions(DPID_3DMouse);
        AddConnexion(SPID_3DMouse, DPID_RArmDesCartVel);
        AddCommand(PID_Robot,"iks RightArmSeq");
        break;
    case BC_3DMOUSE_TO_LEFTARM:
        RemAllSrcConnexions(DPID_3DMouse);
        AddConnexion(SPID_3DMouse, DPID_LArmDesCartVel);
        AddCommand(PID_Robot,"iks LeftArmSeq");
        break;
    case BC_TOUCHPAD_TO_RIGHTARM_NONE:
        RemConnexion(SPID_RWristRef, DPID_Touchpad);
        RemConnexion(SPID_Touchpad, DPID_RWristDesCartVel);
        AddCommand(PID_Robot,"iku RightWrist");
        break;
    case BC_TOUCHPAD_TO_RIGHTARM:
        AddConnexion(SPID_RWristRef, DPID_Touchpad);
        AddConnexion(SPID_Touchpad, DPID_RWristDesCartVel);
        AddCommand(PID_Robot,"iks RightWrist");
        break;
    case BC_SENSORS_TO_RIGHTARM:
        AddConnexion(SPID_MotionSensorsArm, DPID_RArmJointsPos);
        AddConnexion(SPID_MotionSensorsHand, DPID_RArmHandPos);        
        AddCommand(PID_Robot,"iku RightArm");
        AddCommand(PID_Robot,"iku RightWrist");
        AddCommand(PID_Robot,"iks Joints");
        break;
    case BC_SENSORS_TO_RIGHTARM_NONE:
        RemConnexion(SPID_MotionSensorsArm, DPID_RArmJointsPos);
        RemConnexion(SPID_MotionSensorsHand, DPID_RArmHandPos);        
        AddCommand(PID_Robot,"iku Joints");
        break;
    case BC_SENSORS_TO_LEFTARM:
        AddConnexion(SPID_MotionSensorsArm, DPID_LArmJointsPos);
        AddConnexion(SPID_MotionSensorsHand, DPID_LArmHandPos);        
        AddCommand(PID_Robot,"iku LeftArm");
        AddCommand(PID_Robot,"iku LeftWrist");
        AddCommand(PID_Robot,"iks Joints");
        break;
    case BC_SENSORS_TO_LEFTARM_NONE:
        RemConnexion(SPID_MotionSensorsArm, DPID_LArmJointsPos);
        RemConnexion(SPID_MotionSensorsHand, DPID_LArmHandPos);        
        AddCommand(PID_Robot,"iku Joints");
        //AddCommand(PID_Robot,"iks Rest");
        break;
    case BC_TRACK_NONE:
        RemAllSrcConnexions(DPID_EyeDesCartPos);
        RemAllSrcConnexions(DPID_EyeInEyeDesCartPos);
        AddCommand(PID_Robot,"iku Eye");
        break;
    case BC_TRACK_RIGHTARM:
        RemAllSrcConnexions(DPID_EyeInEyeDesCartPos);
        AddConnexion(SPID_RArmCartPos, DPID_EyeDesCartPos);
        AddCommand(PID_Robot,"iks Eye");
        break;
    case BC_TRACK_LEFTARM:
        RemAllSrcConnexions(DPID_EyeInEyeDesCartPos);
        AddConnexion(SPID_LArmCartPos, DPID_EyeDesCartPos);
        AddCommand(PID_Robot,"iks Eye");
        break;
    case BC_TRACK_OBJECT0:
        RemAllSrcConnexions(DPID_EyeDesCartPos);
        AddConnexion(SPID_Vision0, DPID_EyeInEyeDesCartPos);
        AddCommand(PID_Robot,"iks Eye");
        break;
    case BC_TRACK_OBJECT1:
        RemAllSrcConnexions(DPID_EyeDesCartPos);
        AddConnexion(SPID_Vision1, DPID_EyeInEyeDesCartPos);
        AddCommand(PID_Robot,"iks Eye");
        break;
    case BC_HAND_OPENRIGHT:
        AddCommand(PID_Robot,"rh open");
        break;
    case BC_HAND_OPENLEFT:
        AddCommand(PID_Robot,"lh open");
        break;
    case BC_HAND_CLOSERIGHT:
        AddCommand(PID_Robot,"rh close");
        break;
    case BC_HAND_CLOSELEFT:
        AddCommand(PID_Robot,"lh close");
        break;
    case BC_EYETARGET_TO_NONE:
        RemConnexion(SPID_EyeCartPos, DPID_RArmDesCartPos);
        RemConnexion(SPID_EyeCartPos, DPID_LArmDesCartPos);
        break;
    case BC_EYETARGET_TO_RIGHTARM:
        AddConnexion(SPID_EyeCartPos, DPID_RArmDesCartPos);
        AddCommand(PID_Robot,"iku RightArm");
        AddCommand(PID_Robot,"iks RightArmPos");
        break;
    case BC_EYETARGET_TO_LEFTARM:
        AddConnexion(SPID_EyeCartPos, DPID_LArmDesCartPos);
        AddCommand(PID_Robot,"iku LeftArm");
        AddCommand(PID_Robot,"iks LeftArmPos");
        break;
    case BC_GMM_LEARN_RIGHT:
    case BC_GMM_LEARN_LEFT:
    case BC_GMM_LEARN_CORR_RIGHT:
    case BC_GMM_LEARN_CORR_LEFT:
        {PortId pid = ((mBasicCommand==BC_GMM_LEARN_RIGHT)||(mBasicCommand==BC_GMM_LEARN_CORR_RIGHT)?PID_GMMRight:PID_GMMLeft);
        AddCommand(pid,"set k 5");
        AddCommand(pid,"set ts 1.0");
        if((mBasicCommand==BC_GMM_LEARN_CORR_RIGHT)||(mBasicCommand==BC_GMM_LEARN_CORR_LEFT)){
            AddCommand(pid,"set pmode weighted");
            AddCommand(pid,"set imode load");
        }else{
            AddCommand(pid,"set pmode simple");
            AddCommand(pid,"set imode timesplit");
        }
        AddCommand(pid,"set demoLength 100");
        AddCommand(pid,"run process");
        AddCommand(pid,"run learning");
        AddCommand(pid,"run test");
        AddCommand(pid,"run gnuplot");}
        break;
    case BC_GMM_REC_RIGHT_START:
    case BC_GMM_REC_LEFT_START:
        {PortId pid = (mBasicCommand==BC_GMM_REC_RIGHT_START?PID_GMMRight:PID_GMMLeft);
        if(mBasicCommandParams.length()>0)
            AddCommand(pid,"set demoId",mBasicCommandParams.c_str());
        AddCommand(pid,"set rmode rec");
        AddCommand(pid,"run start");}
        break;
    case BC_GMM_CORR_RIGHT_START:
    case BC_GMM_CORR_LEFT_START:
        {PortId pid = (mBasicCommand==BC_GMM_CORR_RIGHT_START?PID_GMMRight:PID_GMMLeft);
        if(mBasicCommandParams.length()>0)
            AddCommand(pid,"set demoId",mBasicCommandParams.c_str());
        if(pid == PID_GMMRight){
            AddConnexion(SPID_TouchpadSignal, DPID_GMMRightSignal);
            AddConnexion(SPID_RRefTransOutput, DPID_RArmDesCartPos);
            AddCommand(PID_Robot,"iks RightArmSeq");
        }else{
            
        }
        AddCommand(pid,"set pmode weighted");
        AddCommand(pid,"set rmode corr");
        AddCommand(pid,"run start");}
        break;
    case BC_GMM_RIGHT_STOP:
    case BC_GMM_LEFT_STOP:
        {PortId pid = (mBasicCommand==BC_GMM_RIGHT_STOP?PID_GMMRight:PID_GMMLeft);
        AddCommand(pid,"run stop");}
        break;
    case BC_GMM_REPRO_RIGHT_START:
    case BC_GMM_REPRO_LEFT_START:
        {PortId pid = (mBasicCommand==BC_GMM_REPRO_RIGHT_START?PID_GMMRight:PID_GMMLeft);
        if(pid == PID_GMMRight){
            AddConnexion(SPID_RRefTransOutput, DPID_RArmDesCartPos);
            AddCommand(PID_Robot,"iks RightArmSeq");
        }else{
            AddConnexion(SPID_LRefTransOutput, DPID_LArmDesCartPos);
            AddCommand(PID_Robot,"iks LeftArmSeq");
        }
        //AddCommand(pid,"set reproTime 10.0");
        AddCommand(pid,"set rmode repro");
        AddCommand(pid,"run start");}
        break;
    case BC_GMM_LOAD_RIGHT:
    case BC_GMM_LOAD_LEFT:
        {PortId pid = (mBasicCommand==BC_GMM_LOAD_RIGHT?PID_GMMRight:PID_GMMLeft);
        if(mBasicCommandParams.length()>0)
            AddCommand(pid,"load",mBasicCommandParams.c_str());
        }
        break;
    case BC_GMM_DEMONAME_RIGHT:
        AddCommand(PID_GMMRight,"set demoPath",mBasicCommandParams.c_str());
        break;
    case BC_GMM_DEMONAME_LEFT:
        AddCommand(PID_GMMLeft,"set demoPath",mBasicCommandParams.c_str());
        break;
    case BC_GMM_CORRNAME_RIGHT:
        AddCommand(PID_GMMRight,"set corrPath",mBasicCommandParams.c_str());
        break;
    case BC_GMM_CORRNAME_LEFT:
        AddCommand(PID_GMMLeft,"set corrPath",mBasicCommandParams.c_str());
        break;
    case BC_REF_RIGHT_NONE:
    case BC_REF_LEFT_NONE:
        {PortId pid = (mBasicCommand==BC_REF_RIGHT_NONE?PID_RefTransRight:PID_RefTransLeft);
        AddCommand(pid,"ref off");
        }
        break;
    case BC_REF_RIGHT_OBJ0:
    case BC_REF_LEFT_OBJ0:
        {PortId pid = (mBasicCommand==BC_REF_RIGHT_OBJ0?PID_RefTransRight:PID_RefTransLeft);
        AddConnexion(SPID_EyeCartPos,(mBasicCommand==BC_REF_RIGHT_OBJ0?DPID_RRefTransRef:DPID_LRefTransRef));
        AddCommand(pid,"ref on");
        AddCommand(pid,"ori off");
        AddCommand(pid,"lock off");
        }
        break;
    case BC_REF_RIGHT_OBJ1:
    case BC_REF_LEFT_OBJ1:
        {PortId pid = (mBasicCommand==BC_REF_RIGHT_OBJ1?PID_RefTransRight:PID_RefTransLeft);
        AddConnexion(SPID_EyeCartPos,(mBasicCommand==BC_REF_RIGHT_OBJ1?DPID_RRefTransRef:DPID_LRefTransRef));
        AddCommand(pid,"ref on");
        AddCommand(pid,"ori off");
        AddCommand(pid,"lock off");
        }
        break;
    case BC_REF_RIGHT_LEFTHAND:
    case BC_REF_LEFT_RIGHTHAND:
        {PortId pid = (mBasicCommand==BC_REF_RIGHT_LEFTHAND?PID_RefTransRight:PID_RefTransLeft);
        AddConnexion((mBasicCommand==BC_REF_RIGHT_LEFTHAND?SPID_LArmCartPos:SPID_RArmCartPos),(mBasicCommand==BC_REF_RIGHT_LEFTHAND?DPID_RRefTransRef:DPID_LRefTransRef));
        AddCommand(pid,"ref on");
        AddCommand(pid,"ori off");
        AddCommand(pid,"lock off");
        }
        break;
    case BC_REF_RIGHT_LOCK:
    case BC_REF_LEFT_LOCK:
        {PortId pid = (mBasicCommand==BC_REF_RIGHT_LOCK?PID_RefTransRight:PID_RefTransLeft);
        AddCommand(pid,"lock on");
        }
        break;
    case BC_REF_RIGHT_UNLOCK:
    case BC_REF_LEFT_UNLOCK:
        {PortId pid = (mBasicCommand==BC_REF_RIGHT_UNLOCK?PID_RefTransRight:PID_RefTransLeft);
        AddCommand(pid,"lock off");
        }
        break;
    }
    
    mBasicCommand       = BC_NONE;
    mBasicCommandParams = "";
}



void ImitationApplicationThread::ConnectToNetwork(bool bConnect){
    cout << (bConnect?"Connecting to network":"Disonnecting from network")<<endl;
    for(int i=0;i<PID_SIZE;i++){
        cout << "  From: "<< mSrcCtrlPortName[i] <<" to: "<< mDstCtrlPortName[i] <<endl;
#ifndef FAKE_NETWORK    
        if(bConnect){
            if(!Network::isConnected(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                if(!Network::connect(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                    cerr << "Error: Unable to connect "<<mSrcCtrlPortName[i] <<" to "<<mDstCtrlPortName[i]<<endl;
        }else{
            if( Network::isConnected(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                Network::disconnect(mSrcCtrlPortName[i],mDstCtrlPortName[i]);
        }
#endif
    }
    cout << "Done..."<<endl;
}

void ImitationApplicationThread::ClearCommands(){
    mCommandsType.clear();
    mCommands.clear();
    mCommandsPort.clear();
    mConnexionsSrcPort.clear();
    mConnexionsDstPort.clear();
}

void ImitationApplicationThread::AddCommand(PortId port, const char *cmd, const char *params){
    if(cmd!=NULL){
        if(cmd[0]!=0){
            mCommandsType.push_back(0);
            if((params!=NULL)&&(params[0]!=0)){
                char cmd2[512];
                snprintf(cmd2,512,"%s %s",cmd,params);
                mCommands.push_back(cmd2);
                //cout << "Adding command with params: "<<cmd2<<endl;
            }else{
                mCommands.push_back(cmd);
                //cout << "Adding command: "<<cmd<<endl;
            }
            mCommandsPort.push_back(port);
        }
    }
}

void    ImitationApplicationThread::AddConnexion(SrcPortId src, DstPortId dst, bool bUnique){
    if(bUnique)
        RemAllSrcConnexions(dst);
    
    mCommandsType.push_back(1);
    mConnexionsSrcPort.push_back(src);
    mConnexionsDstPort.push_back(dst);
}
void    ImitationApplicationThread::RemConnexion(SrcPortId src, DstPortId dst){
    mCommandsType.push_back(2);
    mConnexionsSrcPort.push_back(src);
    mConnexionsDstPort.push_back(dst);
}
void    ImitationApplicationThread::RemAllSrcConnexions(DstPortId dst){
    for(int i=0;i<SPID_SIZE;i++)
        RemConnexion(SrcPortId(i),dst);
}
void    ImitationApplicationThread::RemAllDstConnexions(SrcPortId src){
    for(int i=0;i<DPID_SIZE;i++)
        RemConnexion(src,DstPortId(i));
}
void    ImitationApplicationThread::RemAllConnexions(){
    for(int i=0;i<SPID_SIZE;i++)
        for(int j=0;j<DPID_SIZE;j++)
            RemConnexion(SrcPortId(i),DstPortId(j));
}


void ImitationApplicationThread::SendCommands(){
    int cmdCnt = 0;
    int conCnt = 0;
    if(mCommandsType.size()>0){
        cout << "Sending commands..."<<endl;
    }
    for(size_t i=0;i<mCommandsType.size();i++){
        switch(mCommandsType[i]){
        case 0:
            {
#ifndef FAKE_NETWORK
                Bottle &cmd = mPorts[mCommandsPort[cmdCnt]]->prepare();
                cmd.fromString(mCommands[cmdCnt].c_str());
                mPorts[mCommandsPort[cmdCnt]]->writeStrict();
#endif
                cout << "  Sending: <"<<mCommands[cmdCnt]<<"> to: "<< mDstCtrlPortName[mCommandsPort[cmdCnt]] <<endl;
                cmdCnt++;
                break;
            }
        case 1:
            {
                cout << "  Connecting: "<<mSrcPortName[mConnexionsSrcPort[conCnt]]<<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;
#ifndef FAKE_NETWORK
                if(!Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                    if(!Network::connect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                        cerr << "Error: Unable to connect "<<mSrcPortName[mConnexionsSrcPort[conCnt]] <<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;                
#endif
                conCnt++;
            }
            break;
        case 2:
            {
#ifndef FAKE_NETWORK
                if(Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                    Network::disconnect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]);
#endif
                conCnt++;
            }
            break;
        }        
    }
    if(mCommandsType.size()>0)
        cerr <<"Sending done..."<<endl;

}

int ImitationApplicationThread::respond(const Bottle& command, Bottle& reply){
    mMutex.wait();
    int res = 0;
    if(mMode == MODE_BASICCOMMAND){
        res = respondToBasicCommand(command, reply);
    }else{
        res = respondToStateMachine(command, reply);
    }
    mMutex.post();
    return res;
}

int ImitationApplicationThread::respondToStateMachine(const Bottle& command, Bottle& reply){
    int  cmdSize    = command.size();
    int  retVal     = 1;

    if(cmdSize<=0){
        retVal = -1;
    }else{
        switch(command.get(0).asVocab()) {
        case VOCAB3('b','c','m'):
            mMode = MODE_BASICCOMMAND;
            break;
        case VOCAB3('s','m','m'):
            mMode = MODE_STATEMACHINE;
            break;
        case VOCAB1('n'):
        case VOCAB4('n','e','x','t'):
            mStateSignal = SSIG_NEXT;
            break;
        case VOCAB1('p'):
        case VOCAB4('p','r','e','v'):
            mStateSignal = SSIG_PREV;
            break;
        case VOCAB1('o'):
        case VOCAB2('o','k'):
            mStateSignal = SSIG_OK;
            break;
        case VOCAB1('a'):
        case VOCAB4('a','b','r','t'):
            mStateSignal = SSIG_ABORT;
            break;
        default:
            retVal = -1;
            break;
        }
    }
    if(retVal>0){
        reply.addVocab(Vocab::encode("ack"));
        //cout << "*ACK*"<<endl;
    }else if (retVal == 0){
        reply.addVocab(Vocab::encode("fail"));
        //cout << "*FAIL*"<<endl;
    }else{
        //cout << "*BIG_FAIL*"<<endl;
    }
    
    return retVal;
}

int ImitationApplicationThread::respondToBasicCommand(const Bottle& command, Bottle& reply){

    int  cmdSize    = command.size();
    int  retVal     = 1;

    if(cmdSize<=0){
        retVal = -1;
    }else{
        switch(command.get(0).asVocab()) {
        case VOCAB3('b','c','m'):
            mMode = MODE_BASICCOMMAND;
            break;
        case VOCAB3('s','m','m'):
            mMode = MODE_STATEMACHINE;
            break;
        case VOCAB4('i','n','i','t'):
            mBasicCommand = BC_INIT;
            break;
        case VOCAB3('c','l','r'):
            mBasicCommand = BC_CLEAR;
            break;
        case VOCAB3('r','u','n'):
            mBasicCommand = BC_RUN;
            break;
        case VOCAB4('s','t','o','p'):
            mBasicCommand = BC_STOP;
            break;
        case VOCAB4('r','e','s','t'):
            mBasicCommand = BC_REST;
            break;
        case VOCAB2('d','o'):
            if(cmdSize>1){
                ConstString str = command.get(1).asString();
                      if(str == "3DR"){
                    mBasicCommand = BC_3DMOUSE_TO_RIGHTARM;
                }else if(str == "3DRN"){
                    mBasicCommand = BC_3DMOUSE_TO_RIGHTARM_NONE;
                }else if(str == "3DL"){
                    mBasicCommand = BC_3DMOUSE_TO_LEFTARM;
                }else if(str == "3DLN"){
                    mBasicCommand = BC_3DMOUSE_TO_LEFTARM_NONE;
                }else if(str == "3DN"){
                    mBasicCommand = BC_3DMOUSE_TO_NONE;
                }else if(str == "Touch"){
                    mBasicCommand = BC_TOUCHPAD_TO_RIGHTARM;
                }else if(str == "TouchN"){
                    mBasicCommand = BC_TOUCHPAD_TO_RIGHTARM_NONE;
                }else if(str == "SensR"){
                    mBasicCommand = BC_SENSORS_TO_RIGHTARM;
                }else if(str == "SensRN"){
                    mBasicCommand = BC_SENSORS_TO_RIGHTARM_NONE;
                }else if(str == "SensL"){
                    mBasicCommand = BC_SENSORS_TO_LEFTARM;
                }else if(str == "SensLN"){
                    mBasicCommand = BC_SENSORS_TO_LEFTARM_NONE;
                }else{
                    retVal = 0;
                }
            }else{
                retVal = 0;
            }
            break;
        case VOCAB3('t','r','k'):
            if(cmdSize>1){
                ConstString str = command.get(1).asString();
                      if(str == "None"){
                    mBasicCommand = BC_TRACK_NONE;
                }else if(str == "RArm"){
                    mBasicCommand = BC_TRACK_RIGHTARM;
                }else if(str == "LArm"){
                    mBasicCommand = BC_TRACK_LEFTARM;
                }else if(str == "Obj0"){
                    mBasicCommand = BC_TRACK_OBJECT0;
                }else if(str == "Obj1"){
                    mBasicCommand = BC_TRACK_OBJECT1;
                }else{
                    retVal = 0;
                }
            }else{
                retVal = 0;
            }
            break;
        case VOCAB4('h','a','n','d'):
            if(cmdSize>1){
                ConstString str = command.get(1).asString();
                      if(str == "ro"){
                    mBasicCommand = BC_HAND_OPENRIGHT;
                }else if(str == "rc"){
                    mBasicCommand = BC_HAND_CLOSERIGHT;
                }else if(str == "lo"){
                    mBasicCommand = BC_HAND_OPENLEFT;
                }else if(str == "lc"){
                    mBasicCommand = BC_HAND_CLOSELEFT;
                }else{
                    retVal = 0;
                }
            }else{
                retVal = 0;
            }
            break;
        case VOCAB4('e','y','e','t'):
            if(cmdSize>1){
                ConstString str = command.get(1).asString();
                      if(str == "None"){
                    mBasicCommand = BC_EYETARGET_TO_NONE;
                }else if(str == "RArm"){
                    mBasicCommand = BC_EYETARGET_TO_RIGHTARM;
                }else if(str == "LArm"){
                    mBasicCommand = BC_EYETARGET_TO_LEFTARM;
                }else{
                    retVal = 0;
                }
            }else{
                retVal = 0;
            }
            break;
        case VOCAB3('r','e','f'):
            if(cmdSize>1){
                ConstString str = command.get(1).asString();
                      if(str == "RightNone"){
                    mBasicCommand = BC_REF_RIGHT_NONE;
                }else if(str == "RightLock"){
                    mBasicCommand = BC_REF_RIGHT_LOCK;
                }else if(str == "RightUnlock"){
                    mBasicCommand = BC_REF_RIGHT_UNLOCK;
                }else if(str == "RightObj0"){
                    mBasicCommand = BC_REF_RIGHT_OBJ0;
                }else if(str == "RightObj1"){
                    mBasicCommand = BC_REF_RIGHT_OBJ1;
                }else if(str == "RightHand"){
                    mBasicCommand = BC_REF_RIGHT_LEFTHAND;
                }else if(str == "LeftNone"){
                    mBasicCommand = BC_REF_LEFT_NONE;
                }else if(str == "LeftLock"){
                    mBasicCommand = BC_REF_LEFT_LOCK;
                }else if(str == "LeftUnlock"){
                    mBasicCommand = BC_REF_LEFT_UNLOCK;
                }else if(str == "LeftObj0"){
                    mBasicCommand = BC_REF_LEFT_OBJ0;
                }else if(str == "LeftObj1"){
                    mBasicCommand = BC_REF_LEFT_OBJ1;
                }else if(str == "LeftHand"){
                    mBasicCommand = BC_REF_LEFT_RIGHTHAND;
                }else{
                    retVal = 0;
                }
            }else{
                retVal = 0;
            }
            break;
        case VOCAB3('g','m','m'):
            if(cmdSize>1){
                ConstString str = command.get(1).asString();
                      if(str == "LearnRight"){
                    mBasicCommand = BC_GMM_LEARN_RIGHT;
                }else if(str == "LearnCorrRight"){
                    mBasicCommand = BC_GMM_LEARN_CORR_RIGHT;
                }else if(str == "DemoRight"){
                    mBasicCommand = BC_GMM_REC_RIGHT_START;
                    if(cmdSize>2) mBasicCommandParams = command.get(2).toString();
                }else if(str == "CorrRight"){
                    mBasicCommand = BC_GMM_CORR_RIGHT_START;
                    if(cmdSize>2) mBasicCommandParams = command.get(2).toString();
                }else if(str == "StopRight"){
                    mBasicCommand = BC_GMM_RIGHT_STOP;
                }else if(str == "ReproRight"){
                    mBasicCommand = BC_GMM_REPRO_RIGHT_START;
                }else if(str == "LoadRight"){
                    if(cmdSize>2){
                        mBasicCommand = BC_GMM_LOAD_RIGHT;
                        mBasicCommandParams = command.get(2).toString();
                    }else retVal = 0;
                }else if(str == "DemoNameRight"){
                    if(cmdSize>2){
                        mBasicCommand = BC_GMM_DEMONAME_RIGHT;
                        mBasicCommandParams = command.get(2).toString();
                    }else retVal = 0;
                }else if(str == "CorrNameRight"){
                    if(cmdSize>2){
                        mBasicCommand = BC_GMM_CORRNAME_RIGHT;
                        mBasicCommandParams = command.get(2).toString();
                    }else retVal = 0;
                }else if(str == "LearnLeft"){
                    mBasicCommand = BC_GMM_LEARN_LEFT;
                }else if(str == "LearnCorrRight"){
                    mBasicCommand = BC_GMM_LEARN_CORR_LEFT;
                }else if(str == "DemoLeft"){
                    mBasicCommand = BC_GMM_REC_LEFT_START;
                    if(cmdSize>2) mBasicCommandParams = command.get(2).toString();
                }else if(str == "CorrLeft"){
                    mBasicCommand = BC_GMM_CORR_LEFT_START;
                    if(cmdSize>2) mBasicCommandParams = command.get(2).toString();
                }else if(str == "StopLeft"){
                    mBasicCommand = BC_GMM_LEFT_STOP;
                }else if(str == "ReproLeft"){
                    mBasicCommand = BC_GMM_REPRO_LEFT_START;
                }else if(str == "LoadLeft"){
                    if(cmdSize>2){
                        mBasicCommand = BC_GMM_LOAD_LEFT;
                        mBasicCommandParams = command.get(2).toString();
                    }else retVal = 0;
                }else if(str == "DemoNameLeft"){
                    if(cmdSize>2){
                        mBasicCommand = BC_GMM_DEMONAME_LEFT;
                        mBasicCommandParams = command.get(2).toString();
                    }else retVal = 0;
                }else if(str == "CorrNameLeft"){
                    if(cmdSize>2){
                        mBasicCommand = BC_GMM_CORRNAME_LEFT;
                        mBasicCommandParams = command.get(2).toString();
                    }else retVal = 0;
                }else{
                    
                    retVal = 0;
                }
            }else{
                retVal = 0;
            }
            break;
        default:
            retVal = -1;
            break;
        }
    }
    if(retVal>0){
        reply.addVocab(Vocab::encode("ack"));
        cout << "*ACK*"<<endl;
    }else if (retVal == 0){
        reply.addVocab(Vocab::encode("fail"));
        cout << "*FAIL*"<<endl;
    }else{
        cout << "*BIG_FAIL*"<<endl;
    }
    
    return retVal;
}

void ImitationApplicationThread::SendBasicCommand(const char* cmd){
    if(cmd!=NULL){
        Bottle bottleCmd;
        bottleCmd.fromString(cmd);
        Bottle rep;
        respondToBasicCommand(cmd,rep);
        ProcessBasicCommand();
    }
}

void ImitationApplicationThread::InitStateMachine(){
    for(int i=0;i<IAS_SIZE;i++)
        mStateName[i][0] = 0;

    snprintf(mStateName[IAS_NONE],                  256,"NONE");
    snprintf(mStateName[IAS_IDLE],                  256,"IDLE");
    snprintf(mStateName[IAS_INIT],                  256,"INIT");
    snprintf(mStateName[IAS_RUN],                   256,"RUN");
    snprintf(mStateName[IAS_PAUSE],                 256,"PAUSE");
    snprintf(mStateName[IAS_DELAY],                 256,"DELAY");
    snprintf(mStateName[IAS_STOP],                  256,"STOP");
    

    snprintf(mStateName[IAS_DEVTPAD],               256,"MENU TouchPad");
    snprintf(mStateName[IAS_DEVTPAD_INIT],          256,"TouchPad Init");
    snprintf(mStateName[IAS_DEVTPAD_RUN],           256,"TouchPad Run");
    snprintf(mStateName[IAS_DEVTPAD_STOP],          256,"TouchPad Stop");

    snprintf(mStateName[IAS_DEV3DM],                256,"MENU 3DMouse");
    snprintf(mStateName[IAS_DEV3DM_INIT],           256,"3DMouse Init");
    snprintf(mStateName[IAS_DEV3DM_RUN],            256,"3DMouse Run");
    snprintf(mStateName[IAS_DEV3DM_STOP],           256,"3DMouse Stop");

    snprintf(mStateName[IAS_DEVSENSORS],            256,"MENU Dev MotionSensors");
    snprintf(mStateName[IAS_DEVSENSORS_INIT],       256,"MotionSensors Init");
    snprintf(mStateName[IAS_DEVSENSORS_READY],      256,"MotionSensors Ready");
    snprintf(mStateName[IAS_DEVSENSORS_RUN],        256,"MotionSensors Run");
    snprintf(mStateName[IAS_DEVSENSORS_STOP],       256,"MotionSensors Stop");

    snprintf(mStateName[IAS_DEMO],                  256,"MENU Demo");
    snprintf(mStateName[IAS_DEMO_INIT],             256,"Demo Init");
    snprintf(mStateName[IAS_DEMO_RUN],              256,"Demo Run");
    snprintf(mStateName[IAS_DEMO_STOP],             256,"Demo Stop");

    snprintf(mStateName[IAS_TEST],                  256,"MENU Test");


    snprintf(mStateName[IAS_DEMO_LEARNING_SENSORS],     256,"SUB MENU Sensors Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_3DM],         256,"SUB MENU 3DM Learning");
    snprintf(mStateName[IAS_DEMO_REPROONE],             256,"SUB MENU Repro *ONE*");
    snprintf(mStateName[IAS_DEMO_REFINE],               256,"SUB MENU Refine");
    snprintf(mStateName[IAS_DEMO_REPROTWO],             256,"SUB MENU Repro *TWO*");
    snprintf(mStateName[IAS_DEMO_REUSE],                256,"SUB MENU Reuse");
    snprintf(mStateName[IAS_DEMO_REPROTHREE],           256,"SUB MENU Repro *THREE*");
    snprintf(mStateName[IAS_DEMO_FINAL],                256,"SUB MENU *FINAL*");



    snprintf(mStateName[IAS_DEMO_LEARNING_SENSORS_INIT],     256,"SUB INIT Sensors Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_SENSORS_STOP],     256,"SUB STOP Sensors Learning");

    snprintf(mStateName[IAS_DEMO_LEARNING_3DM_INIT],         256,"SUB INIT 3DM Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_3DM_STOP],         256,"SUB STOP 3DM Learning");

    snprintf(mStateName[IAS_DEMO_REPROONE_INIT],             256,"SUB INIT Repro *ONE*");
    snprintf(mStateName[IAS_DEMO_REPROONE_STOP],             256,"SUB STOP Repro *ONE*");

    snprintf(mStateName[IAS_DEMO_REFINE_INIT],               256,"SUB INIT Refine");
    snprintf(mStateName[IAS_DEMO_REFINE_STOP],               256,"SUB STOP Refine");

    snprintf(mStateName[IAS_DEMO_REPROTWO_INIT],             256,"SUB INIT Repro *TWO*");
    snprintf(mStateName[IAS_DEMO_REPROTWO_STOP],             256,"SUB STOP Repro *TWO*");

    snprintf(mStateName[IAS_DEMO_REUSE_INIT],                256,"SUB INIT Reuse");
    snprintf(mStateName[IAS_DEMO_REUSE_STOP],                256,"SUB STOP Reuse");

    snprintf(mStateName[IAS_DEMO_REPROTHREE_INIT],           256,"SUB INIT Repro *THREE*");
    snprintf(mStateName[IAS_DEMO_REPROTHREE_STOP],           256,"SUB STOP Repro *THREE*");

    snprintf(mStateName[IAS_DEMO_FINAL_INIT],                256,"SUB INIT *FINAL*");
    snprintf(mStateName[IAS_DEMO_FINAL_STOP],                256,"SUB STOP *FINAL*");
        


    snprintf(mStateName[IAS_DEMO_LEARNING],              256,"Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_INIT],         256,"INIT  Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_OBJTRK],       256,"TRACK Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_OBJLOCK],      256,"LOCK  Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_STARTSENSORS], 256,"SENS  Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_STARTDEMO],    256,"DEMO  Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_ACCEPTDEMO],   256,"ACK   Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_NEXTDEMO],     256,"NEXT  Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_LEARN],        256,"LEARN Learning");
    snprintf(mStateName[IAS_DEMO_LEARNING_STOP],         256,"STOP  Learning");

    snprintf(mStateName[IAS_DEMO_REFINEREUSE],              256,"RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_INIT],         256,"INIT  RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_OBJTRK],       256,"TRACK RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_OBJLOCK],      256,"LOCK  RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_STARTCORR],    256,"CORR  RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_ACCEPTCORR],   256,"ACK   RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_NEXTDEMO],     256,"NEXT  RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_LEARN],        256,"LEARN RefineReuse");
    snprintf(mStateName[IAS_DEMO_REFINEREUSE_STOP],         256,"STOP  RefineReuse");

    snprintf(mStateName[IAS_REPRO],             256,"Repro");
    snprintf(mStateName[IAS_REPRO_INIT],        256,"INIT  Repro");
    snprintf(mStateName[IAS_REPRO_OBJTRK],      256,"TRACK Repro");
    snprintf(mStateName[IAS_REPRO_OBJLOCK],     256,"LOCK  Repro");
    snprintf(mStateName[IAS_REPRO_STARTREPRO],  256,"STAR  Repro");
    snprintf(mStateName[IAS_REPRO_STOPREPRO],   256,"STOP  Repro");
    snprintf(mStateName[IAS_REPRO_STOP],        256,"QUIT  Repro");


    mState = mPrevState = mNextState = mDelayedState = IAS_NONE;
    mStateSignal = SSIG_NONE;
    mDelayedStateTime   = 0.0;

    cout << "State names: "<<endl;
    cout << "*******************"<<endl;
    for(int i=0;i<IAS_SIZE;i++)
        cout <<"  "<<i<<":" <<mStateName[i]<<endl;
    cout << "*******************"<<endl;
}
void ImitationApplicationThread::DelayNextState(State nextState, double delay){
    mNextState          = IAS_DELAY;
    mDelayedState       = nextState;
    mDelayedStateTime   = Time::now() + delay;
}
void ImitationApplicationThread::ProcessStateMachine(){
    //if(mStateSignal!=SSIG_NONE)
    //    cout << "Recieved Signal: "<<mStateSignal<<endl;

    char txt[256];

    if(mNextState!=mState)
        mState = mNextState;
    
    bool bStateChanged = (mPrevState != mState);
    if(bStateChanged){
        cout << "************************************************************************************************"<<endl;
        cout << "* New State:  <"<<mStateName[mState]<<">("<<mState<<")                      (from <"<<mStateName[mPrevState]<<">("<<mPrevState<<"))"<<endl; 
        cout << "************************************************************************************************"<<endl;
    }
    
    mNextState = mState;
    switch(mState){
    case IAS_NONE:
        mNextState = IAS_IDLE;
        break;
    case IAS_IDLE:
        if(bStateChanged){
            SendBasicCommand("stop");
            SendBasicCommand("clr");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_INIT;
        break;
    case IAS_INIT:
        if(bStateChanged){
            SendBasicCommand("init");
            SendBasicCommand("run");
        }
        mNextState = IAS_DEVTPAD;
        break;
    case IAS_RUN:
        break;
    case IAS_PAUSE:
        break;
    case IAS_DELAY:
        if(Time::now() > mDelayedStateTime)
            mNextState = mDelayedState;
        break;
    case IAS_STOP:
        if(bStateChanged){
            SendBasicCommand("rest");
        }
        DelayNextState(IAS_IDLE,5.0);
        break;


//---------------------------------------------------------

    case IAS_DEVTPAD:
             if(mStateSignal == SSIG_OK)    mNextState = IAS_DEVTPAD_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEV3DM;
        else if(mStateSignal == SSIG_PREV)  mNextState = IAS_TEST;
        break;
    case IAS_DEV3DM:
             if(mStateSignal == SSIG_OK)    mNextState = IAS_DEV3DM_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEVSENSORS;
        else if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEVTPAD;
        break;
    case IAS_DEVSENSORS:
             if(mStateSignal == SSIG_OK)    mNextState = IAS_DEVSENSORS_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO;
        else if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEV3DM;
        break;
    case IAS_DEMO:
             if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_TEST;
        else if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEVSENSORS;
        break;
    case IAS_TEST:
             if(mStateSignal == SSIG_OK)    mNextState = IAS_TEST;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEVTPAD;
        else if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO;
        break;


//---------------------------------------------------------

    case IAS_DEVTPAD_INIT:
        SendBasicCommand("run");
        SendBasicCommand("do Touch");
        mNextState = IAS_DEVTPAD_RUN;
        break;
    case IAS_DEVTPAD_RUN:
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEVTPAD_STOP;
        break;
    case IAS_DEVTPAD_STOP:
        SendBasicCommand("do TouchN");
        mNextState = IAS_DEVTPAD;
        break;

//---------------------------------------------------------

    case IAS_DEV3DM_INIT:
        SendBasicCommand("run");
        SendBasicCommand("do 3DR");
        mNextState = IAS_DEV3DM_RUN;
        break;
    case IAS_DEV3DM_RUN:
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEV3DM_STOP;
        break;
    case IAS_DEV3DM_STOP:
        SendBasicCommand("do 3DRN");
        mNextState = IAS_DEV3DM;
        break;

//---------------------------------------------------------

    case IAS_DEVSENSORS_INIT:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Get the senors ready end press OK  *"<<endl;
            cout << "**************************************"<<endl;
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEVSENSORS_READY;
        break;
    case IAS_DEVSENSORS_READY:
        SendBasicCommand("run");
        SendBasicCommand("do SensR");
        mNextState = IAS_DEVSENSORS_RUN;
        break;
    case IAS_DEVSENSORS_RUN:
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEVSENSORS_STOP;
        break;
    case IAS_DEVSENSORS_STOP:
        SendBasicCommand("do SensRN");
        mNextState = IAS_DEVSENSORS;
        break;


//---------------------------------------------------------
    case IAS_DEMO_INIT:
        mNextState = IAS_DEMO_LEARNING_SENSORS;
        break;
    case IAS_DEMO_LEARNING_SENSORS:
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_FINAL;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_SENSORS_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_LEARNING_3DM;
        break;
    case IAS_DEMO_LEARNING_3DM:
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_LEARNING_SENSORS;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_3DM_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_REPROONE;
        break;
    case IAS_DEMO_REPROONE:
        if(bStateChanged){
            SendBasicCommand("hand ro");
            SendBasicCommand("hand lo");
        }
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_LEARNING_3DM;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REPROONE_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_REFINE;
        break;
    case IAS_DEMO_REFINE:
        if(bStateChanged){
            SendBasicCommand("hand ro");
            SendBasicCommand("hand lo");
        }
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_REPROONE;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REFINE_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_REPROTWO;
        break;
    case IAS_DEMO_REPROTWO:
        if(bStateChanged){
            SendBasicCommand("hand ro");
            SendBasicCommand("hand lo");
        }
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_REFINE;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REPROTWO_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_REUSE;
        break;
    case IAS_DEMO_REUSE:
        if(bStateChanged){
            SendBasicCommand("hand ro");
            SendBasicCommand("hand lo");
        }
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_REPROTWO;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REUSE_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_REPROTHREE;
        break;
    case IAS_DEMO_REPROTHREE:
        if(bStateChanged){
            SendBasicCommand("hand ro");
            SendBasicCommand("hand lo");
        }
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_REUSE;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REPROTHREE_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_FINAL;
        break;
    case IAS_DEMO_FINAL:
        if(bStateChanged){
            SendBasicCommand("hand ro");
            SendBasicCommand("hand lo");
        }
             if(mStateSignal == SSIG_PREV)  mNextState = IAS_DEMO_REPROTHREE;
        else if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_FINAL_INIT;
        else if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_STOP;
        else if(mStateSignal == SSIG_NEXT)  mNextState = IAS_DEMO_LEARNING_SENSORS;
        break;
    case IAS_DEMO_STOP:
        mNextState = IAS_DEMO;
        break;
    
//---------------------------------------------------------

    case IAS_DEMO_LEARNING:
        mNextState = IAS_DEMO_LEARNING_INIT;
        break;
    case IAS_DEMO_LEARNING_INIT:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Starting Learning Demo              *"<<endl;
            cout << "**************************************"<<endl;
            snprintf(txt,256,"gmm DemoNameRight %s/demo",mLearningDemoName);
            SendBasicCommand(txt);
            snprintf(txt,256,"gmm CorrNameRight %s/corr",mLearningCorrName);
            SendBasicCommand(txt);
        }
        mCurrDemoId = 0;

        mNextState = IAS_DEMO_LEARNING_OBJTRK;
        break;
    case IAS_DEMO_LEARNING_OBJTRK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Place, track object and press OK   *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("run");
            SendBasicCommand("trk Obj0");
            SendBasicCommand("ref RightUnlock");
            SendBasicCommand("ref RightObj0");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_OBJLOCK;
        break;
    case IAS_DEMO_LEARNING_OBJLOCK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Get the senors ready and press OK  *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("ref RightLock");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_STARTSENSORS;
        break;
    case IAS_DEMO_LEARNING_STARTSENSORS:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to start demo: "<< mCurrDemoId <<"          *"<<endl;
            cout << "**************************************"<<endl;
            if(mRecSensorType==0) SendBasicCommand("do 3DR");
            else                  SendBasicCommand("do SensR");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_STARTDEMO;
        break;
    case IAS_DEMO_LEARNING_STARTDEMO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to stop demo              *"<<endl;
            cout << "**************************************"<<endl;
            snprintf(txt,256,"gmm DemoRight %d",mCurrDemoId);
            SendBasicCommand(txt);
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_ACCEPTDEMO;
        break;
    case IAS_DEMO_LEARNING_ACCEPTDEMO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* OK to accept demo, ABORT to not    *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm StopRight");
        }
        if(mStateSignal == SSIG_OK)    mCurrDemoId++;
        if((mStateSignal == SSIG_OK)||
          (mStateSignal == SSIG_ABORT))mNextState = IAS_DEMO_LEARNING_NEXTDEMO;
        break;
    case IAS_DEMO_LEARNING_NEXTDEMO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* OK for next demo, ABORT to learn   *"<<endl;
            cout << "**************************************"<<endl;
            if(mRecSensorType==0) SendBasicCommand("do 3DRN");
            else                  SendBasicCommand("do SensRN");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_OBJTRK;
        if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_LEARNING_LEARN;
        break;
    case IAS_DEMO_LEARNING_LEARN:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Learning, OK to end                *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm LearnRight");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_STOP;
        break;
    case IAS_DEMO_LEARNING_STOP:
        //SendBasicCommand("rest");
        mNextState = mGeneralOutState;;
        break;

        
/*
    case IAS_DEMO_LEARNING_INIT:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Starting Sensors Demo              *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm DemoNameLeft ./demo");
        }
        mCurrDemoId = 0;

        mNextState = IAS_DEMO_LEARNING_OBJTRK;
        break;
    case IAS_DEMO_LEARNING_OBJTRK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Place, track object and press OK   *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("run");
            SendBasicCommand("trk RArm");
            SendBasicCommand("ref LeftUnlock");
            SendBasicCommand("ref LeftHand");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_OBJLOCK;
        break;
    case IAS_DEMO_LEARNING_OBJLOCK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Get the senors ready and press OK  *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("ref LeftLock");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_STARTSENSORS;
        break;
    case IAS_DEMO_LEARNING_STARTSENSORS:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to start demo: "<< mCurrDemoId <<"          *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("do 3DL");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_STARTDEMO;
        break;
    case IAS_DEMO_LEARNING_STARTDEMO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to stop demo              *"<<endl;
            cout << "**************************************"<<endl;
            snprintf(txt,256,"gmm DemoLeft %d",mCurrDemoId);
            SendBasicCommand(txt);
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_ACCEPTDEMO;
        break;
    case IAS_DEMO_LEARNING_ACCEPTDEMO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* OK to accept demo, ABORT to not    *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm StopLeft");
        }
        if(mStateSignal == SSIG_OK)    mCurrDemoId++;
        if((mStateSignal == SSIG_OK)||
          (mStateSignal == SSIG_ABORT))mNextState = IAS_DEMO_LEARNING_NEXTDEMO;
        break;
    case IAS_DEMO_LEARNING_NEXTDEMO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* OK for next demo, ABORT to learn   *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("do 3DLN");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_OBJTRK;
        if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_LEARNING_LEARN;
        break;
    case IAS_DEMO_LEARNING_LEARN:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Learning, OK to end                *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm LearnLeft");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_LEARNING_STOP;
        break;
    case IAS_DEMO_LEARNING_STOP:
        //SendBasicCommand("rest");
        mNextState = IAS_DEMO_LEARNING;
        break;
*/
    
    
    
    
    
//---------------------------------------------------------
/*
    case IAS_DEMO_REPROONE_INIT:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Starting ReproONE Demo              *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm DemoNameRight ./demo");
            SendBasicCommand("gmm CorrNameRight ./corr");
            SendBasicCommand("gmm LoadRight ./corr");
        }

        mNextState = IAS_DEMO_REPROONE_OBJTRK;
        break;
    case IAS_DEMO_REPROONE_OBJTRK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Place, track object and press OK   *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("run");
            SendBasicCommand("trk Obj0");
            SendBasicCommand("ref RightUnlock");
            SendBasicCommand("ref RightObj0");
            SendBasicCommand("do 3DR");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REPROONE_OBJLOCK;
        break;
    case IAS_DEMO_REPROONE_OBJLOCK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to START reproduction     *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("ref RightLock");
            //SendBasicCommand("trk None");
            //AddCommand(PID_Robot,   "iku Eye");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REPROONE_STARTREPRO;
        break;
    case IAS_DEMO_REPROONE_STARTREPRO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to STOP reproduction      *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm ReproRight");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REPROONE_STOPREPRO;
        break;
    case IAS_DEMO_REPROONE_STOPREPRO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* OK to do it again, ABORT to not    *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm StopRight");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REPROONE_OBJTRK;
        if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_REPROONE_STOP;
        break;
    case IAS_DEMO_REPROONE_STOP:
        SendBasicCommand("do 3DRN");
        SendBasicCommand("rest");
        mNextState = IAS_DEMO_REPROONE;
        break;
*/
    
//---------------------------------------------------------

    case IAS_DEMO_REFINEREUSE:
        mNextState = IAS_DEMO_REFINEREUSE_INIT;
        break;
    case IAS_DEMO_REFINEREUSE_INIT:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Starting Refinement Demo           *"<<endl;
            cout << "**************************************"<<endl;
            snprintf(txt,256,"gmm DemoNameRight %s/demo",mLearningDemoName);
            SendBasicCommand(txt);
            snprintf(txt,256,"gmm LoadRight %s/demo",mLearningDemoName);
            SendBasicCommand(txt);
            snprintf(txt,256,"gmm CorrNameRight %s/corr",mLearningCorrName);
            SendBasicCommand(txt);
            //SendBasicCommand("gmm DemoNameRight ./demo");
            //SendBasicCommand("gmm LoadRight     ./demo");
            //SendBasicCommand("gmm CorrNameRight ./corr");
            mCurrDemoId = 0;
        }

        mNextState = IAS_DEMO_REFINEREUSE_OBJTRK;
        break;
    case IAS_DEMO_REFINEREUSE_OBJTRK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Place, track object and press OK   *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("run");
            SendBasicCommand("trk Obj0");
            SendBasicCommand("ref RightUnlock");
            SendBasicCommand("ref RightObj0");
            SendBasicCommand("do Touch");
            //SendBasicCommand("do 3DR");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REFINEREUSE_OBJLOCK;
        break;
    case IAS_DEMO_REFINEREUSE_OBJLOCK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to start CORRECTION: "<< mCurrDemoId <<"    *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("ref RightLock");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REFINEREUSE_STARTCORR;
        break;
    case IAS_DEMO_REFINEREUSE_STARTCORR:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to STOP CORRECTION        *"<<endl;
            cout << "**************************************"<<endl;
            //SendBasicCommand("do 3DRN");
            SendBasicCommand("do Touch");
            snprintf(txt,256,"gmm CorrRight %d",mCurrDemoId);
            SendBasicCommand(txt);
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REFINEREUSE_ACCEPTCORR;
        break;
    case IAS_DEMO_REFINEREUSE_ACCEPTCORR:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* OK to accept corr, ABORT to not    *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm StopRight");
        }
        if(mStateSignal == SSIG_OK)    mCurrDemoId++;
        if((mStateSignal == SSIG_OK)||
          (mStateSignal == SSIG_ABORT))mNextState = IAS_DEMO_REFINEREUSE_NEXTDEMO;
        break;
    case IAS_DEMO_REFINEREUSE_NEXTDEMO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* OK for next corr, ABORT to learn   *"<<endl;
            cout << "**************************************"<<endl;
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REFINEREUSE_OBJTRK;
        if(mStateSignal == SSIG_ABORT) mNextState = IAS_DEMO_REFINEREUSE_LEARN;
        break;
    case IAS_DEMO_REFINEREUSE_LEARN:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Learning, OK to end                *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("gmm LearnCorrRight");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_REFINEREUSE_STOP;
        break;
    case IAS_DEMO_REFINEREUSE_STOP:
        //SendBasicCommand("rest");
        SendBasicCommand("do TouchN");
        //SendBasicCommand("do 3DRN");
        mNextState = mGeneralOutState;
        break;
//---------------------------------------------------------

    
//---------------------------------------------------------
    case IAS_DEMO_LEARNING_SENSORS_INIT:
        snprintf(mLearningDemoName,256,"./GraspBall");
        snprintf(mLearningCorrName,256,"./GraspBall");
        mRecSensorType      = 1;
        mGeneralOutState    = IAS_DEMO_LEARNING_SENSORS_STOP;

        mNextState          = IAS_DEMO_LEARNING;
        break;
    case IAS_DEMO_LEARNING_SENSORS_STOP:
        mNextState = IAS_DEMO_LEARNING_SENSORS;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
    case IAS_DEMO_LEARNING_3DM_INIT:
        snprintf(mLearningDemoName,256,"./GraspBall");
        snprintf(mLearningCorrName,256,"./GraspBall");
        mRecSensorType      = 0;
        mGeneralOutState    = IAS_DEMO_LEARNING_3DM_STOP;

        mNextState          = IAS_DEMO_LEARNING;
        break;
    case IAS_DEMO_LEARNING_3DM_STOP:
        mNextState = IAS_DEMO_LEARNING_3DM;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
    case IAS_DEMO_REFINE_INIT:
        snprintf(mLearningDemoName,256,"./GraspBall");
        snprintf(mLearningCorrName,256,"./GraspBall");
        mRecSensorType      = 1;
        mGeneralOutState    = IAS_DEMO_REFINE_STOP;

        mNextState          = IAS_DEMO_REFINEREUSE;
        break;
    case IAS_DEMO_REFINE_STOP:
        mNextState = IAS_DEMO_REFINE;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
    case IAS_DEMO_REUSE_INIT:
        snprintf(mLearningDemoName,256,"./GraspBall");
        snprintf(mLearningCorrName,256,"./GraspBottle");
        mRecSensorType      = 1;
        mGeneralOutState    = IAS_DEMO_REUSE_STOP;

        mNextState          = IAS_DEMO_REFINEREUSE;
        break;
    case IAS_DEMO_REUSE_STOP:
        mNextState = IAS_DEMO_REUSE;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
    case IAS_DEMO_REPROONE_INIT:
        if(bStateChanged){
            InitReproStateMachine(  IAS_DEMO_REPROONE_INIT,
                                    IAS_DEMO_REPROONE_STOP,
                                    "./GraspBall",
                                    true,       // bRightHand
                                    true,       // bUseDemo or corr (false)
                                    0,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    true,       // loop
                                    false,      // skipObjTrk
                                    false,      // inHandState
                                    true,       // outHandState
                                    true,       // lockRef
                                    true,       // stopGMM
                                    true,      // use3DM
                                    6.0);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_REPROONE_STOP:
        mNextState = IAS_DEMO_REPROONE;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
    case IAS_DEMO_REPROTWO_INIT:
        if(bStateChanged){
            InitReproStateMachine(  IAS_DEMO_REPROTWO_INIT,
                                    IAS_DEMO_REPROTWO_STOP,
                                    "./GraspBall",
                                    true,       // bRightHand
                                    false,      // bUseDemo or corr (false)
                                    0,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    true,       // loop
                                    true,       // skipObjTrk
                                    false,      // inHandState
                                    true,       // outHandState
                                    true,       // lockRef
                                    true,       // stopGMM
                                    true,      // use3DM
                                    6.0);

        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_REPROTWO_STOP:
        mNextState = IAS_DEMO_REPROTWO;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
    case IAS_DEMO_REPROTHREE_INIT:
        if(bStateChanged){
            InitReproStateMachine(  IAS_DEMO_REPROTHREE_INIT,
                                    IAS_DEMO_REPROTHREE_STOP,
                                    "./GraspBottle",
                                    true,       // bRightHand
                                    false,      // bUseDemo or corr (false)
                                    0,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    true,       // loop
                                    false,      // skipObjTrk
                                    false,      // inHandState
                                    true,       // outHandState
                                    true,       // lockRef
                                    true,       // stopGMM
                                    true,      // use3DM
                                    6.0);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_REPROTHREE_STOP:
        mNextState = IAS_DEMO_REPROTHREE;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
    case IAS_DEMO_FINAL_INIT:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* FINAL DEMO                         *"<<endl;
            cout << "**************************************"<<endl;
            SendBasicCommand("rest");
        }
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_FINAL_REPRO_1;
        break;
    case IAS_DEMO_FINAL_REPRO_1:
        if(bStateChanged){
            InitReproStateMachine(  IAS_DEMO_FINAL_REPRO_1,
                                    IAS_DEMO_FINAL_REPRO_2_INIT,
                                    "./GraspSponge",
                                    false,      // bRightHand
                                    true,       // bUseDemo or corr (false)
                                    1,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    false,      // loop
                                    false,      // skipObjTrk
                                    false,      // inHandState
                                    true,       // outHandState
                                    true,       // lockRef
                                    true,       // stopGMM
                                    true,      // use3DM
                                    6.0);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_FINAL_REPRO_2_INIT:
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_FINAL_REPRO_2;
        break;
    case IAS_DEMO_FINAL_REPRO_2:
        if(bStateChanged){
            InitReproStateMachine(  IAS_DEMO_FINAL_REPRO_2,
                                    IAS_DEMO_FINAL_REPRO_3_INIT,
                                    "./GraspSpongeReady",
                                    false,      // bRightHand
                                    true,       // bUseDemo or corr (false)
                                    -1,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    false,      // loop
                                    false,      // skipObjTrk
                                    true,      // inHandState
                                    true,       // outHandState
                                    true,       // lockRef
                                    true,       // stopGMM
                                    true,      // use3DM
                                    6.0);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_FINAL_REPRO_3_INIT:
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_FINAL_REPRO_3;
        break;
    case IAS_DEMO_FINAL_REPRO_3:
        if(bStateChanged){
            InitReproStateMachine(  IAS_DEMO_FINAL_REPRO_3,
                                    IAS_DEMO_FINAL_REPRO_5_INIT,
                                    "./GraspBottle",
                                    true,       // bRightHand
                                    false,      // bUseDemo or corr (false)
                                    0,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    false,      // loop
                                    false,      // skipObjTrk
                                    false,      // inHandState
                                    true,       // outHandState
                                    true,       // lockRef
                                    true,       // stopGMM
                                    true,      // use3DM
                                    6.0);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_FINAL_REPRO_4_INIT:
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_FINAL_REPRO_4;
        break;
    case IAS_DEMO_FINAL_REPRO_4:
        if(bStateChanged){
            InitReproStateMachine(  IAS_DEMO_FINAL_REPRO_4,
                                    IAS_DEMO_FINAL_REPRO_5_INIT,
                                    "./PlaceBottle",
                                    true,      // bRightHand
                                    true,       // bUseDemo or corr (false)
                                    0,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    false,      // loop
                                    false,      // skipObjTrk
                                    true,      // inHandState
                                    true,       // outHandState
                                    true,       // lockRef
                                    false,       // stopGMM
                                    true,      // use3DM
                                    6.0);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_FINAL_REPRO_5_INIT:
        if(mStateSignal == SSIG_OK)    mNextState = IAS_DEMO_FINAL_REPRO_5;
        break;
    case IAS_DEMO_FINAL_REPRO_5:
        if(bStateChanged){
            SendBasicCommand("do 3DR");
            InitReproStateMachine(  IAS_DEMO_FINAL_REPRO_5,
                                    IAS_DEMO_FINAL_STOP,
                                    "./CleanBottle",
                                    false,      // bRightHand
                                    true,       // bUseDemo or corr (false)
                                    2,          // objId (0,1: obj, -1: world, 2: opp hand)
                                    true,       // loop
                                    false,      // skipObjTrk
                                    true,      // inHandState
                                    true,       // outHandState
                                    true,       // stopGMM
                                    false,       // lockRef
                                    false,      // use3DM
                                    10.0);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_FINAL_STOP:
        mNextState = IAS_DEMO_FINAL;
        break;
//---------------------------------------------------------

//---------------------------------------------------------
/*
    case IAS_DEMO_REPROTWO_INIT:
        if(bStateChanged){
            InitReproStateMachine(IAS_DEMO_REPROTWO_INIT,IAS_DEMO_REPROTWO_STOP,"./CleanBottle",false,true,2,true,false,true,true,true,false,true);
        }
        ProcessReproStateMachine();
        break;
    case IAS_DEMO_REPROTWO_STOP:
        //SendBasicCommand("rest");
        mNextState = IAS_DEMO_REPROTWO;
        break;
*/
//---------------------------------------------------------
/*
    case IAS_DEVTPAD_INIT:
        break;
    case IAS_DEVTPAD_RUN:
        break;
    case IAS_DEVTPAD_STOP:
        break;
*/
//---------------------------------------------------------


    default:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* ERROR: NOT HANDLED STATE           *"<<endl;
            cout << "**************************************"<<endl;
        }
        break;
    }
    
    mPrevState  = mState;
    mState      = mNextState;

    
    mStateSignal = SSIG_NONE;
}

void ImitationApplicationThread::InitReproStateMachine( ImitationApplicationThread::State inState,
                                                        ImitationApplicationThread::State outState,
                                                        const char *reproName,
                                                        bool bRightHand,
                                                        bool bUseDemo,
                                                        int  objId,
                                                        bool loop,
                                                        bool skipObjTrk,
                                                        bool inHandState,
                                                        bool outHandState,
                                                        bool stopGMM,
                                                        bool lockRef,
                                                        bool use3DM,
double time
){
    mReproInState       = inState;
    mReproOutState      = outState;
    strncpy(mReproName,reproName,256);
    bReproRight         = bRightHand;
    bReproDemo          = bUseDemo;
    mReproObjId         = objId;
    bReproSkipObjTrk    = skipObjTrk;
    bReproLoop          = loop;
    bReproInHandState   = inHandState;
    bReproOutHandState  = outHandState;
    bReproStopGMM       = stopGMM;
    bReproLockRef       = lockRef;
    bReproUse3DM        = use3DM;
    mReproTime          = time;

    mReproPrevState = mReproNextState = mReproState = IAS_REPRO;
}

void ImitationApplicationThread::ProcessReproStateMachine(){
    char txt[256];


    mNextState = mReproInState;

    if(mReproNextState!=mReproState)
        mReproState = mReproNextState;
    
    bool bStateChanged = (mReproPrevState != mReproState);
    if(bStateChanged){
        cout << "************************************************************************************************"<<endl;
        cout << "* New State:  <"<<mStateName[mReproState]<<">("<<mReproState<<")                      (from <"<<mStateName[mReproPrevState]<<">("<<mReproPrevState<<"))"<<endl; 
        cout << "************************************************************************************************"<<endl;
    }
    
    mReproNextState = mReproState;
    switch(mReproState){
    case IAS_REPRO:
        mReproNextState = IAS_REPRO_INIT;
        break;
    case IAS_REPRO_INIT:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Starting Repro: "<< mReproName<< endl;
            cout << "**************************************"<<endl;
            //SendBasicCommand("gmm DemoNameRight ./demo");
            snprintf(txt,256,"gmm DemoName%s %s/demo",(bReproRight?"Right":"Left"),mReproName);
            SendBasicCommand(txt);
            //SendBasicCommand("gmm CorrNameRight ./corr");
            snprintf(txt,256,"gmm CorrName%s %s/corr",(bReproRight?"Right":"Left"),mReproName);
            SendBasicCommand(txt);
            //SendBasicCommand("gmm LoadRight ./corr");
            snprintf(txt,256,"gmm Load%s %s/%s",(bReproRight?"Right":"Left"),mReproName,(bReproDemo?"demo":"corr"));
            SendBasicCommand(txt);

        }

        mReproNextState = IAS_REPRO_OBJTRK;
        break;
    case IAS_REPRO_OBJTRK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Place, track object and press OK   *"<<endl;
            cout << "**************************************"<<endl;
        
            SendBasicCommand("run");
        
            snprintf(txt,256,"hand %s%s",(bReproRight?"r":"l"),(bReproInHandState?"c":"o"));
            SendBasicCommand(txt);

            if(!bReproSkipObjTrk){
                if((mReproObjId>=0)&&(mReproObjId<=1)){
                    //SendBasicCommand("trk Obj0");
                    snprintf(txt,256,"trk Obj%d",mReproObjId);
                    SendBasicCommand(txt);
                    //SendBasicCommand("ref RightObj0");
                    snprintf(txt,256,"ref %sObj0",(bReproRight?"Right":"Left"));
                    SendBasicCommand(txt);
                }else if(mReproObjId<0){
                    snprintf(txt,256,"trk %sArm",(bReproRight?"R":"L"));
                    SendBasicCommand(txt);
                    snprintf(txt,256,"ref %sNone",(bReproRight?"Right":"Left"));
                    SendBasicCommand(txt);
                }else{
                    snprintf(txt,256,"trk %sArm",(bReproRight?"L":"R"));
                    SendBasicCommand(txt);
                    snprintf(txt,256,"ref %sHand",(bReproRight?"Right":"Left"));
                    SendBasicCommand(txt);
                }
                //SendBasicCommand("ref RightUnlock");
                snprintf(txt,256,"ref %sUnlock",(bReproRight?"Right":"Left"));
                SendBasicCommand(txt);
            }
            if(bReproUse3DM){
                //SendBasicCommand("do 3DR");
                snprintf(txt,256,"do 3D%s",(bReproRight?"R":"L"));
                SendBasicCommand(txt);
            }
        }
        if(!bReproSkipObjTrk){
            if(mStateSignal == SSIG_OK)    mReproNextState = IAS_REPRO_OBJLOCK;
        }else{
            mReproNextState = IAS_REPRO_OBJLOCK;
        }
        break;
    case IAS_REPRO_OBJLOCK:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to START reproduction     *"<<endl;
            cout << "**************************************"<<endl;
            if(!bReproSkipObjTrk){
                //SendBasicCommand("ref RightLock");
                if(bReproLockRef){
                    snprintf(txt,256,"ref %sLock",(bReproRight?"Right":"Left"));
                    SendBasicCommand(txt);
                }
            }
        }
        if(mStateSignal == SSIG_OK)    mReproNextState = IAS_REPRO_STARTREPRO;
        break;
    case IAS_REPRO_STARTREPRO:
        if(bStateChanged){
            cout << "**************************************"<<endl;
            cout << "* Press OK to STOP reproduction      *"<<endl;
            cout << "**************************************"<<endl;
            //SendBasicCommand("gmm ReproRight");
            {PortId pid = (bReproRight?PID_GMMRight:PID_GMMLeft);
            snprintf(txt,256,"set reproTime %f",mReproTime);
                AddCommand(pid,txt);
            }
            snprintf(txt,256,"gmm Repro%s",(bReproRight?"Right":"Left"));
            SendBasicCommand(txt);
        }
        if(mStateSignal == SSIG_OK)    mReproNextState = IAS_REPRO_STOPREPRO;
        break;
    case IAS_REPRO_STOPREPRO:
        if(bStateChanged){
            if(bReproLoop){
                cout << "**************************************"<<endl;
                cout << "* OK to do it again, ABORT to not    *"<<endl;
                cout << "**************************************"<<endl;
            }
            //SendBasicCommand("gmm StopRight");
            if(bReproStopGMM){
                snprintf(txt,256,"gmm Stop%s",(bReproRight?"Right":"Left"));
                SendBasicCommand(txt);
            }
            snprintf(txt,256,"hand %s%s",(bReproRight?"r":"l"),(bReproOutHandState?"c":"o"));
            SendBasicCommand(txt);
        }
        if(bReproLoop){
            if(mStateSignal == SSIG_OK)    mReproNextState = IAS_REPRO_OBJTRK;
            if(mStateSignal == SSIG_ABORT) mReproNextState = IAS_REPRO_STOP;
        }else{
            mReproNextState = IAS_REPRO_STOP;
        }
        break;
    case IAS_REPRO_STOP:
        if(bReproUse3DM){
            //SendBasicCommand("do 3DRN");
            snprintf(txt,256,"do 3D%sN",(bReproRight?"R":"L"));
            SendBasicCommand(txt);
        }
        //SendBasicCommand("rest");
        mNextState = mReproOutState;
        break;
    default:
        break;
    }

    mReproPrevState  = mReproState;
    mReproState      = mReproNextState;
}
