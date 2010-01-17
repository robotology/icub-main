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
    
    snprintf(mDstCtrlPortName[PID_Velocity],        256,"/VelocityController/rpc");
    snprintf(mDstCtrlPortName[PID_Robot],           256,"/RobotController/rpc");
    snprintf(mDstCtrlPortName[PID_3DMouse],         256,"/TouchController/3DMouse/rpc");
    snprintf(mDstCtrlPortName[PID_Touchpad],        256,"/TouchController/Touchpad/rpc");
    snprintf(mDstCtrlPortName[PID_GMMRight],        256,"/GaussianMixtureModel/Right/rpc");
    snprintf(mDstCtrlPortName[PID_GMMLeft],         256,"/GaussianMixtureModel/Left/rpc");

    //for(int i=0;i<PID_SIZE;i++){
    //    cout << mSrcCtrlPortName[i] << " "<<mDstCtrlPortName[i]<<endl;
    //}
    
    mVelocityControllerPort.open(   mSrcCtrlPortName[PID_Velocity]);
    mRobotControllerPort.open(      mSrcCtrlPortName[PID_Robot]);
    m3DMouseControllerPort.open(    mSrcCtrlPortName[PID_3DMouse]);
    mTouchpadControllerPort.open(   mSrcCtrlPortName[PID_Touchpad]);
    mGMMRightPort.open(             mSrcCtrlPortName[PID_GMMRight]);
    mGMMLeftPort.open(              mSrcCtrlPortName[PID_GMMLeft]);

    mPorts[PID_Velocity]    = &mVelocityControllerPort;
    mPorts[PID_Robot]       = &mRobotControllerPort;
    mPorts[PID_3DMouse]     = &m3DMouseControllerPort;
    mPorts[PID_Touchpad]    = &mTouchpadControllerPort;
    mPorts[PID_GMMRight]    = &mGMMRightPort;
    mPorts[PID_GMMLeft]     = &mGMMLeftPort;

    
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

    
    snprintf(mDstPortName[DPID_Touchpad],           256,"/TouchController/Touchpad/frameOfRef");
    snprintf(mDstPortName[DPID_3DMouse],            256,"/TouchController/3DMouse/frameOfRef");
    snprintf(mDstPortName[DPID_RArmDesCartVel],     256,"/RobotController/desiredCartVelocityR");
    snprintf(mDstPortName[DPID_LArmDesCartVel],     256,"/RobotController/desiredCartVelocityL");
    snprintf(mDstPortName[DPID_RArmDesCartPos],     256,"/RobotController/desiredCartPositionR");
    snprintf(mDstPortName[DPID_LArmDesCartPos],     256,"/RobotController/desiredCartPositionL");
    snprintf(mDstPortName[DPID_RWristDesCartVel],   256,"/RobotController/desiredCartWristVelocityR");
    snprintf(mDstPortName[DPID_LWristDesCartVel],   256,"/RobotController/desiredCartWristVelocityL");
    snprintf(mDstPortName[DPID_EyeInEyeDesCartPos], 256,"/RobotController/desiredCartEyeInEyePosition");
    snprintf(mDstPortName[DPID_EyeDesCartPos],      256,"/RobotController/desiredCartEyePosition");
    snprintf(mDstPortName[DPID_GMMRightSignal],     256,"/GaussianMixtureModel/Right/signals");
    snprintf(mDstPortName[DPID_GMMLeftSignal],      256,"/GaussianMixtureModel/Left/signals");

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
        cout << "Wiimote says: "<<mWiimoteEvent.toString()<<endl;
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
            if(mWiimoteEvent[4]>0.0){ cmd.fromString("next"); bCmd = true;}
            if(mWiimoteEvent[5]>0.0){ cmd.fromString("prev"); bCmd = true;}
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
        AddCommand(PID_Robot,   "run");
        AddCommand(PID_Velocity,"run");
        break;
    case BC_STOP:
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
        AddCommand(PID_Robot,"iks RightArm");
        break;
    case BC_3DMOUSE_TO_LEFTARM:
        RemAllSrcConnexions(DPID_3DMouse);
        AddConnexion(SPID_3DMouse, DPID_LArmDesCartVel);
        AddCommand(PID_Robot,"iks LeftArm");
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
    case BC_TRACK_NONE:
        RemAllSrcConnexions(DPID_EyeDesCartPos);
        RemAllSrcConnexions(DPID_EyeInEyeDesCartPos);
        AddCommand(PID_Robot,"iku Eye");
        break;
    case BC_TRACK_RIGHTARM:
        AddConnexion(SPID_RArmCartPos, DPID_EyeDesCartPos);
        AddCommand(PID_Robot,"iks Eye");
        break;
    case BC_TRACK_LEFTARM:
        AddConnexion(SPID_LArmCartPos, DPID_EyeDesCartPos);
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
            AddConnexion(SPID_GMMRightCartPos, DPID_RArmDesCartPos);
            AddCommand(PID_Robot,"iks RightArm");
        }else{
            AddConnexion(SPID_GMMLeftCartPos, DPID_LArmDesCartPos);
            AddCommand(PID_Robot,"iks LeftArm");
        }
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
    }
    
    mBasicCommand       = BC_NONE;
    mBasicCommandParams = "";
}

void ImitationApplicationThread::ProcessStateMachine(){
    if(mStateSignal!=SSIG_NONE)
        cout << "SSIG: "<<mStateSignal<<endl;

    if(mNextState!=mState)
        mState = mNextState;
    
    bool bStateChanged = (mPrevState != mState);
    if(bStateChanged){
        cout << "State change: <"<<mStateName[mPrevState]<<">("<<mPrevState<<") -> <"<<mStateName[mState]<<">("<<mState<<")"<<endl; 
    }
    
    mNextState = mState;
    switch(mState){
    case IAS_IDLE:
        break;
    case IAS_INIT:
        break;
    case IAS_RUN:
        if(bStateChanged){
        }
        break;
    case IAS_PAUSE:
        break;
    case IAS_STOP:
        break;
    default:
        break;
    }
    
    mPrevState  = mState;
    mState      = mNextState;

    
    mStateSignal = SSIG_NONE;
}


void ImitationApplicationThread::ConnectToNetwork(bool bConnect){
    cout << (bConnect?"Connecting to network":"Disonnecting from network")<<endl;
    for(int i=0;i<PID_SIZE;i++){
        cout << "  From: "<< mSrcCtrlPortName[i] <<" to: "<< mDstCtrlPortName[i] <<endl;
        if(bConnect){
            if(!Network::isConnected(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                if(!Network::connect(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                    cerr << "Error: Unable to connect "<<mSrcCtrlPortName[i] <<" to "<<mDstCtrlPortName[i]<<endl;
        }else{
            if( Network::isConnected(mSrcCtrlPortName[i],mDstCtrlPortName[i]))
                Network::disconnect(mSrcCtrlPortName[i],mDstCtrlPortName[i]);
        }
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
                Bottle &cmd = mPorts[mCommandsPort[cmdCnt]]->prepare();
                cmd.fromString(mCommands[cmdCnt].c_str());
                /*cmd.clear();
                vector<string> currCmds = Tokenize(RemoveSpaces(mCommands[cmdCnt]));
                for(size_t j=0;j<currCmds.size();j++)
                    cmd.addString(currCmds[j].c_str());
                */
                cout << "  Sending: <"<<mCommands[cmdCnt]<<"> to: "<< mDstCtrlPortName[mCommandsPort[cmdCnt]] <<endl;
                mPorts[mCommandsPort[cmdCnt]]->writeStrict();
                cmdCnt++;
                break;
            }
        case 1:
            {
                cout << "  Connecting: "<<mSrcPortName[mConnexionsSrcPort[conCnt]]<<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;
                if(!Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                    if(!Network::connect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                        cerr << "Error: Unable to connect "<<mSrcPortName[mConnexionsSrcPort[conCnt]] <<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;                
                conCnt++;
            }
            break;
        case 2:
            {
                //cout << "disconnecting... "<<mSrcPortName[mConnexionsSrcPort[conCnt]]<<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;
                if(Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                    Network::disconnect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]);
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
        case VOCAB4('n','e','x','t'):
            mStateSignal = SSIG_NEXT;
            break;
        case VOCAB4('p','r','e','v'):
            mStateSignal = SSIG_PREV;
            break;
        case VOCAB2('o','k'):
            mStateSignal = SSIG_OK;
            break;
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
        cout << "*ACK*"<<endl;
    }else if (retVal == 0){
        reply.addVocab(Vocab::encode("fail"));
        cout << "*FAIL*"<<endl;
    }else{
        cout << "*BIG_FAIL*"<<endl;
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

void ImitationApplicationThread::InitStateMachine(){
    for(int i=0;i<IAS_SIZE;i++)
        mStateName[i][0] = 0;

    snprintf(mStateName[IAS_IDLE],              256,"IDLE");
    snprintf(mStateName[IAS_INIT],              256,"INIT");
    snprintf(mStateName[IAS_RUN],               256,"RUN");
    snprintf(mStateName[IAS_PAUSE],             256,"PAUSE");
    snprintf(mStateName[IAS_STOP],              256,"STOP");

    mState = mPrevState = mNextState = IAS_IDLE;
    mStateSignal = SSIG_NONE;

    cout << "State names: "<<endl;
    cout << "*******************"<<endl;
    for(int i=0;i<IAS_SIZE;i++)
        cout <<"  "<<i<<":" <<mStateName[i]<<endl;
}

