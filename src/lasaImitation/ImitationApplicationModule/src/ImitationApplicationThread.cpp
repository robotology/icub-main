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
    char portName[256];

    snprintf(portName,256,"/%s/VC",mBaseName);
    mVelocityControllerPort.open(portName);
    snprintf(portName,256,"/%s/RC",mBaseName);
    mRobotControllerPort.open(portName);
    snprintf(portName,256,"/%s/TC3D",mBaseName);
    m3DMouseControllerPort.open(portName);
    snprintf(portName,256,"/%s/TCTP",mBaseName);
    mTouchpadControllerPort.open(portName);

    mPorts[PID_Velocity]    = &mVelocityControllerPort;
    mPorts[PID_Robot]       = &mRobotControllerPort;
    mPorts[PID_3DMouse]     = &m3DMouseControllerPort;
    mPorts[PID_Touchpad]    = &mTouchpadControllerPort;

    snprintf(mSrcPortName[SPID_Touchpad],           256,"/TouchController/Touchpad/velocity");
    snprintf(mSrcPortName[SPID_3DMouse],            256,"/TouchController/3DMouse/velocity");
    snprintf(mSrcPortName[SPID_RWristRef],          256,"/RobotController/currentWristRefR");
    snprintf(mSrcPortName[SPID_LWristRef],          256,"/RobotController/currentWristRefL");
    snprintf(mSrcPortName[SPID_RArmCartPos],        256,"/RobotController/currentCartPositionR");
    snprintf(mSrcPortName[SPID_LArmCartPos],        256,"/RobotController/currentCartPositionL");
    snprintf(mSrcPortName[SPID_EyeCartPos],        256,"/RobotController/currentCartEyeTargetPosition");
    snprintf(mSrcPortName[SPID_SIZE],               256,"");

    
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
    snprintf(mDstPortName[DPID_SIZE],               256,"");

    
    mState = mPrevState = mNextState = IA_IDLE;

    mBasicCommand = BC_NONE;
    
    return true;
}
void ImitationApplicationThread::PrepareToStop(){
    mMutex.wait();
    mNextState = IA_STOP;
    mMutex.post();
    while(mState != IA_IDLE){
        Time::delay(double(mPeriod)*0.001);
        if(mState != IA_IDLE)
            cerr<<"Waiting to quit"<<endl;
    }
}

void ImitationApplicationThread::threadRelease()
{
    ConnectToNetwork(false);
    
    mVelocityControllerPort.close();
    mRobotControllerPort.close();
    m3DMouseControllerPort.close();
    mTouchpadControllerPort.close();
}

void ImitationApplicationThread::run()
{
    mMutex.wait();
    ClearCommands();
    
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
    }
    
    mBasicCommand = BC_NONE;

    /*
    if(mNextState!=mState)
        mState = mNextState;
    
    bool bStateChanged = (mPrevState != mState);
    if(bStateChanged){
        cout << "State change: <"<<mPrevState<<"> -> <"<<mState<<">"<<endl;
    }
    
    mNextState = mState;
    switch(mState){
    case IA_IDLE:
        break;
    case IA_INIT:
        if(bStateChanged){
            ConnectToNetwork(true);
            AddCommand(PID_Velocity,"kd 0.1");
            //AddConnexion(SPID_Test1,DPID_Test1);
        }
        break;
    case IA_STOP:
    case IA_REST:
        if(bStateChanged){
            AddCommand(PID_Velocity,"mask all");
            if(mState==IA_REST) AddCommand(PID_Velocity,"rest");
            else                AddCommand(PID_Velocity,"susp");
            AddCommand(PID_Robot,"susp");
            RemAllConnexions();
        }
        mNextState = IA_IDLE;
        break;
    case IA_RUN:
        if(bStateChanged){
            AddConnexion(SPID_3DMouse, DPID_LArmDesCartVel);
            
            AddCommand(PID_Robot,   "run");
            AddCommand(PID_Robot,   "iks LeftArm");

            AddCommand(PID_Velocity,"run");
        }
        break;
    }
    
    mPrevState  = mState;
    mState      = mNextState;
    */
    
    SendCommands();
    mMutex.post();
    
}

void ImitationApplicationThread::ConnectToNetwork(bool bConnect){
    char srcPortName[16][256];
    char dstPortName[16][256];

    snprintf(srcPortName[0],256,"/%s/VC",mBaseName);
    snprintf(dstPortName[0],256,"/VelocityController/rpc");
    snprintf(srcPortName[1],256,"/%s/RC",mBaseName);
    snprintf(dstPortName[1],256,"/RobotController/rpc");
    snprintf(srcPortName[2],256,"/%s/TC3D",mBaseName);
    snprintf(dstPortName[2],256,"/TouchController/3DMouse/rpc");
    snprintf(srcPortName[3],256,"/%s/TCTP",mBaseName);
    snprintf(dstPortName[3],256,"/TouchController/Touchpad/rpc");

    
    int nbConn = 4;
    for(int i=0;i<nbConn;i++){
        if(bConnect){
            if(!Network::isConnected(srcPortName[i],dstPortName[i]))
                if(!Network::connect(srcPortName[i],dstPortName[i]))
                    cerr << "Error: Unable to connect "<<srcPortName[i] <<" to "<<dstPortName[i]<<endl;
        }else{
            if( Network::isConnected(srcPortName[i],dstPortName[i]))
                Network::disconnect(srcPortName[i],dstPortName[i]);
        }
    }
}

void ImitationApplicationThread::ClearCommands(){
    mCommandsType.clear();
    mCommands.clear();
    mCommandsPort.clear();
    mConnexionsSrcPort.clear();
    mConnexionsDstPort.clear();
}

void ImitationApplicationThread::AddCommand(PortId port, const char *cmd){
    if(cmd!=NULL){
        if(cmd[0]!=0){
            mCommandsType.push_back(0);
            mCommands.push_back(cmd);
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
    
    for(size_t i=0;i<mCommandsType.size();i++){
        switch(mCommandsType[i]){
        case 0:
            {
                Bottle &cmd = mPorts[mCommandsPort[cmdCnt]]->prepare();
                cmd.clear();
                vector<string> currCmds = Tokenize(RemoveSpaces(mCommands[cmdCnt]));
                for(size_t j=0;j<currCmds.size();j++)
                    cmd.addString(currCmds[j].c_str());
                
                mPorts[mCommandsPort[cmdCnt]]->writeStrict();
                cmdCnt++;
                break;
            }
        case 1:
            {
                if(!Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                    if(!Network::connect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                        cerr << "Error: Unable to connect "<<mSrcPortName[mConnexionsSrcPort[conCnt]] <<" to "<<mDstPortName[mConnexionsDstPort[conCnt]]<<endl;                
                conCnt++;
            }
            break;
        case 2:
            {
                if(Network::isConnected(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]))
                    Network::disconnect(mSrcPortName[mConnexionsSrcPort[conCnt]],mDstPortName[mConnexionsDstPort[conCnt]]);
                conCnt++;
            }
            break;
        }        
    }
    /*if(cmdCnt+conCnt>0)
        cerr <<"Sending done..."<<endl;*/

}


int ImitationApplicationThread::respond(const Bottle& command, Bottle& reply){
    mMutex.wait();

    int  cmdSize    = command.size();
    int  retVal     = 1;
    
    if(cmdSize<=0){
        retVal = -1;
    }else{
        switch(command.get(0).asVocab()) {
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
        default:
            retVal = -1;
            break;
        }
        /*
        switch(command.get(0).asVocab()) {
        case VOCAB4('i','n','i','t'):
            if(mState == IA_IDLE){ 
                mNextState = IA_INIT;
                retVal = 1;
            }else{
                retVal = 0;
            }
            break;
        case VOCAB3('r','u','n'):
            if(mState == IA_INIT){ 
                mNextState = IA_RUN;
                retVal = 1;
            }else{
                retVal = 0;
            }
            break;
        case VOCAB4('s','t','o','p'):
            mNextState = IA_STOP;
            retVal = 1;
            break;
        case VOCAB4('r','e','s','t'):
            mNextState = IA_REST;
            retVal = 1;
            break;
        }
        */
    }
    if(retVal>0){
        reply.addVocab(Vocab::encode("ack"));
        cout << "*ACK*"<<endl;
    }else if (retVal == 0){
        reply.addVocab(Vocab::encode("fail"));
        cout << "*FAIL*"<<endl;
    }
    


    mMutex.post();
    return retVal;
}
