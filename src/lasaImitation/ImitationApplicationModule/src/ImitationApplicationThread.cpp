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

    snprintf(mSrcPortName[SPID_Test1],256,"/test");
    
    snprintf(mDstPortName[DPID_Test1],256,"/test");
    
    
    mState = mPrevState = mNextState = IA_IDLE;

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
            AddConnexion(SPID_Test1,DPID_Test1);
        }
        break;
    case IA_STOP:
    case IA_REST:
        if(bStateChanged && (mPrevState!=IA_IDLE)){
            AddCommand(PID_Velocity,"mask all");
            if(mState==IA_REST) AddCommand(PID_Velocity,"rest");
            else                AddCommand(PID_Velocity,"susp");
        }
        mNextState = IA_IDLE;
        break;
    case IA_RUN:
        if(bStateChanged){
            AddCommand(PID_Velocity,"run");
        }
        break;
    }
    
    mPrevState  = mState;
    mState      = mNextState;

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

void    ImitationApplicationThread::AddConnexion(SrcPortId src, DstPortId dst){
    mCommandsType.push_back(1);
    mConnexionsSrcPort.push_back(src);
    mConnexionsDstPort.push_back(dst);
}
void    ImitationApplicationThread::RemConnexion(SrcPortId src, DstPortId dst){
    mCommandsType.push_back(2);
    mConnexionsSrcPort.push_back(src);
    mConnexionsDstPort.push_back(dst);
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
                cmd.addString(mCommands[cmdCnt].c_str());
                mPorts[mCommandsPort[cmdCnt]]->writeStrict();
                //cout << "Sending: "<< mCommands[cmdCnt].c_str()<<endl;
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
    int  retVal     = -1;
    
    if(cmdSize<=0){
        retVal = -1;
    }else{
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
    }
    if(retVal>0){
        reply.addVocab(Vocab::encode("ack"));
    }else if (retVal == 0){
        reply.addVocab(Vocab::encode("fail"));
    }

    mMutex.post();
    return retVal;
}
