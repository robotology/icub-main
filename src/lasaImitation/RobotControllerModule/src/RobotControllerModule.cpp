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

#include "RobotControllerModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>
#include <string.h>

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  RobotControllerModule module;

  return module.runModule(argc,argv);
}


RobotControllerModule::RobotControllerModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
RobotControllerModule::~RobotControllerModule(){
    close();
}

bool RobotControllerModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","");
        fprintf(stderr, "No module base name specifed, using <> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",0.01);
        fprintf(stderr, "No module period specifed, using <0.01> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.01)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <0.01> ms as default\n",mPeriod);
        mPeriod = 0.01;
    }
    
    char portName[255];
    if(strlen(getName().c_str())==0)
        snprintf(portName,255,"/RobotController/rpc",getName().c_str());
    else
        snprintf(portName,255,"/RobotController/%s/rpc",getName().c_str());

    mControlPort.open(portName);
    mControlPort.setStrict();
    attach(mControlPort,true);
    
    if(strlen(getName().c_str())==0)
        snprintf(portName,255,"RobotController",getName().c_str());
    else
        snprintf(portName,255,"RobotController/%s",getName().c_str());
    mThread = new RobotControllerThread(int(floor(mPeriod*1000)),portName);
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool RobotControllerModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool RobotControllerModule::respond(const Bottle& command, Bottle& reply) {
    int index = 0;
    int cmdSize = command.size();

    bool retVal = true;
    
    if(cmdSize>0){
        cerr << "Cmd: ";
        for(int i=0;i<cmdSize;i++)
            cerr <<"*"<<command.get(i).asString().c_str()<<"*";
        cerr <<endl;

        switch(command.get(0).asVocab())  {
        case VOCAB3('r','u','n'):
            mThread->SetState(RobotControllerThread::RCS_RUN);
            break;
        case VOCAB4('s','u','s','p'):
            mThread->SetState(RobotControllerThread::RCS_IDLE);
            break;
        case VOCAB3('i','k','s'):
            if(cmdSize>1){
                for(int i=1;i<cmdSize;i++){
                    RobotControllerThread::IKSetID id = mThread->IKStringToIKSet(command.get(i).asString().c_str());
                    mThread->SetIKSolverSet(id,true);
                }
            }
            break;
        case VOCAB3('i','k','u'):
            if(cmdSize>1){
                for(int i=1;i<cmdSize;i++){
                    RobotControllerThread::IKSetID id = mThread->IKStringToIKSet(command.get(i).asString().c_str());
                    mThread->SetIKSolverSet(id,false);
                }
            }
            break;
        default:
            retVal = false;
            break;            
        }         
    }else{
        retVal = false;
    }

    if(retVal){
        reply.addVocab(Vocab::encode("ack"));
    }else{
        return Module::respond(command,reply);
    }
    return retVal;
}


double RobotControllerModule::getPeriod(){
    return 2.0;
}

int RobotControllerModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   RobotControllerModule::updateModule() {
    return true;
}



