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

#include "TouchControllerModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>
#include <string.h>

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  TouchControllerModule module;

  return module.runModule(argc,argv);
}


TouchControllerModule::TouchControllerModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
TouchControllerModule::~TouchControllerModule(){
    close();
}

bool TouchControllerModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","Touch000");
        fprintf(stderr, "No module base name specifed, using <Touch000> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",0.05);
        fprintf(stderr, "No module period specifed, using <0.01> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.01)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <0.01> ms as default\n",mPeriod);
        mPeriod = 0.01;
    }

    if(!mParams.check("type")){
        fprintf(stderr, "No device type specifed: please specify either '3Dmouse' or 'touchpad'\n");
        fprintf(stderr, "  usage: --type <type>\n");
        return false;
    }
    char cType[256];
    strncpy(cType,mParams.find("type").asString().c_str(),256);
    if(strncmp(cType,"3Dmouse",256)==0){
        mType = 0;
    }else if(strncmp(cType,"touchpad",256)==0){
        mType = 1;    
    }else{
        fprintf(stderr, "Bad device type specifed: please specify either '3Dmouse' or 'touchpad'\n");
        fprintf(stderr, "  usage: --type <type>\n");
        return false;
    }

    if(!mParams.check("device")){
        fprintf(stderr, "No device name specifed\n");
        fprintf(stderr, "  usage: --device name (e.g. 3Dconnexion)\n");
        return false;
    }
    strncpy(mDeviceName,mParams.find("device").asString().c_str(),256);
    
    
    char portName[255];
    snprintf(portName,255,"/TouchController/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    attach(mControlPort,true);
    
    snprintf(portName,255,"TouchController/%s",getName().c_str());
    mThread = new TouchControllerThread(int(floor(mPeriod*1000)),portName);
    mThread->SetDevice(mDeviceName,mType);
    if(!mThread->start()){
        return false;
    }
    
    bIsReady = true;
    return bIsReady;
}

bool TouchControllerModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool TouchControllerModule::respond(const Bottle& command, Bottle& reply) {
    int  index      = 0;
    int  cmdSize    = command.size();
    bool retVal     = true;
    bool defRetVal  = false;
    
    
    if(cmdSize<=0){
        retVal = false;
    }else{
        while(cmdSize>0){
            int prevIndex = index;
            
            switch(command.get(index).asVocab()) {
            case VOCAB3('s','i','d'):
                if(cmdSize>=2){
                    int id = command.get(index+1).asInt();
                    mThread->SetSensorIdByTouch(id);
                    index+=2;
                }else{
                    retVal = false;
                }
                break;
            case VOCAB3('r','u','n'):
                mThread->Activate(true);
                index+=1;
                break;
            case VOCAB4('s','u','s','p'):
                mThread->Activate(false);
                index+=1;
                break;
            case VOCAB3('k','p','t'):
                if(cmdSize>=2){
                    mThread->SetTransGain(command.get(index+1).asDouble());
                    index+=2;
                }else{
                    retVal = false;
                }
                break;
            case VOCAB3('k','p','r'):
                if(cmdSize>=2){
                    mThread->SetRotGain(command.get(index+1).asDouble());
                    index+=2;
                }else{
                    retVal = false;
                }
                break;
            case VOCAB4('l','i','m','t'):
                if(cmdSize>=2){
                    mThread->SetTransLimit(command.get(index+1).asDouble());
                    index+=2;
                }else{
                    retVal = false;
                }
                break;
            case VOCAB4('l','i','m','r'):
                if(cmdSize>=2){
                    mThread->SetRotLimit(command.get(index+1).asDouble());
                    index+=2;
                }else{
                    retVal = false;
                }
                break;
            default:
                retVal      = Module::respond(command,reply);
                defRetVal   = true;
                break;
            }
            
            if(defRetVal){
                return retVal;
            }else{
                if(retVal){
                    cmdSize -= index-prevIndex;
                }else{
                    break;
                }
            }
        }
    }

    if(retVal){
        reply.addVocab(Vocab::encode("ack"));
    }
    return retVal;
}


double TouchControllerModule::getPeriod(){
    return 2.0;
}

int TouchControllerModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   TouchControllerModule::updateModule() {
    return true;
}



