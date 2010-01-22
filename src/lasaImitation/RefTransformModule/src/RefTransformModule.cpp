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

#include "RefTransformModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  RefTransformModule module;

  return module.runModule(argc,argv);
}


RefTransformModule::RefTransformModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
RefTransformModule::~RefTransformModule(){
    close();
}

bool RefTransformModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","RefTransform000");
        fprintf(stderr, "No module base name specifed, using <RefTransform000> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",0.05);
        fprintf(stderr, "No module period specifed, using <0.05> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.05)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <0.05> ms as default\n",mPeriod);
        mPeriod = 0.05;
    }
    
    char portName[255];
    snprintf(portName,255,"/RefTransform/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    mControlPort.setStrict();
    attach(mControlPort,true);
    
    snprintf(portName,255,"RefTransform/%s",getName().c_str());
    mThread = new RefTransformThread(int(floor(mPeriod*1000)),portName);
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool RefTransformModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool RefTransformModule::respond(const Bottle& command, Bottle& reply) {
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
            case VOCAB3('r','u','n'):
                mThread->Activate(true);
                index++;
                break;
            case VOCAB4('s','u','s','p'):
                mThread->Activate(false);
                index++;
                break;
            case VOCAB3('r','e','f'):
                if(cmdSize>=2){
                          if(command.get(index+1).asString() == "on"){
                                mThread->Enable(true);
                                index+=2;
                    }else if(command.get(index+1).asString() == "off"){
                                mThread->Enable(false);
                                index+=2;
                    }else{
                        retVal = false;
                    }
                }else{
                    retVal = false;
                }
                break;
            case VOCAB4('l','o','c','k'):
                if(cmdSize>=2){
                          if(command.get(index+1).asString() == "on"){
                                mThread->Lock(true);
                                index+=2;
                    }else if(command.get(index+1).asString() == "off"){
                                mThread->Lock(false);
                                index+=2;
                    }else{
                        retVal = false;
                    }
                }else{
                    retVal = false;
                }
                break;
            case VOCAB3('o','r','i'):
                if(cmdSize>=2){
                          if(command.get(index+1).asString() == "on"){
                                mThread->EnableOrient(true);
                                index+=2;
                    }else if(command.get(index+1).asString() == "off"){
                                mThread->EnableOrient(false);
                                index+=2;
                    }else{
                        retVal = false;
                    }
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


double RefTransformModule::getPeriod(){
    return 2.0;
}

int RefTransformModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   RefTransformModule::updateModule() {
    return true;
}



