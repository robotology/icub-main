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

#include "ImitationApplicationModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  ImitationApplicationModule module;

  return module.runModule(argc,argv);
}


ImitationApplicationModule::ImitationApplicationModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
ImitationApplicationModule::~ImitationApplicationModule(){
    close();
}

bool ImitationApplicationModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","ImitationApplication000");
        fprintf(stderr, "No module base name specifed, using <ImitationApplication000> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",1.0);
        fprintf(stderr, "No module period specifed, using <1.00> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 1.0)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <1.0> ms as default\n",mPeriod);
        mPeriod = 1.0;
    }
    
    char portName[255];
    snprintf(portName,255,"/ImitationApplication/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    attach(mControlPort,true);
    
    snprintf(portName,255,"ImitationApplication/%s",getName().c_str());
    mThread = new ImitationApplicationThread(int(floor(mPeriod*1000)),portName);
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool ImitationApplicationModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool ImitationApplicationModule::respond(const Bottle& command, Bottle& reply) {
    
    bool retVal = true;
    int res = mThread->respond(command,reply);
    if(res<0){
        retVal = Module::respond(command,reply);
        if(retVal){
            reply.addVocab(Vocab::encode("ack"));
        }
    }else{
        retVal = (res>0);
    }
    return retVal;
}


double ImitationApplicationModule::getPeriod(){
    return 2.0;
}

int ImitationApplicationModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   ImitationApplicationModule::updateModule() {
    return true;
}



