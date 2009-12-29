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

#include "DataStreamerModule.h"

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  DataStreamerModule module;

  return module.runModule(argc,argv);
}

DataStreamerModule::DataStreamerModule(){
    mPeriod         = 0.01;
    bIsReady        = false;
    mThread         = NULL;
}
DataStreamerModule::~DataStreamerModule(){
    close();
}

bool DataStreamerModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","DataStreamer000");
        fprintf(stderr, "No module base name specifed, using <DataStreamer000> as default\n");
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
    snprintf(portName,255,"/DataStreamer/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    attach(mControlPort,true);
    
    snprintf(portName,255,"DataStreamer/%s",getName().c_str());
    mThread = new DataStreamerThread(int(floor(mPeriod*1000)),portName);
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool DataStreamerModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool DataStreamerModule::respond(const Bottle& command, Bottle& reply) {
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
                if(cmdSize>=2){
                          if(command.get(index+1).asString() == "start"){
                                mThread->SetLoop(false);
                                mThread->Start();
                                index+=2;
                    }else if(command.get(index+1).asString() == "loop"){
                                mThread->SetLoop(true);
                                mThread->Start();
                                index+=2;
                    }else if(command.get(index+1).asString() == "stop"){
                                mThread->Stop();
                                index+=2;
                    }else if(command.get(index+1).asString() == "pause"){
                                mThread->Pause();
                                index+=2;
                    }else if(command.get(index+1).asString() == "resume"){
                                mThread->Resume();
                                index+=2;
                    }else{
                        retVal = false;
                    }
                }else{
                    retVal = false;
                }
                break;
                
            case VOCAB3('r','e','c'):
                if(cmdSize>=2){
                          if(command.get(index+1).asString() == "set"){
                                mThread->SetRecordMode(true);
                                index+=2;
                    }else if(command.get(index+1).asString() == "unset"){
                                mThread->SetRecordMode(false);
                                index+=2;
                    }else{
                        retVal = false;
                    }
                }else{
                    retVal = false;
                }
                break;

            case VOCAB4('d','a','t','a'):
                if(cmdSize>=2){
                          if(command.get(index+1).asString() == "lineSize"){
                                if(cmdSize>=3){
                                    mThread->SetStreamLineSize(command.get(index+2).asInt());
                                    index+=3;
                                }else{
                                    retVal = false;
                                }
                    }else if(command.get(index+1).asString() == "maxSize"){
                                if(cmdSize>=3){
                                    mThread->SetStreamMaxSize(command.get(index+2).asInt());
                                    index+=3;
                                }else{
                                    retVal = false;
                                }
                    }else if(command.get(index+1).asString() == "load"){
                                if(cmdSize>=3){
                                    mThread->Load(command.get(index+2).asString().c_str());
                                    index+=3;
                                }else{
                                    retVal = false;
                                }
                    }else if(command.get(index+1).asString() == "save"){
                                if(cmdSize>=3){
                                    mThread->Save(command.get(index+2).asString().c_str());
                                    index+=3;
                                }else{
                                    retVal = false;
                                }
                    }else if(command.get(index+1).asString() == "timeOn"){
                                mThread->SetUseTime(true);
                                index+=2;
                    }else if(command.get(index+1).asString() == "timeOff"){
                                mThread->SetUseTime(false);
                                index+=2;
                    }else if(command.get(index+1).asString() == "clear"){
                                mThread->Clear();
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


double DataStreamerModule::getPeriod(){
    return 2.0;
}

int DataStreamerModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   DataStreamerModule::updateModule() {
    return true;
}



