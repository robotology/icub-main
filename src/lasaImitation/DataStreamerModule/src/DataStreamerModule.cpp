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
    mControlPort.setStrict();
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
    int index = 0;
    int cmdSize = command.size();

    bool retVal = true;
    
    if(cmdSize>0){
        cerr << "Cmd: ";
        for(int i=0;i<cmdSize;i++)
            cerr <<"*"<<command.get(i).asString().c_str()<<"*";
        cerr <<endl;
            
        switch(command.get(0).asVocab()) {
        case VOCAB3('r','u','n'):
            if(cmdSize>=2){
                      if(command.get(1).asString() == "start"){
                            mThread->SetLoop(false);
                            if(cmdSize>=3){
                                mThread->Start(command.get(2).asDouble());
                            }else{
                                mThread->Start();
                            }
                }else if(command.get(1).asString() == "loop"){
                            mThread->SetLoop(true);
                            mThread->Start();
                }else if(command.get(1).asString() == "stop"){
                            mThread->Stop();
                }else if(command.get(1).asString() == "pause"){
                            mThread->Pause();
                }else if(command.get(1).asString() == "resume"){
                            mThread->Resume();
                }else if(command.get(1).asString() == "resumeOnce"){
                            mThread->Resume(true);
                }else{
                    retVal = false;
                }
            }else{
                retVal = false;
            }
            break;
            
        case VOCAB3('r','e','c'):
            if(cmdSize>=2){
                      if(command.get(1).asString() == "set"){
                            mThread->SetRecordMode(true);
                }else if(command.get(1).asString() == "unset"){
                            mThread->SetRecordMode(false);
                }else{
                    retVal = false;
                }
            }else{
                retVal = false;
            }
            break;

        case VOCAB4('d','a','t','a'):
            if(cmdSize>=2){
                      if(command.get(1).asString() == "lineSize"){
                            if(cmdSize>=3){
                                mThread->SetStreamLineSize(command.get(2).asInt());
                            }else{
                                retVal = false;
                            }
                }else if(command.get(1).asString() == "maxSize"){
                            if(cmdSize>=3){
                                mThread->SetStreamMaxSize(command.get(2).asInt());
                            }else{
                                retVal = false;
                            }
                }else if(command.get(1).asString() == "load"){
                            if(cmdSize>=3){
                                mThread->Load(command.get(2).asString().c_str());
                            }else{
                                retVal = false;
                            }
                }else if(command.get(1).asString() == "save"){
                            if(cmdSize>=3){
                                mThread->Save(command.get(2).asString().c_str());
                            }else{
                                retVal = false;
                            }
                }else if(command.get(1).asString() == "timeOn"){
                            mThread->SetUseTime(true);
                }else if(command.get(1).asString() == "timeOff"){
                            mThread->SetUseTime(false);
                }else if(command.get(1).asString() == "clear"){
                            mThread->Clear();
                }else{
                    retVal = false;
                }
            }else{
                retVal = false;
            }
            break;

        default:
            retVal = false;
            break;            
        }         
    }else{
        retVal = false;
    }
    if(retVal)
        cerr << "*ACK*"<<endl;
    else 
        cerr << "*FAIL*"<<endl;
    if(retVal){
        reply.addVocab(Vocab::encode("ack"));
    }else{
        return Module::respond(command,reply);
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



