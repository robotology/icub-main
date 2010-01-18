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

#include "GaussianMixtureModelModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>

int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;
  GaussianMixtureModelModule module;

  return module.runModule(argc,argv);
}


GaussianMixtureModelModule::GaussianMixtureModelModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
}
GaussianMixtureModelModule::~GaussianMixtureModelModule(){
    close();
}

bool GaussianMixtureModelModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","GaussianMixtureModel000");
        fprintf(stderr, "No module base name specifed, using <GaussianMixtureModel000> as default\n");
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
    snprintf(portName,255,"/GaussianMixtureModel/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    mControlPort.setStrict();
    attach(mControlPort,true);
    
    snprintf(portName,255,"GaussianMixtureModel/%s",getName().c_str());
    mThread = new GaussianMixtureModelThread(int(floor(mPeriod*1000)),portName);
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool GaussianMixtureModelModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    bIsReady = false;
    return true;
}

bool GaussianMixtureModelModule::respond(const Bottle& command, Bottle& reply) {
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
            if(cmdSize>1){
                     if(command.get(1).asString()=="learning"){
                    mThread->Learn();
                }else if(command.get(1).asString()=="process"){
                    mThread->ProcessRawDemos();
                }else if(command.get(1).asString()=="test"){
                    mThread->GenerateDefaultRegression();
                }else if(command.get(1).asString()=="gnuplot"){
                    mThread->GenerateGnuplotScript();
                }else if(command.get(1).asString()=="init"){
                    mThread->InitRun();
                }else if(command.get(1).asString()=="start"){
                    mThread->StartRun();
                }else if(command.get(1).asString()=="pause"){
                    mThread->PauseRun();
                }else if(command.get(1).asString()=="resume"){
                    mThread->ResumeRun();
                }else if(command.get(1).asString()=="stop"){
                    mThread->StopRun();
                }else{
                    retVal = false;
                }
            }else{
                retVal = false;
            }
            break;  
        /*
        case VOCAB4('s','u','s','p'):
            mThread->SetState(RobotControllerThread::RCS_IDLE);
            break;
        */
        case VOCAB4('l','o','a','d'):
            if(cmdSize>1){
                mThread->Load(command.get(1).asString().c_str());
            }else{
                retVal = false;
            }
            break;
        case VOCAB4('s','a','v','e'):
            if(cmdSize>1){
                mThread->Save(command.get(1).asString().c_str());
            }else{
                retVal = false;
            }
            break;
        case VOCAB3('s','e','t'):
            if(cmdSize>2){
                      if(command.get(1).asString()=="demoPath"){
                    mThread->SetEMDemosPath(command.get(2).asString().c_str());
                }else if(command.get(1).asString()=="corrPath"){
                    mThread->SetEMCorrDemosPath(command.get(2).asString().c_str());
                }else if(command.get(1).asString()=="k"){
                    mThread->SetEMNbComponents(command.get(2).asInt());
                }else if(command.get(1).asString()=="ts"){
                    mThread->SetEMTimeSpan(command.get(2).asDouble());
                }else if(command.get(1).asString()=="reproTime"){
                    mThread->SetGMRReproTime(command.get(2).asDouble());
                }else if(command.get(1).asString()=="demoId"){
                    mThread->SetCurrDemoId(command.get(2).asInt());
                }else if(command.get(1).asString()=="demoLength"){
                    mThread->SetEMDemoLength(command.get(2).asInt());
                }else if(command.get(1).asString()=="pmode"){
                          if(command.get(2).asString()=="simple"){
                        mThread->SetEMProcessingMode(GaussianMixtureModelThread::PM_SIMPLE);
                    }else if(command.get(2).asString()=="weighted"){
                        mThread->SetEMProcessingMode(GaussianMixtureModelThread::PM_WEIGHTED);
                    }else{
                        retVal = false;
                    }
                }else if(command.get(1).asString()=="imode"){
                          if(command.get(2).asString()=="load"){
                        mThread->SetEMInitialisationMode(GaussianMixtureModelThread::IM_NONE);
                    }else if(command.get(2).asString()=="timesplit"){
                        mThread->SetEMInitialisationMode(GaussianMixtureModelThread::IM_TIMESPLIT);
                    }else if(command.get(2).asString()=="kmeans"){
                        mThread->SetEMInitialisationMode(GaussianMixtureModelThread::IM_KMEANS);
                    }else if(command.get(2).asString()=="random"){
                        mThread->SetEMInitialisationMode(GaussianMixtureModelThread::IM_RAND);
                    }else{
                        retVal = false;
                    }
                }else if(command.get(1).asString()=="rmode"){
                          if(command.get(2).asString()=="rec"){
                        mThread->SetRunMode(GaussianMixtureModelThread::RM_REC);
                    }else if(command.get(2).asString()=="repro"){
                        mThread->SetRunMode(GaussianMixtureModelThread::RM_REPRO);
                    }else if(command.get(2).asString()=="corr"){
                        mThread->SetRunMode(GaussianMixtureModelThread::RM_CORR);
                    }else{
                        retVal = false;
                    }
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


double GaussianMixtureModelModule::getPeriod(){
    return 2.0;
}

int GaussianMixtureModelModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   GaussianMixtureModelModule::updateModule() {
    return true;
}



