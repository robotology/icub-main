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

#include "WiimoteServerModule.h"

#include <yarp/os/Network.h>
using namespace yarp::os;

#include <math.h>


int main(int argc, char *argv[])
{
  Network yarp;
  if(!yarp.checkNetwork())
      return 0;

#define MAX_WIIMOTES 1
    

    WiimoteServerModule module;
    
    bool res = module.runModule(argc,argv);
    return res;
}




WiimoteServerModule::WiimoteServerModule(){
    mPeriod         = 0.02;
    bIsReady        = false;
    mThread         = NULL;
    
}
WiimoteServerModule::~WiimoteServerModule(){
    close();
}

bool WiimoteServerModule::open(Searchable &s){
    if(bIsReady)
        return true;

    if(!mParams.check("name")){
        mParams.put("name","WiimoteServer000");
        fprintf(stderr, "No module base name specifed, using <WiimoteServer000> as default\n");
        fprintf(stderr, "  usage: --name name (e.g. test)\n");
    }
    setName(mParams.find("name").asString());
    
    if(!mParams.check("period")){
        mParams.put("period",0.1);
        fprintf(stderr, "No module period specifed, using <0.1> ms as default\n");
        fprintf(stderr, "  usage: --period time (e.g. 0.1)\n");
    }
    mPeriod = mParams.find("period").asDouble();
    if(mPeriod<=0.0){
        fprintf(stderr, "Period specifed, %f<0, using <0.1> ms as default\n",mPeriod);
        mPeriod = 0.1;
    }


    //wiimote** wiimotes;
    int found, connected;
    WiimoteServerThread::sWiimotes =  wiiuse_init(MAX_WIIMOTES);

    printf ("Please turn on your wiimote now...\n");
    found = wiiuse_find(WiimoteServerThread::sWiimotes, MAX_WIIMOTES, 5);
    if (!found) {
        printf ("No wiimotes found.\n");
        return false;
    }
    connected = wiiuse_connect(WiimoteServerThread::sWiimotes, MAX_WIIMOTES);
    if (connected)
        printf("Connected to %i wiimotes (of %i found).\n", connected, found);
    else {
        printf("Failed to connect to any wiimote.\n");
        return false;
    }
    wiiuse_rumble(WiimoteServerThread::sWiimotes[0], 1);
    //wiiuse_set_leds(WiimoteServerThread::sWiimotes[0], WIIMOTE_LED_1|WIIMOTE_LED_2|WIIMOTE_LED_3|WIIMOTE_LED_4);
    usleep(200000);
    wiiuse_rumble(WiimoteServerThread::sWiimotes[0], 0);
    wiiuse_set_leds(WiimoteServerThread::sWiimotes[0], WIIMOTE_LED_2|WIIMOTE_LED_3);

    char portName[255];
    snprintf(portName,255,"/WiimoteServer/%s/rpc",getName().c_str());
    mControlPort.open(portName);
    attach(mControlPort,true);
    
    snprintf(portName,255,"WiimoteServer/%s",getName().c_str());
    mThread = new WiimoteServerThread(int(floor(mPeriod*1000)),portName);
    mThread->start();
    
    bIsReady = true;
    return bIsReady;
}

bool WiimoteServerModule::close(){
    if(!bIsReady)
        return true;
    
    mControlPort.close();

    mThread->stop();
    delete mThread;
    mThread = NULL;
    
    wiiuse_cleanup(WiimoteServerThread::sWiimotes, MAX_WIIMOTES);
    printf("exititng\n");

    bIsReady = false;
    return true;
}

bool WiimoteServerModule::respond(const Bottle& command, Bottle& reply) {
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
            case VOCAB4('e','m','o','d'):
                mThread->SetEventMode(true);
                index++;
                break;
            case VOCAB4('s','m','o','d'):
                mThread->SetEventMode(false);
                index++;
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


double WiimoteServerModule::getPeriod(){
    return 2.0;
}

int WiimoteServerModule::runModule(int argc, char *argv[], bool skipFirst){
    mParams.fromCommand(argc, argv);
    return Module::runModule(argc,argv,skipFirst);
}

bool   WiimoteServerModule::updateModule() {
    return !WiimoteServerThread::sDisconnected;
}



