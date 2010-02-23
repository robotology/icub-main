// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/**
*
@ingroup icub_module
\defgroup icub_controlBasisControlLaw controlBasisControlLaw

A module that implements a prioritized control law 

\section intro_sec Description
This module allows a user to start up a control basis control law that
attempts to accomplish a set of prioritized control laws.  

\section lib_sec Libraries
YARP

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed
None.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files
None.

\section conf_file_sec Configuration Files
None.

\section tested_os_sec Tested OS
Linux.

\section example_sec Example Instantiation of the Module

controlBasisControlLaw --?

\author Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/controlBasis/modules/main.cpp.
**/

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include "RunnableControlLaw.h"
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>

#include <iostream>
#include <string.h>

#include "PotentialFunctionRegister.h"
#include "JacobianRegister.h"

using namespace yarp::os;
using namespace yarp::sig;

using namespace std;
using namespace CB;

#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_RUN VOCAB3('r','u','n')
#define COMMAND_VOCAB_STOP VOCAB4('s','t','o','p')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_ADD VOCAB3('a','d','d')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_QUIT VOCAB4('q','u','i','t')

#define COMMAND_VOCAB_SENSOR_NAME VOCAB4('s','e','n','s')
#define COMMAND_VOCAB_REFERENCE_NAME VOCAB4('r','e','f','e')
#define COMMAND_VOCAB_EFFECTOR_NAME VOCAB4('e','f','f','e')
#define COMMAND_VOCAB_POTENTIAL_NAME VOCAB4('p','o','t','e')
#define COMMAND_VOCAB_GAIN VOCAB4('g','a','i','n')


class ControlBasisControlLawModule : public RFModule {

    Port handlerPort; // to handle messagees
    
    RunnableControlLaw controlLaw;
    int numControllers;
    bool controlLawRunning;
    Semaphore mutex;

    string sensor, effector, pf, reference;
    double gain;

public:

    ControlBasisControlLawModule() :
        numControllers(0),
        gain(1.0) 
    {
        controlLawRunning = false;
        controlLaw.useTranspose(true);
    }
    
    ~ControlBasisControlLawModule() { }
  

    bool configure(ResourceFinder &rf)  {

        string iCubDir(getenv("ICUB_ROOT"));

        if(rf.check("type")) {
            //resourceType=rf.find("type").asString().c_str();
        } 

        handlerPort.open("/cb/controlModule/rpc:i");
        attach(handlerPort);
        attachTerminal();

        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)  {
     
        bool ok = false;
        bool rec = false; // is the command recognized?

        bool sensor_set = false;
        bool effector_set = false;
        bool pf_set = false;
        bool reference_set = false;

        mutex.wait();

        switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP:
            rec = true;
            {
                reply.addString("help");
                cout << "help" << endl;
                // print help
                ok = true;
            }        
            break;
        case COMMAND_VOCAB_RUN:
            rec = true;
            {
                if (!controlLawRunning) {
                    controlLaw.startAction();
                }
                controlLawRunning = true;
            }
            ok = true;
            break;
        case COMMAND_VOCAB_STOP:
            rec = true;
            {
                if (controlLawRunning) {
                    controlLaw.stopAction();
                }
                controlLawRunning = false;
            }
            ok = true;
            break; 
        case COMMAND_VOCAB_ADD:
            rec = true;
            {
                sensor_set = false;
                effector_set = false;
                pf_set = false;
                reference_set = false;
                gain = 1.0;
                
                cout << "adding controller, size: " << command.size() << endl;
                for(int i=1; i<command.size(); i+=2) {
                    switch(command.get(i).asVocab()) {
                    case COMMAND_VOCAB_SENSOR_NAME:
                        {
                            sensor = command.get(i+1).asString();
                            sensor_set = true;
                        }
                        break;
                    case COMMAND_VOCAB_REFERENCE_NAME:
                        {
                            reference = command.get(i+1).asString();
                            reference_set = true;
                        }
                        break;
                    case COMMAND_VOCAB_POTENTIAL_NAME:
                        {
                            pf = command.get(i+1).asString();
                            pf_set = true;
                        }
                        break;
                    case COMMAND_VOCAB_EFFECTOR_NAME:
                        {
                            effector = command.get(i+1).asString();
                            effector_set = true;
                        }
                        break;
                    case COMMAND_VOCAB_GAIN:
                        {
                            gain = command.get(i+1).asDouble();
                        }
                        break;
                    default:
                        break;                
                    }
                }
                if(sensor_set && effector_set && pf_set) {

                    // if the control law is running, pause it before adding the new one
                    if(controlLawRunning) {
                        controlLaw.stopAction();
                    }
                    if(reference_set) {
                        controlLaw.addController(sensor,reference,pf,effector,gain);
                    } else {
                        controlLaw.addController(sensor,pf,effector,gain);
                    }                   

                    cout << "ADDING CONTROLLER:" << endl;
                    cout << "\tsen: " << sensor << endl;
                    if(reference_set) cout << "\tref: " << reference << endl;
                    cout << "\teff: " << effector << endl;
                    cout << "\tpf: " << pf << endl;                    
                    cout << "\tgain: " << gain << endl;                    

                    // restart the controller cause it was running
                    if(controlLawRunning) {
                        controlLaw.startAction();
                    }

                } else {
                    cout << "not all controller resources set!!" << endl;
                }
            }
            ok = true;
            break;
        case COMMAND_VOCAB_GET:
            rec = true;
            {
                cout << "get" << endl;
                reply.addDouble(0.0);
            }
            ok = true;
            break;
        }
        
        mutex.post();
        
        if (!rec)
            ok = RFModule::respond(command,reply);
        
        if (!ok) {
            reply.clear();
            reply.addVocab(COMMAND_VOCAB_FAILED);
        }
        else
            reply.addVocab(COMMAND_VOCAB_OK);

        return ok;
    }
    

    bool close() {
        cout << "Closing CB Resource Module..." << endl; 
        if (controlLawRunning) {
            controlLaw.stopAction();
            controlLaw.resetControlLaw();   
        }
        controlLawRunning = false;

        handlerPort.close();
        return true;
    }
  
    double getPeriod() {
        return 1;
    }
    
    bool updateModule() {
        Time::delay(1);
        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    ControlBasisControlLawModule mod;

    registerPotentialFunctions();
    registerJacobians();

    ResourceFinder rf;
    //    rf.setDefault("type","yarpConfiguration");

    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);

    cout<<"Starting CB Control Law Module..."<<endl;
    return mod.runModule(rf);

}
