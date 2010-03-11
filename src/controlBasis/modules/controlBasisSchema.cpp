// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/**
*
@ingroup icub_module
\defgroup icub_controlBasisSchema controlBasisSchema

A module that implements a control basis schema

\section intro_sec Description

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

controlBasisSchema --?

\author Stephen Hart

Copyright (C) 2010 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/controlBasis/modules/controlBasisSchema.cpp.
**/

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>

#include <iostream>
#include <sstream>
#include <string.h>

#include "GeneralizableSchema.h"
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
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_QUIT VOCAB4('q','u','i','t')
#define COMMAND_VOCAB_LOAD VOCAB4('l','o','a','d')

#define COMMAND_VOCAB_GET_SCHEMA_STATE VOCAB4('s','t','a','t')
#define COMMAND_VOCAB_GET_DYNAMIC_STATE VOCAB4('d','y','n','a')

class ControlBasisSchemaModule : public RFModule {

    Port handler_port; // to handle messagees
  
    GeneralizableSchema *schema;
    bool schema_running;
    Semaphore mutex;

    string schema_name;
    int composite_limit;
    int num_controllers;
    int num_sub_schema;
    vector<string> goals;

    vector<ControllerParameters> controllers;
    vector<string> sub_schema;

public:

    ControlBasisSchemaModule() 
        : schema(NULL)
    {
        schema_running = false;
    }
    
    ~ControlBasisSchemaModule() { 
        cout << "ControlBasisSchemaModule destructor..." <<endl;
        if(schema!=NULL) {
            cout << "module destructor deleting schema..." << endl;
            delete schema; 
            schema = NULL;
        }
    }
  

    bool configure(ResourceFinder &rf)  {

        ostringstream st;
        string iCubDir(getenv("ICUB_ROOT"));
        Bottle b;

        schema_name = rf.findGroup("SCHEMA_INFO").check("name",Value(1)).asString();
        composite_limit = rf.findGroup("SCHEMA_INFO").check("composite_limit",Value(1)).asInt();
        b = rf.findGroup("SCHEMA_INFO").findGroup("goal_states");

        goals.clear();
        if(!b.isNull()) {
            for(int i=1; i<b.size(); i++) {
                string s = b.get(i).asString().c_str();
                cout << "found goal[" << i <<"]: " << s << endl;
                goals.push_back(s);                
            }
        }

        // find the number of controllers 
        Bottle b1 = rf.findGroup("CONTROLLER");
        if(b1.size() != 0) {
            num_controllers = b1.size()-1;
        } else {
            num_controllers = 0;
        }

        // find the number of controllers 
        Bottle b2 = rf.findGroup("SCHEMA");
        if(b2.size() != 0) {
            num_sub_schema = b2.size()-1;
        } else {
            num_sub_schema = 0;
        }

        cout << endl;
        cout << "schema name: " << schema_name << endl;
        cout << "composite limit: " << composite_limit << endl;
        cout << "number of controllers: " << num_controllers << endl;
        cout << "number of sub-schema: " << num_sub_schema << endl;
        cout << endl;
    
        // add controller info
        for(int i=0; i<num_controllers; i++) {
            st.str(""); st << (i+1);
            ControllerParameters cp;
            Bottle c = rf.findGroup(b1.get(i+1).asString());

            cp.Sensor = c.check("sensor",Value("")).asString().c_str();
            cp.Reference = c.check("reference",Value("")).asString().c_str();
            cp.PotentialFunction = c.check("potential_function",Value("")).asString().c_str();
            cp.Effector = c.check("effector",Value("")).asString().c_str();
            cp.Gain = c.check("gain",Value("")).asDouble();
            
            //cp.display();
            controllers.push_back(cp);
        }
       
        // add sub schema info
        for(int i=0; i<num_sub_schema; i++) {
            st.str(""); st << (i+1);            
            sub_schema.push_back("");
        }

        schema = new GeneralizableSchema(controllers, sub_schema, composite_limit);
        schema->setName(schema_name);
        schema->load();

        for(int i=0; i<goals.size(); i++) {
            schema->addGoal(goals[i]);
        }

        //        schema->save();

        handler_port.open("/cb/schemaModule/rpc:i");
        attach(handler_port);
        //attachTerminal();

        cout << "SCHEMA MODULE CONFIGURE COMPLETE" << endl;

        return true;
    }

    bool respond(const Bottle &command, Bottle &reply)  {
     
        bool ok = false;
        bool rec = false; // is the command recognized?

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
                if (!schema_running) {
                    schema->startAction();
                }
                schema_running = true;
            }
            ok = true;
            break;
        case COMMAND_VOCAB_STOP:
            rec = true;
            {
                if (schema_running) {
                    schema->stopAction();
                }
                schema_running = false;
            }
            ok = true;
            break; 
        case COMMAND_VOCAB_LOAD:
            rec = true;
            {
                if (schema_running) {
                    schema->stopAction();
                }
                schema_running = false;
                schema->load();                
            }
            ok = true;
            break; 
        case COMMAND_VOCAB_GET:
            rec = true;
            {
                switch (command.get(1).asVocab()) {
                case COMMAND_VOCAB_GET_SCHEMA_STATE:
                    reply.addString(schema->getInternalStateString().c_str());
                    break;
                case COMMAND_VOCAB_GET_DYNAMIC_STATE:
                    reply.addInt(schema->getState());
                    break;
                default:
                    break;
                }
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
        cout << "Closing CB Schema Module..." << endl;     
        if (schema_running) {
            schema->resetSchema();
        }
        schema_running = false;
        /*
        if(schema!=NULL) {
            cout << "module close deleting schema..." << endl;
            delete schema; schema = NULL:
        }
        */

        handler_port.close();
        return true;
    }
  
    double getPeriod() {
        return 1;
    }
    
    bool updateModule() {
        Time::delay(0.1);
        return true;
    }

};

int main(int argc, char *argv[])
{
    Network yarp;
    ControlBasisSchemaModule mod;

    registerPotentialFunctions();
    registerJacobians();

    ResourceFinder rf;

    rf.setDefaultContext("controlBasis/conf");
    rf.setDefaultConfigFile("testSchema.ini");

    rf.setDefault("name","schema_name");
    rf.setDefault("composite_limit","1");

    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);

    cout<<"Starting CB Schema Module..."<<endl;
    return mod.runModule(rf);

}
