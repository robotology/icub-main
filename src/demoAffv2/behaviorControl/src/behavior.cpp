// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Luis Montesano lmontesano@isr.ist.utl.pt
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <stdio.h>
#include <yarp/os/ResourceFinder.h>
#include "iCub/behavior.h"


enum stateenum {
  FIRSTINIT,
  IDLE,
  ATTENTION,
  AFFORDANCES,
  NUMSTATES
};
char statename[NUMSTATES][20]={"firstinit","idle","attention","affordances"};



//added here due to version problem
#define VOCAB_FAILED VOCAB4('f','a','i','l')
#define VOCAB_OK VOCAB2('o','k')
#define VOCAB_QUIT VOCAB4('q','u','i','t')
//#define VOCAB_STOP VOCAB4('s','t','o','p')
#define VOCAB_CONT VOCAB4('c','o','n','t')


Behavior::Behavior(){
  state=FIRSTINIT;
  printf("After constructor\n");
}

Behavior::~Behavior(){ 

  port_gaze.close();
  port_aff_in.close();
  port_aff_out.close();

}

bool Behavior::respond(const Bottle &command,Bottle &reply){
    bool ok = false; // command executed successfully
    
    switch (command.get(0).asVocab()) {
    case VOCAB_QUIT:
        reply.addVocab(VOCAB_OK);
        ok = true;
        OK_MSG = 1;
        break;
    case VOCAB_STOP:
        reply.addVocab(VOCAB_OK);
        ok = true;
        next_state=IDLE;
        OK_MSG = 1;
        break;
    case VOCAB_CONT:
        reply.addVocab(VOCAB_OK);
        ok = true;
        next_state=ATTENTION;
        OK_MSG = 1;
        break;
    default:
        printf("VOCAB default\n ");
        reply.addVocab(VOCAB_FAILED);
    }

    ok = RFModule::respond(command,reply); // will add message 'not recognized' if not recognized
    
    return ok;
}

/*bool Behavior::configure(ResourceFinder &rf) {

}
*/

bool Behavior::configure(ResourceFinder& rf){
    bool ok=true;

    cg_az_max = rf.check("cg_az_max",yarp::os::Value(0.0)).asDouble();
    cg_az_min = rf.check("cg_az_min",yarp::os::Value(0.0)).asDouble();
    cg_el_max = rf.check("cg_el_max",yarp::os::Value(0.0)).asDouble();
    cg_el_min = rf.check("cg_el_min",yarp::os::Value(0.0)).asDouble();
    min_attention_time = rf.check("min_attention_time",yarp::os::Value(0.0)).asDouble();


    cout << "cg_az_max: " << cg_az_max << endl;
    cout << "cg_az_min: " << cg_az_min << endl;
    cout << "cg_el_max: " << cg_el_max << endl;
    cout << "cg_el_min: " << cg_el_min << endl;
    cout << "min_attention_time: " << min_attention_time << endl;
    
    // Ports
    // open camshiftplus ports: data and sync
    ok &= port_gaze.open("/demoAffv2/behavior/gaze");
    ok &= port_aff_in.open("/demoAffv2/behavior/aff_in");
	ok &= port_aff_out.open("/demoAffv2/behavior/aff_out");
    ok &= remoteAtt.open("/demoAffv2/behavior/att");
    ok &= port_in.open("/demoAffv2/behavior/in");
    return ok;
}


bool Behavior::close(){
    
    port_gaze.close();
    port_aff_in.close();
	port_aff_out.close();
    port_in.close();
    
    return true;
}

bool Behavior::interruptModule(){


  port_gaze.interrupt();
  port_aff_in.interrupt();
  port_aff_out.interrupt();
  port_in.close();
  
    return true;
}

bool Behavior::updateModule()
{
	printf(".");fflush(stdout); 
  
	cout << "state: " << state << statename[state] << endl;

	Bottle *input=port_in.read(false);
	if(input != NULL)
	{
		string cmd=input->get(0).asString().c_str();

		if (cmd=="att")
		{
			state=ATTENTION;
                        attention_time = yarp::os::Time::now();
			remoteAtt.setInhibitOutput(false);
		}
		else if (cmd=="aff")
		{
			state=AFFORDANCES;
                        remoteAtt.setInhibitOutput(true);
			// Tell affordances to start
			Bottle& output = port_aff_out.prepare();
			output.clear();
			output.addInt(1);
			port_aff_out.write();
		}
		else 
		{
			cout << "Message ignored: " << cmd << endl;
		}
		cout << "state change: " << state << statename[state] << endl;
	}


	switch (state)
	{

		case FIRSTINIT: 
		{
			state=ATTENTION;
                        attention_time = yarp::os::Time::now();
		}
		break;

		case ATTENTION:
		{
			// Read data from Gaze Control
			yarp::os::Time::delay(0.04);
			Bottle *input_obj=port_gaze.read(false);
      
			if (input_obj!=NULL)
			{
				int cg_state;
				float cg_az, cg_el;

				if (input_obj->size()==5) 
				{
					cg_state = input_obj->get(0).asInt();
					cg_az = input_obj->get(3).asDouble();
					cg_el = input_obj->get(4).asDouble();
                                        cout << "cg_state: " << cg_state << endl;
                                        cout << "cg_az: " << cg_az << endl;
                                        cout << "cg_el: " << cg_el << endl;

				}
				else 
				{
					// generate an error and quit
				}

				double currTime = yarp::os::Time::now();
				if(     (currTime - attention_time > min_attention_time ) &&
                                        (cg_state == 5 || cg_state == 4) && 
					cg_az < cg_az_max && 
					cg_az > cg_az_min && 
					cg_el < cg_el_max && 
					cg_el > cg_el_min   ) 
				{
	
					// Send inhibition to the attention system
					remoteAtt.setInhibitOutput(true);

					// Switch attention
					state=AFFORDANCES;

					// Tell affordances to start
					Bottle& output = port_aff_out.prepare();
					output.clear();
					output.addInt(1);
					port_aff_out.write();
				}
			}
		}
		break;

		case IDLE:
		{
			yarp::os::Time::delay(0.04);      
		}
		break;

		case AFFORDANCES:
		{
			// Read data from Affordances
			yarp::os::Time::delay(0.04);
			Bottle *input_obj=port_aff_in.read(false);
      
			if (input_obj!=NULL) 
			{
				int aff_state;
				aff_state = input_obj->get(0).asInt();
				if (aff_state == 1)
				{
					remoteAtt.setInhibitOutput(false);
					state=ATTENTION;
                                        attention_time = yarp::os::Time::now();
				}
			}
		}
		break;
	
		default:
			cout << "Strange state moving to IDLE" << endl;
	}
  
	if (OK_MSG == 1) 
	{
		state=next_state;
		OK_MSG == 0;
	}
  
	return true;
}


