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
  port_aff.close();

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

    ok = Module::respond(command,reply); // will add message 'not recognized' if not recognized

    return ok;
}

/*bool Behavior::configure(ResourceFinder &rf) {

}
*/

bool Behavior::open(Searchable& config){
   
	// locate configuration file
    ResourceFinder rf;        
	if (config.check("context")){
        rf.setDefaultContext(config.find("context").asString());
	}
	if (config.check("from")){
        rf.setDefaultConfigFile(config.find("from").asString());
	}
	else if (config.check("file")){
        rf.setDefaultConfigFile(config.find("file").asString());
	}
	else{
        rf.setDefaultConfigFile("icubDemoAffBehavior.ini");
	}
    rf.configure("ICUB_ROOT",0,NULL);
	Property prop(rf.toString());
	prop.fromString(config.toString(), false);
	prop.setMonitor(config.getMonitor());

    bool ok=true;

  ConstString str = prop.check("motorboard","/baltaHead","Name of the control board").asString();

  // used a reference here - otherwise check for null doesn't work
  // (cannot copy a null bottle)
  //framerate = config.check("FrameRate", yarp::os::Value(20.0), "FrameRate").asDouble();


  // Ports
  // open camshiftplus ports: data and sync
  ok &= port_gaze.open("/demoAffv2/behavior/gaze");
  ok &= port_aff.open("/demoAffv2/behavior/aff");
  ok &= remoteAtt.open("/demoAffv2/behavior/att");
  return ok;
}


bool Behavior::close(){

  port_gaze.close();
  port_aff.close();

  return true;
}

bool Behavior::interruptModule(){


  port_gaze.interrupt();
  port_aff.interrupt();

    return true;
}

bool Behavior::updateModule(){

  printf(".");fflush(stdout);

  
  
  cout << "state: " << state << statename[state] << endl;
  switch (state){
  case FIRSTINIT: 
    {
      state=ATTENTION;
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

	  if (input_obj->size()==3) {
	    cg_state= input_obj->get(0).asInt();
	    cg_az = input_obj->get(1).asDouble();
	    cg_el = input_obj->get(2).asDouble();
	  }
	  else {
	    // generate an error and quit
	  }

	  if (cg_state==5 && cg_az<cg_az_max && cg_az>cg_az_min && 
	      cg_el<cg_el_max && cg_el>cg_el_min) {
	
	    // Send inhibition to the attention system
	    remoteAtt.setInhibitOutput(true);

	    // Switch attention
	    state=AFFORDANCES;
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
      // Read data from Gaze Control
      yarp::os::Time::delay(0.04);
      Bottle *input_obj=port_aff.read(false);
      
      if (input_obj!=NULL) {
	int aff_state;
	aff_state= input_obj->get(0).asInt();
	if (aff_state==1) {
	  remoteAtt.setInhibitOutput(false);
	  state=ATTENTION;
	}
      }
    }
      break;
      default:
	cout << "Strange state moving to IDLE" << endl;

  }
  
  if (OK_MSG==1) {
    state=next_state;
    OK_MSG==0;
  }
  
  return true;
}


