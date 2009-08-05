 

 // -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 1876 Luis Montesano
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __DEMOAFFMODULE__
#define __DEMOAFFMODULE__

#ifndef M_PI_2
#define M_PI_2	((float)(asin(1.0)))
#endif
#ifndef M_PI
#define M_PI	((float)(2*M_PI_2))
#endif

 // std
#include <stdio.h>
#include <string>
#include <iostream>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Module.h>


#include "affordances.h"
#include "processobjdata.h"

#define MAXNODES 10
#define MAXLOCATIONS 3
#define NUMEFF 2

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

namespace iCub {
     namespace contrib {
         class demoAff;
     }
}

using namespace iCub::contrib;

/**
 *
 * affordance demo Module class
 *
 * \see icub_demoaff
 *
 */
class DemoAff : public Module {

private:

  // ports
  // Camshiftplus provides only the effects
  BufferedPort<Bottle> port_eff;
  BufferedPort<Bottle> port_sync;
  
  // object_segmentator
  BufferedPort<Bottle> port_askobj;
  BufferedPort<Bottle> port_descriptor;

  // motion primitives
  BufferedPort<Bottle> port_primitives;

  // control gaze
  BufferedPort<Bottle> port_gaze_in;
  BufferedPort<Bottle> port_gaze_out;

  // behavior
  BufferedPort<Bottle> port_behavior_in;
  BufferedPort<Bottle> port_behavior_out;

  // Output info
  BufferedPort<Bottle> port_output;

  // Expressions
  BufferedPort<Bottle> port_emotions;

  
  int state; // Current state of the FSM

  // Affordance variables
  Affordances affBN;
  int _numNodes;
  int _mat[MAXNODES*MAXNODES];
  int _sizes[MAXNODES];

  
  // objects data
  processobjdata processdata;

  const int max_obj = 10;

  // Demonstration data
  int democolor, demoshape;
  int demoeffects[NUMEFF];


  bool InitAff(Searchable& config);
  bool restartTracker(double *pos, double width);

  int numObservedObj;
  int substate;
  int colorobj;
  int shapeobj;

  int selectedaction;
  int selectedobj;

  double objpos[max_obj][2];

  int shape[max_obj];
  int color[max_obj];
  int size[max_obj];

   
  int OK_MSG;
 
  bool respond(const Bottle &command,Bottle &reply);
  bool readDetectorInput(Bottle *Input, string name, double *cx, double *cy);
  bool getObjDesc(Bootle * input_obj);

public:

    DemoAff();
    virtual ~DemoAff();
    
    /** Passes config on to iCub::contrib::CalibTool */
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

public:

};


#endif
