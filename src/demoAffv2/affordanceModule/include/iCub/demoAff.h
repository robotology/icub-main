 

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


#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/affActionPrimitives.h>
#include <iCub/vislab/EyeTableProjection.h>

#include "affordances.h"
#include "processobjdata.h"
#include "objSupport.h"

#define MAXNODES 10
#define MAXLOCATIONS 3
#define NUMEFF 2

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace vislab::math;


/**
 *
 * affordance demo Module class
 *
 * \see icub_demoaff
 *
 */





class DemoAff : public RFModule {

private:

  // ports
  // Camshiftplus provides only the effects
  BufferedPort<Bottle> port_eff;
  BufferedPort<Bottle> port_sync;
  
  // object_segmentator
  BufferedPort<Bottle> port_descriptor;
  BufferedPort<Bottle> port_track_info;

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

  static const int max_obj = 10;

  // Demonstration data
  int democolor, demoshape, demosize;
  int demoeffects[NUMEFF];


  bool InitAff(Searchable& config);
  bool restartTracker(TrackInfo obj);

  int numObservedObj;
  int substate;
  int colorobj;
  int shapeobj;

  int selectedaction;
  int selectedobj;

  double objpos[max_obj][2];

  int shapeObj[max_obj];
  int colorObj[max_obj];
  int sizeObj[max_obj];

  BlobInfo   *objDescTable;
  TrackInfo *trackDescTable;              
  int       numObjs;
  int       maxObjects; 
  int       numBlobs;
  int       numTracks;

  double table_az, table_el;

  int OK_MSG;
 
  bool respond(const Bottle &command,Bottle &reply);
  bool readDetectorInput(Bottle *Input, string name, double *cx, double *cy);

  bool getObjInfo();
  bool getBlobInfo(const Bottle *msg);
  bool getTrackerInfo(const Bottle *msg);


  // Deprecated functions
  bool readEffect();
  bool getObjDesc(Bottle * input_obj);

protected:

  // affActionPrimitive member variable
  string partUsed;

  affActionPrimitivesLayer1 *actionL;
  affActionPrimitivesLayer1 *actionR;
  affActionPrimitivesLayer1 *action;
  BufferedPort<Bottle>       inPort;
  Port                       rpcPort;
  
  Vector graspOrienL, graspOrienR;
  Vector graspDispL,  graspDispR;
  Vector dOffsL,      dOffsR;
  Vector dLiftL,      dLiftR;
  Vector home_xL,     home_xR;
  Vector home_oL,     home_oR;
  
  Vector *graspOrien;
  Vector *graspDisp;
  Vector *dOffs;
  Vector *dLift;
  Vector *home_x;
  Vector *home_o;
  
  bool openPorts;

  // affActionPrimitive methods
  bool InitAffPrimitives();
  bool configureAffPrimitives(Searchable &config, 
			      yarp::os::ConstString handSeqFile, 
			      string name);
  void useArm(const int arm);
  void getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp,
			      Vector &_dOffs, Vector &_dLift, Vector &_home_x,
			      Vector &_home_o);


  // Eye2World member variables
  std::map<const std::string, yarp::os::Property*> cameras;
  std::map<const std::string, EyeTableProjection*> projections;

  // Eye2World member methods
  bool configureEye2World(yarp::os::ConstString calibrationFilename);  

public:

  DemoAff();
  virtual ~DemoAff();
  
  /** Passes config on to iCub::contrib::CalibTool */
  virtual bool configure(ResourceFinder &rf);
  virtual bool close();
  virtual bool interruptModule();
  virtual bool updateModule();

};



#endif
