 

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
#include <map>
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
#define affActionPrimitivesLayer affActionPrimitivesLayer2

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace vislab::math;

using namespace ctrl;
using namespace actions;



/**
 *
 * affordance demo Module class
 *
 * \see icub_demoaff
 *
 */





class DemoAff : public RFModule {

private:

  // Functions to command controlGaze2
  void controlGazeSaccadeAbsolute(double az, double el);
  void controlGazeSaccadePixel(double x, double y);
  void controlGazePursuitPixel(double x, double y);
  // TODO: sorry for squeezing everything to the top :]
  // Hand orientations
  static std::map<std::string, yarp::sig::Matrix> palmOrientations;
  static std::map<std::string, yarp::sig::Matrix> computePalmOrientations();

  // Emotion interface
  bool emotionCtrl(const yarp::os::ConstString cmd);

  std::string armToBeUsed;

  // ports
  // Camshiftplus provides only the effects
  BufferedPort<Bottle> port_eff;
  BufferedPort<Bottle> port_sync;
  
  // object_segmentator
  BufferedPort<Bottle> port_descriptor;
  BufferedPort<Bottle> port_track_info;


  // control gaze
  BufferedPort<Vector> port_gazepos_out;
  BufferedPort<Vector> port_gazevel_out;

  // behavior
  BufferedPort<Bottle> port_behavior_in;
  BufferedPort<Bottle> port_behavior_out;

  // Output info
  BufferedPort<Bottle> port_output;

  // Expressions
  /*BufferedPort<Bottle>*/ Port port_emotions;

  // Head and torso confgiruations
  BufferedPort<Bottle> port_head_state;
  BufferedPort<Bottle> port_torso_state;

  
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

  Vector objposreach;

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

  actions::affActionPrimitivesLayer *actionL;
  actions::affActionPrimitivesLayer *actionR;
  actions::affActionPrimitivesLayer *action;
  yarp::dev::ICartesianControl *cartIF;
  BufferedPort<Bottle>       inPort;
  Port                       rpcPort;
  
  Vector graspOrienL, graspOrienR;
  Vector graspDispL,  graspDispR;
  Vector graspReliefL, graspReliefR;
  Vector dOffsL,      dOffsR;
  Vector dLiftL,      dLiftR;
  Vector home_xL,     home_xR;
  Vector home_oL,     home_oR;
  
  Vector *graspOrien;
  Vector *graspDisp;
  Vector *graspRelief;
  Vector *dOffs;
  Vector *dLift;
  Vector *home_x;
  Vector *home_o;
  
  bool openPorts;

  // affActionPrimitive methods
  bool InitAffPrimitives();
  bool configureAffPrimitives(Searchable &config, 
			      yarp::os::ConstString handSeqFileLeft,
                              yarp::os::ConstString handSeqFileRight, 
			      string name);
  void useArm(const int arm);
  void getArmDependentOptions(Bottle &b, Vector &_gOrien, Vector &_gDisp, Vector &_gRelief,
			      Vector &_dOffs, Vector &_dLift, Vector &_home_x,
			      Vector &_home_o);


  // Eye2World member variables
  std::map<const std::string, yarp::os::Property*> cameras;
  std::map<const std::string, EyeTableProjection*> projections;
  double zOffset;
  std::string usedEye;
  yarp::sig::Vector object3d;
  yarp::sig::Vector tableTop;
  
  // Eye2World member methods
  bool configureEye2World(yarp::os::ConstString calibrationFilename,yarp::os::ConstString tableConfiguration);  


  // position configurations
  double headPosActAz,headPosActEl;
  double torsoPosActPitch, torsoPosActRoll, torsoPosActYaw;

  double headPosObsAz,headPosObsEl;
  double torsoPosObsPitch, torsoPosObsRoll, torsoPosObsYaw;

  double headActPos[6];
  double torsoActPos[3];

  double headObsPos[6];
  double torsoObsPos[3];

  Vector rotVec;

  // head interface
  // This is not needed when using controlGaze2
  /*
  yarp::dev::PolyDriver                           dd;
  yarp::dev::IPositionControl                     *ipos;
  yarp::dev::IVelocityControl                     *ivel;
  yarp::dev::IEncoders                            *ienc;
  yarp::dev::IAmplifierControl                  *iamp;
  yarp::dev::IPidControl                          *ipid;
  yarp::dev::IControlLimits                       *ilim;
  */

  // Torso interface
  yarp::dev::PolyDriver                           t_dd;
  yarp::dev::IPositionControl                     *t_ipos;
  yarp::dev::IVelocityControl                     *t_ivel;
  yarp::dev::IEncoders                            *t_ienc;
  yarp::dev::IAmplifierControl                    *t_iamp;
  yarp::dev::IPidControl                          *t_ipid;
  yarp::dev::IControlLimits                       *t_ilim;

  bool InitTorso();
  int _numTorsoAxes;
  // This is not needed when using controlGaze2
  //bool InitHead();
  int _numHeadAxes;


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
