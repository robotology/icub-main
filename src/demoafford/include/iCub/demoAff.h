 

 // -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __DEMOAFFMODULE__
#define __DEMOAFFMODULE__

//#define BALTAZAR 

#undef BALTAZAR
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
#ifdef BALTAZAR
#include "grasp.h"
#else
#endif

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
    // Camshiftplus provides vision based object information
    BufferedPort<Bottle> port_obj;
    BufferedPort<Bottle> port_sync;
  //BufferedPort<Bottle> port_hand;
  //BufferedPort<Bottle> port_hand_sync;
 
	BufferedPort<Bottle> port_detector;

    int state; // Current state of the FSM
    Affordances affBN;
    int _numNodes;
    int _mat[MAXNODES*MAXNODES];
    int _sizes[MAXNODES];
    int democolor, demoshape;
    int demoeffects[NUMEFF];
    processobjdata processdata;

  bool InitAff(Searchable& config);

  bool restartTracker(double *pos, double width);

#ifdef BALTAZAR
  int mycheckmotion( int axis, double target);
  int posmove(double *pos);

  // communication with the head motors
  yarp::dev::PolyDriver                           dd;
  yarp::dev::IPositionControl                     *ipos;
  yarp::dev::IVelocityControl                     *ivel;
  yarp::dev::IEncoders                            *ienc;
  yarp::dev::IAmplifierControl            *iamp;
  yarp::dev::IPidControl                          *ipid;
  yarp::dev::IControlLimits                       *ilim;
#else
    // head interface
  yarp::dev::PolyDriver                           dd;
  yarp::dev::IPositionControl                     *ipos;
  yarp::dev::IVelocityControl                     *ivel;
  yarp::dev::IEncoders                            *ienc;
    yarp::dev::IAmplifierControl                  *iamp;
  yarp::dev::IPidControl                          *ipid;
  yarp::dev::IControlLimits                       *ilim;

    // arm interface
  yarp::dev::IPositionControl                     *a_ipos;
  yarp::dev::IVelocityControl                     *a_ivel;
  yarp::dev::IEncoders                            *a_ienc;
  yarp::dev::IAmplifierControl                    *a_iamp;
  yarp::dev::IPidControl                          *a_ipid;
  yarp::dev::IControlLimits                       *a_ilim;

  yarp::dev::PolyDriver                           ar_dd;
  yarp::dev::IPositionControl                     *ar_ipos;
  yarp::dev::IVelocityControl                     *ar_ivel;
  yarp::dev::IEncoders                            *ar_ienc;
  yarp::dev::IAmplifierControl                    *ar_iamp;
  yarp::dev::IPidControl                          *ar_ipid;
  yarp::dev::IControlLimits                       *ar_ilim;

  yarp::dev::PolyDriver                           al_dd;
  yarp::dev::IPositionControl                     *al_ipos;
  yarp::dev::IVelocityControl                     *al_ivel;
  yarp::dev::IEncoders                            *al_ienc;
  yarp::dev::IAmplifierControl                    *al_iamp;
  yarp::dev::IPidControl                          *al_ipid;
  yarp::dev::IControlLimits                       *al_ilim;

  bool  InitArm();
  bool  InitHead();
  bool selectArm(char sarm);
#endif

  int _numHeadAxes;
  int _numArmAxes;
  int _numLArmAxes;
  int _numRArmAxes;
  double framerate;
  int numObservedObj;
  int substate;
  int colorobj[2];
  int shapeobj[2];
  int selectedaction;
  int selectedobj;
  double height;
  double objpos[2];

  int shape;
  int color;

  double headlocations[MAXLOCATIONS][4];
  int detectedlocations;
  // Baltazar arm class

  
#ifdef BALTAZAR
  ArmSkills arm;
  int velmove(double v1, double v2, double v3, double v4);
#else
  bool allMotionDone(yarp::dev::IPositionControl *tmp_ipos, int naxes);
  bool allSpeedZero(yarp::dev::IEncoders *tmp_enc, int naxes);
  bool encNoMotion(yarp::dev::IEncoders *tmp_enc, int naxes);
  bool encNoMotion(yarp::dev::IPositionControl *tmp_pos, int foo);
  bool encNoMotion(yarp::dev::IPositionControl *tmp_pos, int foo, char *msg);
#endif

  int OK_MSG;
  int nMoves;

  bool respond(const Bottle &command,Bottle &reply);
  bool readDetectorInput(Bottle *Input, string name, double *cx, double *cy);
public:

    DemoAff();
    virtual ~DemoAff();
    
    /** Passes config on to iCub::contrib::CalibTool */
    virtual bool open(Searchable& config);
    virtual bool close();
    virtual bool interruptModule();
    virtual bool updateModule();

public:
	// performs servoing until closer than err, returns the final position in headpos, reentrant

	float headServo(double errx, double erry, double* headvel, float gain);
	int saveLocation(double* pos, int locnumber);
};


#endif
