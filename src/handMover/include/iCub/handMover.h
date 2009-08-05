 

 // -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/matrix_proxy.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

#ifndef __HANDMOVER__
#define __HANDMOVER__



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



using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


// Using namespace iCub::contrib;

/**
 *
 *
 */
class HandMover : public Module {

private:

  // communication with the arm motors
  yarp::dev::PolyDriver                   armdd;
  yarp::dev::IPositionControl             *armipos;
  yarp::dev::IVelocityControl             *armivel;
  yarp::dev::IEncoders                    *armienc;
  yarp::dev::IAmplifierControl            *armiamp;
  yarp::dev::IPidControl                  *armipid;
  yarp::dev::IControlLimits               *armilim;


  int _numArmAxes;
  bool respond(const Bottle &command,Bottle &reply);

public:
    HandMover();
    ~HandMover();
    
    bool open(Searchable& config);
    bool close();
    bool interruptModule();
    bool updateModule();

  int moveto(double *targetpos);
  int hasarrived(double *target);


};


#endif
