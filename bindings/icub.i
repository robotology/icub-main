// Copyright: (C) 2011 IITRBCS
// Authors: Paul Fitzpatrick
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

%module icub

%include "std_string.i"
%include "std_vector.i"

%import "yarp.i"

%{
#include <yarp/dev/Drivers.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
using namespace yarp::os;
using namespace yarp::sig;
#include "cartesianController/ClientCartesianController.h"
#include "gazeController/ClientGazeController.h"
%}

%include "cartesianController/ClientCartesianController.h"
%include "gazeController/ClientGazeController.h"

bool init();

%{
  bool init() {
      return true;
  }
%}

