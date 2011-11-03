// Copyright: (C) 2011 IITRBCS
// Authors: Paul Fitzpatrick
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

%module icub

%include "std_string.i"
%include "std_vector.i"

%import "yarp.i"

%{
#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)
#include "cartesianController/ClientCartesianController.h"
#include "gazeController/ClientGazeController.h"
%}

%include "cartesianController/ClientCartesianController.h"
%include "gazeController/ClientGazeController.h"

bool init();

%{
  bool init() {
      YARP_REGISTER_DEVICES(icubmod)
  }
%}

