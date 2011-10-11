// Copyright: (C) 2011 IITRBCS
// Authors: Paul Fitzpatrick
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

%module icub

%{
#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)
%}

bool init();

%{
  bool init() {
      YARP_REGISTER_DEVICES(icubmod)
  }
%}
