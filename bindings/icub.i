// Copyright: (C) 2011 IITRBCS
// Authors: Paul Fitzpatrick
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

%module icub

%include "std_string.i"
%include "std_vector.i"
%include "std_map.i"

%{
#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)
%}

%init %{
  YARP_REGISTER_DEVICES(icubmod)
%}


