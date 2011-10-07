// Copyright: (C) 2011 IITRBCS
// Authors: Paul Fitzpatrick
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

%module icub

%{
#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)
%}

#if defined(SWIGCSHARP)

// csharp initialization seems to work differently to other language bindings
%pragma(csharp) imclasscode=%{ 
   protected class SWIGCustomInit { 
     static SWIGCustomInit() { 
       YARP_REGISTER_DEVICES(icubmod)
     } 
   } 
   static protected SWIGCustomInit swigCustomInit = new SWIGCustomInit(); 
%} 

#else

%init %{
  YARP_REGISTER_DEVICES(icubmod)
%}

#endif

