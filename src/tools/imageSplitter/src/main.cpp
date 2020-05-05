// Copyright (C) 2016 Istituto Italiano di Tecnologia - iCub Facility
// Author: Alberto Cardellino <alberto.cardellino@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

/**
@ingroup icub_tools

Split a single image with 2 frames in it into 2 images left/right

Parameters
\code
  align horizontal / vertical  :  input images are coupled on the horizontal / vertical way  -- default horizontal
\endcode
*/ 

#include <yarp/dev/Drivers.h>
#include <imageSplitter.h>


int main(int argc, char * argv[])
{
   yarp::os::Network yarp;
   ImageSplitter imageSplitter;
   yarp::os::ResourceFinder rf;
   rf.configure(argc, argv);

   imageSplitter.runModule(rf);
   return 0;
}
