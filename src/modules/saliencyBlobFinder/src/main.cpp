// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/saliencyBlobFinderModule.h>

#include <iostream>
#include <string.h>
using namespace std;



int main(int argc, char *argv[]) {
        
    //initialise Yarp Network
    Network yarp;
    Time::turboBoost();

   /* create your module */

   saliencyBlobFinderModule module; 

   /* prepare and configure the resource finder */

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("blobFinderLeft.ini");  //overridden by --from parameter
   rf.setDefaultContext("attentionMechanism/conf");   //overridden by --context parameter
   rf.configure("ICUB_ROOT", argc, argv);
 
   /* run the module: runModule() calls configure first and, if successful, it then runs */
   module.runModule(rf);

   return 0;
}