// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/selectiveAttentionModule.h>

#include <iostream>

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[]) {
    //initialise Yarp Network
    Network yarp;
    Time::turboBoost();
    selectiveAttentionModule module; 

    /* prepare and configure the resource finder */

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("selectiveAttentionLeft.ini");  //overridden by --from parameter
    rf.setDefaultContext("attentionMechanism/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

}