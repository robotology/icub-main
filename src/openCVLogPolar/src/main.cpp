// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/LogPolarModule.h>



int main(int argc, char *argv[]) {
    
    
    //initialise Yarp Network
    Network yarp;
   
    // Create and run our module
    LogPolarModule module;

    // Get command line options
    Property options;
    if(argc<2){
        //there are no parameters
        // litte help on how to use
        printf("______ HELP ________ \n");
        printf(" \n");
        printf("USER COMMANDS: \n");
        printf("--mode (SIMULATION,FORWARD,INVERSE): selects what this module operates \n");
        printf("--name (XXXX): defines the name of this module \n");
        printf(" \n");
        printf(" \n");
        //start of the default mode
        printf("No commands, starting of the default mode ................... \n");
        options.put("mode","FORWARD");
        options.put("name","/rea/logPolar");
    }
    else{
        //estracts the command from command line
        options.fromCommand(argc,argv);
    }
    module.setOptions(options);

    return module.runModule(argc,argv);

    //MyModule module;
    //module.setName("/default/name");
    //return module.runModule(argc,argv);
}
