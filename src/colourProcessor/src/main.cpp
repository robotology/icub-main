// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/colourProcessorModule.h>



int main(int argc, char *argv[]) {
        
    //initialise Yarp Network
    Network yarp;
   
    // Create and run our module
    colourProcessorModule module;

    Property options;
	if(argc<2){
		//there are no parameters
		// litte help on how to use
		printf("______ HELP ________ \n");
		printf(" \n");
		printf("USER COMMANDS: \n");
		printf("--name (XXXX): defines the name of this module \n");
		printf(" \n");
		printf(" \n");
		return 0;
	}
	else{
		//estracts the command from command line
		options.fromCommand(argc,argv);
	}


    return module.runModule(argc,argv);

}