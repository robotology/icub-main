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
    // Create and run our module
    selectiveAttentionModule module;


    std::string fname;
 
    //property from command line
    Property options;
    //property from configuration line
    Property prop;

	if(argc<2){
		//there are no parameters
		// litte help on how to use
		printf("______ HELP ________ \n");
		printf(" \n");
		printf("USER COMMANDS: \n");
		printf("--file (XXXX): defines the name of the configfile to look into parameters \n");
        printf("e.g.: selectiveAttentionProcessor --file selectiveAttentionProcessor.ini \n");
		printf(" \n");
		printf(" \n");
		return 0;
	}
	else{
		//estracts the command from command line
        for (int i=1;i<argc;i++) {
		    if ((strcmp(argv[i],"--file")==0)||(strcmp(argv[i],"-c")==0)) {
			    fname = argv[++i];
			    printf("file name:%s \n",fname.c_str());
		    }
		    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
			    cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--file file] " <<endl;
		        return 0;
		    }
		    options.fromCommand(argc,argv);
	    }
    }

    ResourceFinder* _the_finder = new ResourceFinder;
	ResourceFinder& finder = *_the_finder;
    finder.setVerbose();
    //---
    finder.setDefaultConfigFile("attentionMechanism.ini");
    finder.setDefaultContext("attentionMechanism");
    //---
    finder.configure("ICUB_ROOT", argc, argv); 
	ConstString location = finder.findFile(fname.c_str());
    
    if (location=="") {
        printf("config file not found: \n");
    }
    else
	    printf("file found in location: %s \n",location.c_str() );

	prop.fromConfigFile(location.c_str());
    module.setOptions(prop);

    
    return module.runModule(argc,argv);

}