// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/cRefinementModule.h>
#include <yarp/os/all.h>
#include <string>

#include <stdio.h>
#include <iostream>

//primatevision include
#include "multiclass.h" 

using namespace yarp::os;
using namespace std;

/*namespace iCub {
  namespace contrib {
    namespace primateVision {
		namespace MultiClass;
	}
  }
}*/

int main(int argc, char *argv[]) {
    //initialise Yarp Network
	Network yarp;
	// Create and run our module
	cRefinementModule module;
	module.setName("/rea/cRefinement");

	//get config file parameters:
	//std::string fname=string(getenv("ICUB_DIR"))+string("/../iCub/src/primateVision/utils/multiclass/testmulticlass/multiclass.cfg");
	std::string fname;

	for (int i=1;i<argc;i++) {
		if ((strcmp(argv[i],"--configfile")==0)||(strcmp(argv[i],"-c")==0)) {
			fname = argv[++i];
		}
		if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
			cout <<"usage: "<<argv[0]<<" [-h/--help] [-c/--configfile file] " <<endl;
		exit(0);
		}
	}	
	Property prop;
	prop.fromConfigFile(fname.c_str());
	struct MultiClass::Parameters params;

	params.iter_max = prop.findGroup("MRF").find("MAX_ITERATIONS").asInt();
	params.randomize_every_iteration = prop.findGroup("MRF").find("RANDOMIZE_EVERY_ITER").asInt();
  
	params.smoothness_penalty = prop.findGroup("MRF").find("SMOOTHNESS_PENALTY").asInt();
	params.smoothness_3sigmaon2 = prop.findGroup("MRF").find("SMOOTHNESS_3SIGMAON2").asInt();
	params.data_penalty = prop.findGroup("MRF").find("DATA_PENALTY").asInt();

	// Get command line options
	/*Property options;
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
		//start of the DEFAULT MODE
		printf("No commands, starting of the default mode ................... \n");
		options.put("mode","FORWARD");
		options.put("name","/rea/logPolar");
	}
	else{
		//estracts the command from command line
		options.fromCommand(argc,argv);
	}*/


	


	//module.setOptions(options);
	module.setProperties(params);

	Property options;
    options.fromCommand(argc, argv);
	module.istantiateThread(options);

    return module.runModule(argc,argv);

    //MyModule module;
    //module.setName("/default/name");
    //return module.runModule(argc,argv);
}
