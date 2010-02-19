// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/MachineBoltzmann.h>
#include <iCub/BMLEngine.h>
#include <iostream>
//#include <conio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

using namespace std;


int main(int argc, char *argv[]) {
    
    
    //initialise Yarp Network
    Network yarp;
    
    // Get command line options
    //Property options;
    //options.fromCommand(argc,argv);
    //module.setOptions(options);
    
    std::string fname;

    if(argc<2){
		//there are no parameters
		// litte help on how to use
		printf("______ HELP ________ \n");
		printf(" \n");
		printf("USER COMMANDS: \n");
		printf("--name (XXXX): defines the name of the module and the rootname of every port \n");
        printf("e.g bmlEngine --name /bmlEngine \n");
		printf(" \n");
		printf(" \n");
		return 0;
	}
	else{
		//estracts the command from command line
        for (int i=1;i<argc;i++) {
		    if ((strcmp(argv[i],"--name")==0)||(strcmp(argv[i],"-n")==0)) {
			    fname = argv[++i];
			    printf("file name:%s \n",fname.c_str());
		    }
		    if ((strcmp(argv[i],"--help")==0)||(strcmp(argv[i],"-h")==0)) {
			    printf("______ HELP ________ \n");
		        printf(" \n");
		        printf("USER COMMANDS: \n");
		        printf("--name (XXXX): defines the name of the module and the rootname of every port \n");
                printf("e.g bmlEngine --name /bmlEngine \n");
		        printf(" \n");
		        printf(" \n");
                
		        return 0;
		    }
		    //options.fromCommand(argc,argv);
	    }
    }
    

    BMLEngine module;
    module.setName(fname.c_str());
    bool ret=module.runModule(argc,argv);
    if(!ret)
        module.close();
    return ret;
    //return 0;
}
