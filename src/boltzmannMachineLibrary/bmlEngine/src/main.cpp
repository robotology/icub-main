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

	

    BMLEngine module;
    module.setName("/rea/BMLEngine");
	bool ret=module.runModule(argc,argv);
	if(!ret)
		module.close();
	return ret;
	//return 0;
}
