// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/MachineBoltzmann.h>
#include <iCub/BIControlGazeEngine.h>
#include <iostream>
#include <conio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

using namespace std;

//BufferedPort<yarp::sig::Vector> *targetLeft=0; 
//BufferedPort<yarp::sig::Vector> *targetRight=0;
//Port *trackingPort=0;
//BufferedPort<yarp::sig::Vector> *inertial=0;


int main(int argc, char *argv[]) {

	
	//initialise Yarp Network
	Network yarp;
	Time::turboBoost();

	// Get command line options
	//Property options;
	//options.fromCommand(argc,argv);
	//module.setOptions(options);

    BIControlGazeEngine module;
	Terminee terminee("/rea/BIControlGazeEngine/quit");
	if (!terminee.isOk()) {
        printf("Failed to create proper quit socket\n");
        return 1;
    }

    // just list the devices if no argument given
    if (argc <= 2) {
        printf("You can call %s like this:\n", argv[0]);
        printf("   %s --file FILENAME\n", argv[0]);
        printf("For example:\n");
        printf("   %s --file icub_yaheadcontrol.ini\n", argv[0]);
        
        return 0;
    }

	// get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("file")) {
        ACE_OS::printf("Missing the --file option\n");
        return 0;
    }

	if (!options.check("device")) {
		ACE_OS::printf("Adding device remote_controlboard\n");
		options.put("device", "remote_controlboard");
	}

	Value& file = options.find("file");

	Property p;
	printf("Head controller working with config file %s\n", file.asString().c_str());
	p.fromConfigFile(file.asString().c_str());
    fprintf(stderr, "%s", p.toString().c_str());

    Value robotname = p.findGroup("GENERAL").find("Robot");

	yarp::String s("/");
	s += robotname.asString().c_str();
	s += "/head/control";
	options.put("local", s.c_str());

	s.clear();
	s += "/";
	s += robotname.asString().c_str();
	s += "/head";
	options.put("remote", s.c_str());

    bool verbose = options.check("verbose") || p.findGroup("GENERAL").find("Verbose").asInt() != 0;
	module.istantiateThread(options);

	

    // receive the reference value for the PID controller
	yarp::String name;
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/right/target";
	module.thread->targetRight=new BufferedPort<yarp::sig::Vector>;
    module.thread->targetLeft=new BufferedPort<yarp::sig::Vector>;
    module.thread->trackingPort=new Port;

    module.thread->targetRight->open(name.c_str());
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/left/target";
    module.thread->targetLeft->open(name.c_str());
    module.thread->inertial = new BufferedPort<yarp::sig::Vector>;
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/inertial_data";
    module.thread->inertial->open(name.c_str());

	yarp::String name_src("/");
	name_src += robotname.asString().c_str();
	name_src += "/inertial";

	Network::connect(name_src.c_str(), name.c_str(), "udp");

	Bottle tmp_b = p.findGroup("PIDS").findGroup("Pan");
	printf("%s\n", tmp_b.toString().c_str());
    module.thread->Kpan=tmp_b.get(1).asDouble();
    	
	tmp_b = p.findGroup("PIDS").findGroup("Tilt");
	printf("%s\n", tmp_b.toString().c_str());
    module.thread->Ktilt=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("Vergence");
	printf("%s\n", tmp_b.toString().c_str());
    module.thread->Kvergence=tmp_b.get(1).asDouble();
	
	tmp_b = p.findGroup("PIDS").findGroup("NeckPan");
	printf("%s\n", tmp_b.toString().c_str());
    module.thread->Kn_pan=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("NeckTilt");
    module.thread->Kn_tilt=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("VOR");
	printf("%s\n", tmp_b.toString().c_str());
    module.thread->vorx = tmp_b.get(1).asDouble();
	module.thread->vory = tmp_b.get(2).asDouble();
	module.thread->vorz = tmp_b.get(3).asDouble();

    module.setName("/rea/BIControlGazeEngine");
    return module.runModule(argc,argv);
	
	//return 0;
}
