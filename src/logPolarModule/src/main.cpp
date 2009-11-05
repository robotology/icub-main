// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/LogPolarModule.h>

using namespace yarp::os;

class MyModule : public Module {
private:
    Port myPort1;
    Port myPort2;
public:
    virtual bool open(Searchable& config) {
        myPort1.open(getName()); // will be /default/name (unless overridden)
        myPort2.open(getName("copy")); // will be /default/name/copy ( " " )
		attach(myPort1); // process input to this port
        return true;
    }
	virtual bool interruptModule() {
		myPort1.interrupt();
		myPort2.interrupt();
		return true;
	}
	virtual bool close() {
		myPort1.close();
		myPort2.close();
		return true;
	}
	virtual bool updateModule() {
         Bottle bot;
         bool ok = myPort2.read(bot);
         if (!ok) { return false; }  // shutdown if interrupted
			printf("got %s\n", bot.toString().c_str());
         return true;
    }

};

int main(int argc, char *argv[]) {
    //initialise Yarp Network
	Network yarp;
	// Create and run our module
	LogPolarModule module;
	bool ret=yarp.connect("icubSim/cam/right","/rea/LogPolar/in");
	if(!ret){
		module.setName("/rea/LogPolar2");
	}
	else
		module.setName("/rea/LogPolar");

	// Get command line options
	Property options;
	if(argc<2){
		//there are no parameters
		// litte help on how to use
		printf("______ HELP ________ \n");
		printf(" \n");
		printf("USER COMMANDS: \n");
		printf("--mode (SIMULATION,FORWARD,INVERSE): selects what this module operates \n");
		printf(" \n");
		printf(" \n");
		//start of the default mode
		printf("No commands, starting of the default mode ................... \n");
		options.put("mode","FORWARD");
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
