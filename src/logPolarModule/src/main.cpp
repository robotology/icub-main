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
    module.setName("/rea/LogPolar");
	// Get command line options
	//Property options;
	//options.fromCommand(argc,argv);
	//module.setOptions(options);

    return module.runModule(argc,argv);

    //MyModule module;
    //module.setName("/default/name");
    //return module.runModule(argc,argv);
}
