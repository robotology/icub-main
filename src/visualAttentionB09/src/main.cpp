// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//#include "vld.h"

#include <yarp/os/all.h>
#include <iCub/ImageProcessModule.h>

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

static GtkWidget *mainWindow = NULL;


int main(int argc, char *argv[]) {
     // Create and run processor module
	ImageProcessModule *module=new ImageProcessModule();
	module->processor1=new ImageProcessor();
	module->processor2=new ImageProcessor();
	module->processor3=new ImageProcessor();
	module->currentProcessor=module->processor1;
    module->setName("/rea/imageProcessor");
	
	//initialise Yarp Network
    Network yarp;
	// This is called in all GTK applications. Arguments are parsed
	// from the command line and are returned to the application.
    gtk_init (&argc, &argv);

	// create a new window
	module->createObjects();
	module->setUp();
	module->setAdjs();
    mainWindow = module->createMainWindow();
	

	// Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
	gtk_window_move(GTK_WINDOW(mainWindow), 10,10);
	// All GTK applications must have a gtk_main(). Control ends here
	// and waits for an event to occur (like a key press or
	// mouse event).

	gtk_main ();

    yarp::os::Network::fini();
	// Get command line options
	//Property options;
	//options.fromCommand(argc,argv);
	//module.setOptions(options);

    //return module.runModule(argc,argv);
	return 0;

    //MyModule module;
    //module.setName("/default/name");
    //return module.runModule(argc,argv);
}
