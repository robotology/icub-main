// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/WatershedModule.h>

using namespace yarp::os;

static GtkWidget *mainWindow = NULL;

int main(int argc, char *argv[]) {
      // Create and run processor module
	WatershedModule *module=new WatershedModule();
	//module->processor1=new ImageProcessor();
	//module->processor2=new ImageProcessor();
	//module->processor3=new ImageProcessor();
	//module->currentProcessor=module->processor1;
    module->setName("/rea/Watershed");
	
	//initialise Yarp Network
    Network yarp;
	// This is called in all GTK applications. Arguments are parsed
	// from the command line and are returned to the application.
    gtk_init (&argc, &argv);

	// create a new window
	module->createObjects();
	module->setUp();
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
}
