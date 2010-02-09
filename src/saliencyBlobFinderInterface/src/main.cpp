// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <iCub/blobFinderModule.h>

using namespace yarp::os;

static GtkWidget *mainWindow = NULL;

int main(int argc, char *argv[]) {
      // Create and run processor module
	blobFinderModule *module=new blobFinderModule();
	//module->processor1=new ImageProcessor();
	//module->processor2=new ImageProcessor();
	//module->processor3=new ImageProcessor();
	//module->currentProcessor=module->processor1;
    module->setName("/blobFinderInterface");
	
	//initialise Yarp Network
    Network yarp;
	// This is called in all GTK applications. Arguments are parsed
	// from the command line and are returned to the application.
    gtk_init (&argc, &argv);

	
	// Get command line options
	//Property options;
	//options.fromCommand(argc,argv);
	//module.setOptions(options);

    return module->runModule(argc,argv);
	//return 0;
}
