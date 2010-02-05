// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//#include "vld.h"

#include <yarp/os/all.h>
#include <iCub/fingerInterfaceModule.h>

#include <gtk/gtk.h>
//#include <glib.h>

using namespace yarp::os;

#ifdef USE_ICUB_MOD
#include "iCub/drivers.h"
#endif

static GtkWidget *mainWindow = NULL;

int main(int argc, char *argv[]) {
    Network yarp;

    #ifdef USE_ICUB_MOD
	yarp::dev::DriverCollection dev;
	#endif

     // Create and run processor module
	fingerInterfaceModule module;

    //module->setName("/rea/fingerInterface");
	
	//initialise Yarp Network

	//so that gdk looks nice when we use threads
  //  g_thread_init(NULL);
  ////  gdk_threads_init();
   // gdk_threads_enter();
	//gdk_threads_leave(); we put into fingerIntefaceModule.cpp

	// This is called in all GTK applications. Arguments are parsed
	// from the command line and are returned to the application.
    gtk_init (&argc, &argv);

    return module.runModule(argc,argv);
	//return 0;
}
