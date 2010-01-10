// CrawlPlanner_gtkmm.cpp : Defines the entry point for the console application.
//
#include <gtkmm.h>
#include "PlannerSupervisor.h"
#include "GlobalStuff.h"

int main(int argc, char* argv[])
{
    Network yarp;

	Glib::thread_init();

    Gtk::Main kit(argc, argv);
    Glib::Dispatcher dispatcher;
    Glib::Mutex lockMutex;
    queue<Vision *> visionM_PIpe;
    queue<Vector> potentialVectorM_PIpe;

    PlannerSupervisor supervisor(dispatcher, lockMutex, visionM_PIpe, potentialVectorM_PIpe);  
    if (!supervisor.start()) 
    {
        throw std::runtime_error("Couldn't start supervisor thread!");
    }

    Gui myGui(dispatcher, lockMutex, visionM_PIpe, potentialVectorM_PIpe);
    Gtk::Main::run(myGui);

    supervisor.stop();
}

