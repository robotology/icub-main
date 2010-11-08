#include <gtkmm.h>
#include <yarp/os/Network.h>

#include "iCubInterfaceGuiClient.h"

int main(int argc, char *argv[])
{
    Glib::thread_init();
    Gtk::Main kit(argc,argv);

	yarp::os::Network yarp;

    iCubInterfaceGuiWindow client;

    kit.run(client);

    return 0;
}
