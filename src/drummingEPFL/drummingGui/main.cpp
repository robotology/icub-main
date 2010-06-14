#include <yarp/os/Network.h>
using namespace yarp::os;

#include <gtkmm.h>
#include "drummingGui.h"

#include <iostream>
using namespace std;

int main(int argc, char* argv[])
{
    Network yarp;

    Gtk::Main kit(argc, argv);

    DrummingGui myGui;
    Gtk::Main::run(myGui);
}

