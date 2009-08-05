#include <string>
#include <iostream>
#include <stdlib.h>

#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/Port.h>

#include <iCub/FileReaderT.h>

using namespace yarp::os;
using namespace iCub::contrib;

void exitErrorHelp(std::string error = "", bool help = false) {
    int errorCode = 0;
    if(error != "") {
        std::cerr << "Error: " << error << std::endl << std::endl;
        errorCode = 1;
    }
    if(help) {
        std::cerr << "Available options for sendcmd" << std::endl;
        std::cerr << "--help                 Display this help message" << std::endl;
        std::cerr << "--portout port         Port from which commands will be sent (/sendcmd:o)" << std::endl;
        std::cerr << "--port port            Port to which commands will be sent" << std::endl;
        std::cerr << "--file filename        File containing commands, otherwise read from stdin" << std::endl;
    }
    exit(errorCode);
}

int main(int argc, char *argv[]) {
    Network yarp;
    Property property;
    Port port;
    Value* val;

    // load property from command line parameters    
    property.fromCommand(argc, argv);
    
    // check for help parameter
    if(property.check("help", val)) {
        exitErrorHelp("", true);
    }

    // open local port
    std::string portOutName = property.check("portout", yarp::os::Value("/sendcmd:o"), "port from which commands will be sent").asString().c_str();
    if (port.open(portOutName.c_str()) != true) {
        exitErrorHelp(std::string("Failed to open outgoing port: ") + portOutName);
    }

    // connect to remote port
    if(property.check("port", val, "port to which the commands will be sent")) {
        if(!port.addOutput(val->asString().c_str())) {
            exitErrorHelp("Failed to connect ports");
        }
    } else {
        exitErrorHelp("Please supply a port to connect to using --port", true);
    }

    // open file or stdin using file reader template
    FileReaderT<Bottle> str;
    try {
        if(property.check("file", val, "file with commands, if any")) {
            str.open(val->asString().c_str());
        } else {
            str.open(std::cin);
        }
    } catch(const std::exception& e) {
        exitErrorHelp(e.what());
    }

    // send bottles and wait for replies
    try {
        Bottle* command = (Bottle*) 0;
        Bottle reply;
        while(str.hasNext()) {
            command = str.getNext();
            std::cout << /*">>> " <<*/ command->toString() << std::endl;
            port.write(*command, reply);
            for(int i = 0; i < reply.size(); i++) {
                std::cout << /*"<<< " <<*/ reply.get(i).toString() << std::endl;
            }
            // clean up
            delete command;

            // check if we still have connections
            if(port.getOutputCount() < 1) {
                exitErrorHelp("Remote port disconnected");
            }
        }
    } catch(const std::exception& e) {
        exitErrorHelp(e.what());
    }
    
    return 0;
}

