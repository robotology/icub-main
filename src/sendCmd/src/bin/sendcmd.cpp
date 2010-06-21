#include <string>
#include <iostream>
#include <iomanip>
#include <stdlib.h>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>

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
        std::cerr << "--delay s              Delay in seconds between commands (0.0)" << std::endl;
        std::cerr << "--noreply             Do not wait for reply of port" << std::endl;
    }
    exit(errorCode);
}

int main(int argc, char *argv[]) {
    Network yarp;
    //Property property;
    ResourceFinder rf;
    Port port;
    Value* val;
    double delay;
    bool expectreply = true;


    

    // load resource finder from command line parameters
    rf.configure("ICUB_ROOT", argc, argv);
    // load property from command line parameters    
    //property.fromCommand(argc, argv);
    
    // check for help parameter
    if(rf.check("help")) {
        exitErrorHelp("", true);
    }
    
    // check for delay parameter
    delay = rf.check("delay", Value(0.0)).asDouble();
    
    // check for reply parameter
    if(rf.check("noreply")) {
        expectreply = false;
    }

    // open local port
    std::string portOutName = rf.check("portout", yarp::os::Value("/sendcmd:o"), "port from which commands will be sent").asString().c_str();
    if (port.open(portOutName.c_str()) != true) {
        exitErrorHelp(std::string("Failed to open outgoing port: ") + portOutName);
    }

    // open file or stdin using file reader template
    FileReaderT<Bottle> str;
    try {
        if(rf.check("file", val, "file with commands, if any")) {
            str.open(val->asString().c_str());
        } else {
            str.open(std::cin);
        }
    } catch(const std::exception& e) {
        exitErrorHelp(e.what());
    }

    // connect to remote port
    if(rf.check("port", val, "port to which the commands will be sent")) {
        if(!port.addOutput(val->asString().c_str())) {
            port.close();
            exitErrorHelp("Failed to connect ports");
        }
    } else {
        port.close();
        exitErrorHelp("Please supply a port to connect to using --port", true);
    }

    // send bottles and wait for replies
    try {
        Bottle* command = (Bottle*) 0;
        Bottle reply;
        double start = Time::now();
        double cmdstart, cmdpassed;
        while(str.hasNext()) {
            cmdstart = Time::now();
            command = str.getNext();
            std::cout << ">" << std::setw(6) << std::fixed << std::setprecision(3) << (Time::now() - start) << "s: ";
            std::cout << command->toString() << std::endl;

            if(expectreply) {
                port.write(*command, reply);
                for(int i = 0; i < reply.size(); i++) {
                    std::cout << "<" << std::setw(6) << std::fixed << std::setprecision(3) << (Time::now() - start) << "s: ";
                    std::cout << reply.get(i).toString() << std::endl;
                }
            } else {
                port.write(*command);            
            }
            // clean up
            delete command;

            // check if we still have connections
            if(port.getOutputCount() < 1) {
                port.close();
                exitErrorHelp("Remote port disconnected");
            }
            
            // wait given delay before sending next one
            cmdpassed = Time::now() - cmdstart;
            if(cmdpassed < delay) {
                Time::delay(delay - cmdpassed);
            }
        }
    } catch(const std::exception& e) {
        port.close();
        exitErrorHelp(e.what());
    }
    
    return 0;
}

