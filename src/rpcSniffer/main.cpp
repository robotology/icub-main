// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_rpcSniffer rpcSniffer 
 *
 * A basic module for sniffing an RPC port
 * (e.g. a remote control board)
 *
 * \section intro_sec Description
 * 
 * This module can be used to forward 
 * the commands of a remote_control_board 
 * to a non-rpc port.  
 *
 * \section parameters_sec Parameters
 * The module requires the following parameters (see config.ini for a template)
 *
 * ./rpcSniffer --client clientName --server serverName --local localRpcName --out outputPortName 
 *
 * - client: the rpc:o port opened by the remoteControlBoard (sending commands)
 * - server: the rpc:i port opened by the controlBoard (accepting and replying commands)
 * - local:  the name of rpc port that rpcSniffer will open in order to listen to the server/client conversation
 * - out :   the name of the port used to forward the commands between client and server

 * \section portsc_sec Ports Created
 * 
 * - an rpc port for sniffing (specified by "local" in config.ini)
 * - a port for forwarding the sniffed messages (specified by "out" in config.ini)
 * 
 *
 * \author Francesco Nori
 *
 * Copyright (C) 2008 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at src/cableLengthGuard/main.cpp.
 */



#include "collectorPort.h"

//
int main(int argc, char *argv[]) 
{
    // get config file options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("local") || 
        !options.check("client")|| 
        !options.check("server")||
        !options.check("out")) {
        ACE_OS::printf("Missing either 'client' or 'server' or 'local' or 'out' options\n");
        return 0;
    }

    Network::init();
	Time::turboBoost();
    
    //client port name
    Value& client = options.find("client");
    //server port name
    Value& server = options.find("server");
    //local port name
    Value& middle = options.find("local");
    //out port name
    Value& output = options.find("out");

    collectorPort *lastCommand = new collectorPort();

    //Start the port listener
    lastCommand -> open(middle.toString().c_str(),
                        client.toString().c_str(), 
                        server.toString().c_str(),
                        output.toString().c_str());
    lastCommand -> start();

    //Start the cable length checker
    Time::delay(1);


    char cmd[80];
    bool quit=false;
    while (!quit) 
    {
        ACE_OS::printf("Type 'quit+enter' to exit the program\n");
        scanf("%s", cmd);
        if (strcmp(cmd, "quit")==0)
            quit=true;
    }
    
    fprintf(stderr, "Closing port sniffer\n");
    lastCommand->close();
    Time::delay(1);

    fprintf(stderr, "Stopping port sniffer\n");
    lastCommand->stop();

    fprintf(stderr, "Deleting port sniffer class\n");
    delete lastCommand;

    Network::fini();
    return 0;
}
