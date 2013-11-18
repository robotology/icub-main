// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
* @ingroup icub_module
* 
* \defgroup RpcIdl RpcIdl
*
* Example to show idl usage. Implements a trivial server that provides
* access to an integer.
*
* The interface of this module is defined in IRpcServer.
* 
* Copyright (C) 2013 Istituto Italiano di Tecnologia, iCub Facility
*
* Authors: Lorenzo Natale 
*
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include "RpcServerImpl.h"

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>

int main(int argc, char *argv[]) {
    yarp::os::Network yarp;

    RpcServerImpl server;
    yarp::os::Port port;
    server.yarp().attachAsServer(port);
    if (!port.open("/server")) { return 1; }
    
    while (true) {
        printf("Server running happily\n");
        yarp::os::Time::delay(10);
    }
    port.close();
    return 0;
}
