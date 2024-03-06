/*
 * Copyright (C) 2024 iCub Facility - Istituto Italiano di Tecnologia
 * Author:  Jacopo Losi
 * email:   jacopo.losi@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "TemperatureManager.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>



using namespace yarp::os;

int main(int argc, char * argv[])
{
    /* initialize yarp network */
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << " YARP network does not work. Aborting...";
        return EXIT_FAILURE;
    }
    
    /* create your module */
    TemperatureManager module;
    /* prepare and configure the resource finder */
    ResourceFinder rf;
    //rf.seDefaultContext("TemperatureManager");
    //rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);
    rf.setVerbose(true);
    yDebug() << "Configuring and starting module. \n";
    module.runModule(rf);   // This calls configure(rf) and, upon success, the module execution begins with a call to updateModule()
    yDebug()<<"Main returning...";
    return 0;
}