/* 
 * Copyright (C)2014  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
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

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <string>
#include <stdio.h>

#include "logger.h"

using namespace yarp::dev;
using namespace yarp::os;

//
int main(int argc, char *argv[]) 
{
    /*
    // just list the devices if no argument given
    if (argc <= 2) {
        printf("You can call %s like this:\n", argv[0]);
        printf("   %s --robot ROBOTNAME --OPTION VALUE ...\n", argv[0]);
        printf("For example:\n");
        printf("   %s --robot icub --local /talkto/james --remote /controlboard/rpc\n", argv[0]);
        printf("Here are devices listed for your system:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    // get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("robot") || !options.check("part")) {
        printf("Missing either --robot or --part options\n");
        return 0;
    }
    */
    Network::init();
	Time::turboBoost();
    
    logger l("/logger");
    l.start();

    while(1)
    {
        yarp::os::Time::delay(1.0);

        std::list<message_entry> m;
        l.get_messages_by_process("gravityCompensator", m);
        {std::list<message_entry>::iterator it;
        for (it = m.begin(); it != m.end(); it++)
            printf(" %s %s \n",it->timestamp.c_str(), it->messeges.c_str());
        }
    }

    l.stop();

    Network::fini();
    return 0;
}
