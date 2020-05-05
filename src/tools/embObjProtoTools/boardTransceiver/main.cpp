/*
* Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
* Author:  Alberto Cardellino
* email:   alberto.cardellino@iit.it
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

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>

#include <boardTransceiver.hpp>

using namespace std;
using namespace yarp::os;


int main(int argc, char *argv[])
{
    Network yarp;
    ResourceFinder rf;
//    rf.setDefaultConfigFile("board.ini");    //overridden by --from parameter
//    rf.setDefaultContext("???");           //overridden by --context parameter
    rf.configure(argc, argv);
    BoardTransceiver module;
    return module.runModule(rf);
}
