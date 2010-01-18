/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
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

#include <iostream>
#include <string>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "iCub/PredictModule.h"
#include "iCub/MachineCatalogue.h"
#include "iCub/EventListenerCatalogue.h"

using namespace iCub::learningmachine;

int main (int argc, char* argv[]) {
    Network yarp;
    int ret;

    ResourceFinder rf;
    rf.setDefaultContext("learningMachine");
    rf.configure("ICUB_ROOT", argc, argv);

    PredictModule module;
    try {
        // initialize catalogue of machine factory
        registerMachines();

        // initialize catalogue of event listeners
        registerEventListeners();

        ret = module.runModule(rf);
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Type 'quit' or CTRL+C to exit." << std::endl;
        module.close();
        return 1;
    } catch(char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        std::cerr << "Type 'quit' or CTRL+C to exit." << std::endl;
        module.close();
        return 1;
    }
    return ret;
}
