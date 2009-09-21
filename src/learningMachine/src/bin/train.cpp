/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Executable for the Train Module
 *
 */

#include <iostream>
#include <string>

#include "iCub/TrainModule.h"
#include "iCub/MachineCatalogue.h"
#include "iCub/EventListenerCatalogue.h"

using namespace iCub::contrib::learningmachine;

int main (int argc, char* argv[]) {
    Network yarp;

    TrainModule module;
    try {
        // initialize catalogue of machine factory
        registerMachines();
        
        // initialize catalogue of event listeners
        registerEventListeners();

        module.runModule(argc,argv);
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch(char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    }
    return 0;
}

