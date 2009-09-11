/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Executable for the Predict Module
 *
 */
#include <iostream>
#include <string>

#include "iCub/PredictModule.h"
//#include "iCub/MachineCatalogue.h"
#include "iCub/Support.h"

using namespace iCub::contrib::learningmachine;

int main (int argc, char* argv[]) {
    Network yarp;
    Support support;

    PredictModule module(&support);
    try {
        // initialize catalogue of machine factory
        //registerMachines();

        module.runModule(argc,argv);
        //module.attachTerminal();
    } catch(const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    } catch(char* msg) {
        std::cerr << "Error: " << msg << std::endl;
        return 1;
    }
    return 0;
}
