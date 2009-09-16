/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Executable for the Transform Module
 *
 */

#include <iostream>
#include <string>

#include "iCub/TransformModule.h"
//#include "iCub/TransformerCatalogue.h"
#include "iCub/TransformerSupport.h"

using namespace iCub::contrib::learningmachine;

int main (int argc, char* argv[]) {
    Network yarp;
    TransformerSupport

    TransformerModule module;
    try {
        //registerTransformers();

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

