/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the abstract base class of learning modules.
 *
 */

#include "iCub/IMachineLearnerModule.h"
#include <stdexcept>

namespace iCub {
namespace learningmachine {


void IMachineLearnerModule::registerPort(Contactable& port, std::string name) {
    if(port.open(name.c_str()) != true) {
        std::string msg("could not register port ");
        msg+=name;
        throw std::runtime_error(msg);
    }
}


bool IMachineLearnerModule::close() {
    unregisterAllPorts();
    return true;
}

} // learningmachine
} // iCub


