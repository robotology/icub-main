/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation of the linear scaler (element-based).
 *
 */


//#include <cassert>
#include <sstream>

// NOTE TO SELF: remove ASAP
#include <iostream>

#include <yarp/os/Bottle.h>
#include "iCub/IScaler.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


std::string IScaler::getStats() {
    std::ostringstream buffer;
    buffer << "Type: " << this->getName() << ", ";
    buffer << "Offset: " << this->offset << ", ";
    buffer << "Scale: " << this->scale << ", ";
    buffer << "Update: " << (this->updateEnabled ? "enabled" : "disabled");
    return buffer.str();
}

bool IScaler::configure(Searchable& config) {
    bool success = false;

    // enable/disable update
    if(!config.findGroup("update").isNull()) {
        this->setUpdateEnabled(!this->getUpdateEnabled());
        success = true;
    }

    return success;
}


} // learningmachine
} // contrib
} // iCub

