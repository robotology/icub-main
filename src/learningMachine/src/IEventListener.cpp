/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the abstract event listener class.
 *
 */

#include <yarp/os/Bottle.h>

#include "iCub/IEventListener.h"
#include "iCub/EventListenerFactory.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

IEventListener::IEventListener(std::string name) {
    this->setName(name);
    this->setEnabled(true);
}

bool IEventListener::configure(Searchable& config) {
    bool success = false;

    // enable
    if(!config.findGroup("enable").isNull()) {
        this->setEnabled(true);
        success = true;
    }

    // disable
    if(!config.findGroup("disable").isNull()) {
        this->setEnabled(false);
        success = true;
    }

    return success;
}

} // learningmachine
} // contrib
} // iCub

