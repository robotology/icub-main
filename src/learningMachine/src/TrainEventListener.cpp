/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the standard event listener for train events.
 *
 */

#include <stdexcept>

#include <yarp/os/Bottle.h>

#include "iCub/TrainEventListener.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


TrainEventListener::TrainEventListener(std::string name) : IEventListener(name) {
    this->resetPort("/lm/event/train:o");
}

TrainEventListener::~TrainEventListener() {
}

void TrainEventListener::resetPort(std::string name) {
    this->port.interrupt();
    this->port.close();
    if (port.open(name.c_str()) != true) {
        std::string msg("could not register port ");
        msg+=name;
        throw std::runtime_error(msg);
    }
}

void TrainEventListener::handle(TrainEvent& e) {
    std::cout << e.toString() << std::endl;
}

bool TrainEventListener::configure(Searchable& config) {
    bool success = this->IEventListener::configure(config);

    // enable
    if(!config.findGroup("port").isNull()) {
        this->resetPort(config.findGroup("port").get(0).asString().c_str());
    }

    return success;
}



} // learningmachine
} // contrib
} // iCub
