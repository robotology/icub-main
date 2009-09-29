/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the standard event listener for predict events.
 *
 */

#include <stdexcept>
#include <sstream>

#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>

#include "iCub/PredictEventListener.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


PredictEventListener::PredictEventListener(std::string name) : IEventListener(name) {
}

PredictEventListener::~PredictEventListener() {
    this->port.interrupt();
    this->port.close();
}

void PredictEventListener::resetPort(std::string portName) {
    // if empty portName, find first available standardly prefixed port
    std::ostringstream buffer;
    if(portName.empty()) {
        int i = 1;
        do {
            // standard prefix + i
            buffer.str(""); // clear buffer
            buffer << "/lm/event/predict" << i++;
        } while(Network::queryName(buffer.str().c_str()).isValid());
        portName = buffer.str();
    }
    
    // check availability of new port
    if(Network::queryName(portName.c_str()).isValid()) {
        throw std::runtime_error(std::string("port already registered"));
    } else {
        this->port.interrupt();
        this->port.close();
        if (port.open(portName.c_str()) != true) {
            throw std::runtime_error(std::string("could not register port"));
        }
    }
}

void PredictEventListener::handle(TrainEvent& e) {
    //std::cout << e.toString() << std::endl;
}

bool PredictEventListener::configure(Searchable& config) {
    bool success = this->IEventListener::configure(config);

    // enable
    if(!config.findGroup("port").isNull()) {
        success = true;
        this->resetPort(config.findGroup("port").get(1).asString().c_str());
    }

    return success;
}

} // learningmachine
} // contrib
} // iCub
