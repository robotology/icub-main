/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the standard event listener for train events.
 *
 */

#include <stdexcept>
#include <sstream>
#include <iostream>

#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>

#include "iCub/TrainEventListener.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


TrainEventListener::TrainEventListener(std::string name) : IEventListener(name) {
}

TrainEventListener::~TrainEventListener() {
    this->port.interrupt();
    this->port.close();
}

void TrainEventListener::resetPort(std::string portName) {
    // if empty portName, find first available standardly prefixed port
    std::ostringstream buffer;
    if(portName.empty()) {
        int i = 1;
        do {
            // standard prefix + i
            buffer.str(""); // clear buffer
            buffer << "/lm/event/train" << i++;
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

void TrainEventListener::handle(TrainEvent& e) {
    Bottle b;
    this->vectorToBottle(e.getInput(), b.addList());
    this->vectorToBottle(e.getDesired(), b.addList());
    this->vectorToBottle(e.getPredicted(), b.addList());
    this->port.write(b);
}

bool TrainEventListener::configure(Searchable& config) {
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
