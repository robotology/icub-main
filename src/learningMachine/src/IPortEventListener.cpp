/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the abstract event listener that outputs to a port.
 *
 */

#include <stdexcept>
#include <sstream>

#include <yarp/os/Network.h>

#include "iCub/IPortEventListener.h"

namespace iCub {
namespace learningmachine {

IPortEventListener& IPortEventListener::operator=(const IPortEventListener& other) {
    if(this == &other) return *this; // handle self initialization

    this->IEventListener::operator=(other);
    this->portPrefix = other.portPrefix;
    return *this;
}

void IPortEventListener::resetPort(std::string portName) {
    // if empty portName, find first available standardly prefixed port
    std::ostringstream buffer;
    if(portName.empty()) {
        int i = 1;
        do {
            // standard prefix + i
            buffer.str(""); // clear buffer
            buffer << this->portPrefix << i++ << ":o";
        } while(Network::queryName(buffer.str().c_str()).isValid());
        portName = buffer.str();
    }

    // check availability of new port
    if(Network::queryName(portName.c_str()).isValid()) {
        throw std::runtime_error(std::string("port already registered"));
    } else {
        this->port.interrupt();
        this->port.close();
        if(port.open(portName.c_str()) != true) {
            throw std::runtime_error(std::string("could not register port"));
        }
    }
}

bool IPortEventListener::configure(Searchable& config) {
    bool success = this->IEventListener::configure(config);

    // enable
    if(!config.findGroup("port").isNull()) {
        success = true;
        this->resetPort(config.findGroup("port").get(1).asString().c_str());
    }

    return success;
}


} // learningmachine
} // iCub
