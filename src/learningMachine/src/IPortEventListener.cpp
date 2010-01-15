/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
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
