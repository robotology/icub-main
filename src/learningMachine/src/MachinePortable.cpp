/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Machine Portable implementation for sending abstract IMachineLearner objects over ports.
 * Can be replaced with PortablePair soon (hopefully)
 *
 */

#include <cassert>
#include <fstream>
#include <sstream>

#include <yarp/os/Bottle.h>

#include "iCub/MachinePortable.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

bool MachinePortable::write(ConnectionWriter& connection) {
    // return false directly if there is no machine. If not, we end up
    // up writing things on the port, after which an exception will be
    // thrown when accessing the machine.
    if(!this->hasMachine()) {
        return false;
    }
    connection.appendInt(BOTTLE_TAG_LIST);
    connection.appendInt(2);
    Bottle nameBottle;
    nameBottle.addString(this->machine->getName().c_str());
    nameBottle.write(connection);
    this->getMachine()->write(connection);

    // for text readers
    connection.convertTextMode();
    return true;
}

bool MachinePortable::read(ConnectionReader& connection) {
    if(!connection.isValid()) {
        return false;
    }

    connection.convertTextMode();
    // check headers for the pair (name + actual machine)
    int header = connection.expectInt();
    int len = connection.expectInt();
    if(header != BOTTLE_TAG_LIST || len != 2) {
        return false;
    }

    // read machine identifier and use it to create machine
    Bottle nameBottle;
    nameBottle.read(connection);
    std::string name = nameBottle.get(0).asString().c_str();
    this->machine = MachineFactory::instance().create(name);
    if(this->machine == (IMachineLearner *) 0) {
        return false;
    }

    // call read method to construct specific machine
    bool ok = machine->read(connection);
    return ok;
}

bool MachinePortable::writeToFile(std::string filename) {
    std::ofstream stream(filename.c_str());
    
    if(!stream.is_open()) {
        throw std::runtime_error(std::string("Could not open file '") + filename + "'");
    }
    
    stream << this->getMachine()->getName().c_str() << std::endl;
    stream << this->getMachine()->toString();
    
    stream.close();
    
    return true;
}
    
bool MachinePortable::readFromFile(std::string filename) {
    std::ifstream stream(filename.c_str());
    
    if(!stream.is_open()) {
        throw std::runtime_error(std::string("Could not open file '") + filename + "'");
    }
    
    std::string name;
    stream >> name;

    this->machine = MachineFactory::instance().create(name);
    std::stringstream strstr;
    //stream.get(str, '\0');
    strstr << stream.rdbuf();
    this->getMachine()->fromString(strstr.str());

    return true;
}

IMachineLearner* MachinePortable::getMachine() {
    if(!this->hasMachine()) {
        throw std::runtime_error("Attempt to retrieve inexistent machine!");
    }
    return this->machine;
}


} // learningmachine
} // contrib
} // iCub

