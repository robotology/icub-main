/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation of the linear scaler (element-based).
 *
 */


#include <sstream>

#include <yarp/os/Bottle.h>

#include "iCub/IScaler.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

double IScaler::transform(double val) {
    // update scaler if enabled
    if(this->updateEnabled) {
        this->update(val);
    }
    // check for division by zero
    return (fabs(this->scale) < 1.e-20) ? (val - this->offset) : (val - this->offset) / this->scale;
}


std::string IScaler::getInfo() {
    std::ostringstream buffer;
    buffer << "Type: " << this->getName() << ", ";
    buffer << "Offset: " << this->offset << ", ";
    buffer << "Scale: " << this->scale << ", ";
    buffer << "Update: " << (this->updateEnabled ? "enabled" : "disabled");
    return buffer.str();
}

std::string IScaler::toString() {
    Bottle model;
    this->writeBottle(model);
    return model.toString().c_str();
}

bool IScaler::fromString(const std::string& str) {
    Bottle model(str.c_str()); 
    this->readBottle(model);
    return true;
}

void IScaler::writeBottle(Bottle& bot) {
    bot.addDouble(this->offset);
    bot.addDouble(this->scale);
    bot.addInt((this->updateEnabled ? 1 : 0));
}

void IScaler::readBottle(Bottle& bot) {
    this->updateEnabled = (bot.pop().asInt() == 1);
    this->scale = bot.pop().asDouble();
    this->offset = bot.pop().asDouble();
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

