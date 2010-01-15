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

#include <sstream>

#include <yarp/os/Bottle.h>

#include "iCub/IScaler.h"

namespace iCub {
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

    // toggle enable/disable update
    if(!config.findGroup("update").isNull()) {
        this->setUpdateEnabled(!this->getUpdateEnabled());
        success = true;
    }

    // enable update
    if(!config.findGroup("enable").isNull()) {
        this->setUpdateEnabled(true);
        success = true;
    }

    // disable update
    if(!config.findGroup("disable").isNull()) {
        this->setUpdateEnabled(false);
        success = true;
    }

    return success;
}


} // learningmachine
} // iCub

