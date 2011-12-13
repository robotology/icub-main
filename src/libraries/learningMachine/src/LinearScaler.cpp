/*
 * Copyright (C) 2007-2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include <cfloat>
#include <sstream>

#include <yarp/os/Value.h>

#include "iCub/learningMachine/LinearScaler.h"

namespace iCub {
namespace learningmachine {

LinearScaler::LinearScaler(double s, double o)
  : IScaler(s, o) {
    this->setName("LinearScaler");
}

std::string LinearScaler::getInfo() {
    std::ostringstream buffer;
    buffer << this->IScaler::getInfo();
    return buffer.str();
}

void LinearScaler::writeBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IScaler::writeBottle(bot);
}

void LinearScaler::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method (will reset transformer)
    this->IScaler::readBottle(bot);
}

bool LinearScaler::configure(yarp::os::Searchable& config) {
    bool success = this->IScaler::configure(config);

    // set the desired scale (double)
    if(config.find("scale").isDouble() || config.find("scale").isInt()) {
        this->setScale(1. / config.find("scale").asDouble());
        success = true;
    }
    // set the desired offset (double)
    if(config.find("offset").isDouble() || config.find("offset").isInt()) {
        this->setOffset(config.find("offset").asDouble());
        success = true;
    }

    return success;
}

} // learningmachine
} // iCub

