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

#include "iCub/learningMachine/FixedRangeScaler.h"
#include "iCub/learningMachine/Serialization.h"

using namespace iCub::learningmachine::serialization;

namespace iCub {
namespace learningmachine {

FixedRangeScaler::FixedRangeScaler(double li, double ui, double lo, double uo)
  : lowerBoundIn(li), upperBoundIn(ui), lowerBoundOut(lo), upperBoundOut(uo) {
    this->setName("Fixed");
    this->updateScales();
}

void FixedRangeScaler::updateScales() {
    this->scale = (this->getUpperBoundIn() - this->getLowerBoundIn()) /
                  (this->getUpperBoundOut() - this->getLowerBoundOut());
    this->offset = this->getLowerBoundIn() - (this->getLowerBoundOut() * this->scale);
}

std::string FixedRangeScaler::getInfo() {
    std::ostringstream buffer;
    buffer << this->IScaler::getInfo() << ", ";
    buffer << "In Bounds: [" << this->getLowerBoundIn() << ","
                             << this->getUpperBoundIn() << "], ";
    buffer << "Out Bounds: [" << this->getLowerBoundOut() << ","
                              << this->getUpperBoundOut() << "]";
    return buffer.str();
}

void FixedRangeScaler::writeBottle(yarp::os::Bottle& bot) {
    bot << this->lowerBoundOut << this->upperBoundOut
        << this->lowerBoundIn << this->upperBoundIn;
    // make sure to call the superclass's method
    this->IScaler::writeBottle(bot);
}

void FixedRangeScaler::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method (will reset transformer)
    this->IScaler::readBottle(bot);

    bot >> this->upperBoundIn >> this->lowerBoundIn
        >> this->upperBoundOut >> this->lowerBoundOut;
}


bool FixedRangeScaler::configure(yarp::os::Searchable& config) {
    bool success = this->IScaler::configure(config);

    // set the expected incoming lower bound (double)
    if(config.find("lowerin").isFloat64() || config.find("lowerin").isInt32()) {
        this->setLowerBoundIn(config.find("lowerin").asFloat64());
        success = true;
    }
    // set the expected incoming upper bound (double)
    if(config.find("upperin").isFloat64() || config.find("upperin").isInt32()) {
        this->setUpperBoundIn(config.find("upperin").asFloat64());
        success = true;
    }

    // set the desired outgoing lower bound (double)
    if(config.find("lowerout").isFloat64() || config.find("lowerout").isInt32()) {
        this->setLowerBoundOut(config.find("lowerout").asFloat64());
        success = true;
    }
    // set the desired outgoing bound (double)
    if(config.find("upperout").isFloat64() || config.find("upperout").isInt32()) {
        this->setUpperBoundOut(config.find("upperout").asFloat64());
        success = true;
    }

    if(!config.findGroup("in").isNull()) {
        yarp::os::Bottle& bot = config.findGroup("in");
        if(bot.size() == 3 && (bot.get(1).isInt32() || bot.get(1).isFloat64()) &&
           (bot.get(2).isInt32() || bot.get(2).isFloat64())) {

            this->setLowerBoundIn(bot.get(1).asFloat64());
            this->setUpperBoundIn(bot.get(2).asFloat64());
            success = true;
        }
    }

    if(!config.findGroup("out").isNull()) {
        yarp::os::Bottle& bot = config.findGroup("out");
        if(bot.size() == 3 && (bot.get(1).isInt32() || bot.get(1).isFloat64()) &&
           (bot.get(2).isInt32() || bot.get(2).isFloat64())) {

            this->setLowerBoundOut(bot.get(1).asFloat64());
            this->setUpperBoundOut(bot.get(2).asFloat64());
            success = true;
        }
    }

    return success;
}

} // learningmachine
} // iCub

