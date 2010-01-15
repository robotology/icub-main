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

#include <cfloat>
#include <sstream>

#include <yarp/os/Value.h>

#include "iCub/FixedRangeScaler.h"

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

void FixedRangeScaler::writeBottle(Bottle& bot) {
    bot.addDouble(this->lowerBoundOut);
    bot.addDouble(this->upperBoundOut);
    bot.addDouble(this->lowerBoundIn);
    bot.addDouble(this->upperBoundIn);

    // make sure to call the superclass's method
    this->IScaler::writeBottle(bot);
}

void FixedRangeScaler::readBottle(Bottle& bot) {
    // make sure to call the superclass's method (will reset transformer)
    this->IScaler::readBottle(bot);

    this->upperBoundIn = bot.pop().asDouble();
    this->lowerBoundIn = bot.pop().asDouble();
    this->upperBoundOut = bot.pop().asDouble();
    this->lowerBoundOut = bot.pop().asDouble();
}


bool FixedRangeScaler::configure(Searchable& config) {
    bool success = this->IScaler::configure(config);

    // set the expected incoming lower bound (double)
    if(config.find("lowerin").isDouble() || config.find("lowerin").isInt()) {
        this->setLowerBoundIn(config.find("lowerin").asDouble());
        success = true;
    }
    // set the expected incoming upper bound (double)
    if(config.find("upperin").isDouble() || config.find("upperin").isInt()) {
        this->setUpperBoundIn(config.find("upperin").asDouble());
        success = true;
    }

    // set the desired outgoing lower bound (double)
    if(config.find("lowerout").isDouble() || config.find("lowerout").isInt()) {
        this->setLowerBoundOut(config.find("lowerout").asDouble());
        success = true;
    }
    // set the desired outgoing bound (double)
    if(config.find("upperout").isDouble() || config.find("upperout").isInt()) {
        this->setUpperBoundOut(config.find("upperout").asDouble());
        success = true;
    }

    if(!config.findGroup("in").isNull()) {
        Bottle& bot = config.findGroup("in");
        if(bot.size() == 3 && (bot.get(1).isInt() || bot.get(1).isDouble()) &&
           (bot.get(2).isInt() || bot.get(2).isDouble())) {

            this->setLowerBoundIn(bot.get(1).asDouble());
            this->setUpperBoundIn(bot.get(2).asDouble());
            success = true;
        }
    }

    if(!config.findGroup("out").isNull()) {
        Bottle& bot = config.findGroup("out");
        if(bot.size() == 3 && (bot.get(1).isInt() || bot.get(1).isDouble()) &&
           (bot.get(2).isInt() || bot.get(2).isDouble())) {

            this->setLowerBoundOut(bot.get(1).asDouble());
            this->setUpperBoundOut(bot.get(2).asDouble());
            success = true;
        }
    }

    return success;
}

} // learningmachine
} // iCub

