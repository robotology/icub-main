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

#include "iCub/Normalizer.h"

namespace iCub {
namespace learningmachine {

Normalizer::Normalizer(double l, double u)
  : lowest(DBL_MAX), highest(-DBL_MAX), lowerBound(l), upperBound(u) {
    this->setName("Normalizer");
}

void Normalizer::update(double val) {
    bool changed = false;
    if(val < this->lowest) {
        this->lowest = val;
        changed = true;
    }
    if(val > this->highest) {
        this->highest = val;
        changed = true;
    }
    if(changed) {
        this->scale = (this->highest - this->lowest) / (this->getUpperBound() - this->getLowerBound());
        this->offset = this->lowest - (this->getLowerBound() * this->scale);
    }
}

std::string Normalizer::getInfo() {
    std::ostringstream buffer;
    buffer << this->IScaler::getInfo() << ", ";
    buffer << "Bounds: [" << this->getLowerBound() << "," << this->getUpperBound() << "], ";
    buffer << "Sample Extrema: [" << this->lowest << "," << this->highest << "]";
    return buffer.str();
}

void Normalizer::writeBottle(Bottle& bot) {
    bot.addDouble(this->lowerBound);
    bot.addDouble(this->upperBound);
    bot.addDouble(this->lowest);
    bot.addDouble(this->highest);

    // make sure to call the superclass's method
    this->IScaler::writeBottle(bot);
}

void Normalizer::readBottle(Bottle& bot) {
    // make sure to call the superclass's method (will reset transformer)
    this->IScaler::readBottle(bot);

    this->highest = bot.pop().asDouble();
    this->lowest = bot.pop().asDouble();
    this->upperBound = bot.pop().asDouble();
    this->lowerBound = bot.pop().asDouble();
}

bool Normalizer::configure(Searchable& config) {
    bool success = this->IScaler::configure(config);

    // set the desired lower bound (double)
    if(config.find("lower").isDouble() || config.find("lower").isInt()) {
        this->setLowerBound(config.find("lower").asDouble());
        success = true;
    }
    // set the desired upper bound (double)
    if(config.find("upper").isDouble() || config.find("upper").isInt()) {
        this->setUpperBound(config.find("upper").asDouble());
        success = true;
    }

    return success;
}

} // learningmachine
} // iCub

