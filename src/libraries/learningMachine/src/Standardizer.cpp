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

#include <sstream>
#include <cmath>

//#include <yarp/os/Value.h>

#include "iCub/learningMachine/Standardizer.h"
#include "iCub/learningMachine/Serialization.h"

using namespace iCub::learningmachine::serialization;

namespace iCub {
namespace learningmachine {

Standardizer::Standardizer(double m, double s)
  : noSamples(0), runningMean(0.), runningStd(0.), squaredErrors(0.) {
    this->setName("Standardizer");
    this->setDesiredMean(m);
    this->setDesiredStd(s);
}

void Standardizer::update(double val) {
    /*
     * Running variance calculation, taken from:
     *
     * Donald E. Knuth (1998). The Art of Computer Programming, volume 2:
     *   Seminumerical Algorithms, 3rd edn., p. 232. Boston: Addison-Wesley
     */
    this->noSamples++;
    double delta = val - this->runningMean;
    this->runningMean += (delta / this->noSamples);
    this->squaredErrors += (delta * (val - this->runningMean)); // M2 in TAOCP
    this->runningStd = sqrt(this->squaredErrors / (this->noSamples + 1));

    this->scale = this->runningStd / this->std;
    this->offset = (this->runningMean - this->mean) * this->scale;
}

std::string Standardizer::getInfo() {
    std::ostringstream buffer;
    buffer << this->IScaler::getInfo() << ", ";
    buffer << "Input Stats: " << this->runningMean << " +/- " << this->runningStd << ", ";
    buffer << "Desired: " << this->getDesiredMean() << " +/- " << this->getDesiredStd();
    return buffer.str();
}

void Standardizer::writeBottle(yarp::os::Bottle& bot) {
    bot << this->noSamples << this->squaredErrors << this->mean << this->std
        << this->runningMean << this->runningStd;

    // make sure to call the superclass's method
    this->IScaler::writeBottle(bot);
}

void Standardizer::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method (will reset transformer)
    this->IScaler::readBottle(bot);

    bot >> this->runningStd >> this->runningMean >> this->std >> this->mean
        >> this->squaredErrors >> this->noSamples;
}

bool Standardizer::configure(yarp::os::Searchable& config) {
    bool success = this->IScaler::configure(config);

    // set the desired output mean (double)
    if(config.find("mean").isDouble() || config.find("mean").isInt()) {
        this->setDesiredMean(config.find("mean").asDouble());
        success = true;
    }
    // set the desired output standard deviation (double)
    if(config.find("std").isDouble() || config.find("std").isInt()) {
        this->setDesiredStd(config.find("std").asDouble());
        success = true;
    }

    return success;
}


} // learningmachine
} // iCub

