/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation of the standardization scaler.
 *
 */

#include <sstream>
#include <cmath>

#include <yarp/os/Value.h>

#include "iCub/Standardizer.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

Standardizer::Standardizer(std::string name, double m, double s) : IScaler(name) { 
    this->noSamples = 0;
    this->runningMean = 0.;
    this->runningStd = 0.;
    this->squaredErrors = 0.;
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

std::string Standardizer::getStats() {
    std::ostringstream buffer;
    buffer << this->IScaler::getStats() << ", ";
    buffer << "Input Stats: " << this->runningMean << " +/- " << this->runningStd << ", ";
    buffer << "Desired: " << this->getDesiredMean() << " +/- " << this->getDesiredStd();
    return buffer.str();

}

bool Standardizer::configure(Searchable& config) {
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
} // contrib
} // iCub

