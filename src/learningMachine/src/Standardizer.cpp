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

Standardizer::Standardizer(double m, double s) { 
    this->setName("Standardizer");
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

std::string Standardizer::getInfo() {
    std::ostringstream buffer;
    buffer << this->IScaler::getInfo() << ", ";
    buffer << "Input Stats: " << this->runningMean << " +/- " << this->runningStd << ", ";
    buffer << "Desired: " << this->getDesiredMean() << " +/- " << this->getDesiredStd();
    return buffer.str();
}

void Standardizer::writeBottle(Bottle& bot) {
    bot.addInt(this->noSamples);
    bot.addDouble(this->squaredErrors);
    bot.addDouble(this->mean);
    bot.addDouble(this->std);
    bot.addDouble(this->runningMean);
    bot.addDouble(this->runningStd);

    // make sure to call the superclass's method
    this->IScaler::writeBottle(bot);
}

void Standardizer::readBottle(Bottle& bot) {
    // make sure to call the superclass's method (will reset transformer)
    this->IScaler::readBottle(bot);
    
    this->runningStd = bot.pop().asDouble();
    this->runningMean = bot.pop().asDouble();
    this->std = bot.pop().asDouble();
    this->mean = bot.pop().asDouble();
    this->squaredErrors = bot.pop().asDouble();
    this->noSamples = bot.pop().asInt();
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

