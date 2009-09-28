/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation of the normalization scaler.
 *
 */

#include <cfloat>
#include <sstream>

#include <yarp/os/Value.h>

#include "iCub/Normalizer.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

Normalizer::Normalizer(std::string name, double l, double u) : IScaler(name) {
    this->lowest = DBL_MAX;
    this->highest = -DBL_MAX;
    this->setLowerBound(l);
    this->setUpperBound(u);
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
} // contrib
} // iCub

