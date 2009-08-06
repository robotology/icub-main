/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * *TEMPORARY* Implementation of the Random Feature transformer.
 *
 */

#include <cassert>
#include <sstream>

#include <math.h>

#include <yarp/math/Rand.h>

#include "iCub/RandomFeature.h"

#define TWOPI 6.2831853071795862

// NOTE TO SELF: remove ASAP
//#include <iostream>

using namespace yarp::math;

namespace iCub {
namespace contrib {
namespace learningmachine {

RandomFeature::RandomFeature(std::string name, double g) : IFixedSizeTransformer(name) {
    // will initiate reset automatically
    this->setGamma(g);
}

RandomFeature::~RandomFeature() {
}


void RandomFeature::transform(const Vector& input, Vector& output) {
    this->IFixedSizeTransformer::transform(input, output);
    //assert(input.size() == this->scalers.size());
    // python: x_f = numpy.cos(numpy.dot(self.W, x) + self.bias) / math.sqrt(self.nproj)
    output = (this->W * input) + this->b;
    double nprojsq = sqrt((double)this->getCoDomainSize());
    for(int i = 0; i < output.size(); i++) {
        output(i) = cos(output(i)) / nprojsq;
    }
    return;
}

/*void RandomFeature::setDomainSize(int size) {
    // domain size and codomain have to be equally sized
    this->IFixedSizeTransformer::setDomainSize(size);
}*/

void RandomFeature::setCoDomainSize(int size) {
    // call method in base class
    this->IFixedSizeTransformer::setCoDomainSize(size);
    // rebuild projection matrix
    this->reset();
}

void RandomFeature::reset() {
    this->IFixedSizeTransformer::reset();

    // create pseudo random number generators
    yarp::math::impl::RandnScalar prng_normal;
    yarp::math::impl::RandScalar prng_uniform;

    // create new projection matrix
    this->W.resize(this->getCoDomainSize(), this->getDomainSize());
    double gammasq = sqrt(2 * this->gamma);

    // python: self.W = math.sqrt(2 * self.gamma) * numpy.random.randn(self.nproj, self.n)
    for(int r = 0; r < this->W.rows(); r++) {
        for(int c = 0; c < this->W.cols(); c++) {
            this->W(r, c) = gammasq * prng_normal.get();
        }
    }
    
    // python: self.bias = 2 * numpy.pi * numpy.random.rand(self.nproj)
    this->b.resize(this->getCoDomainSize());
    for(int i = 0; i < this->b.size(); i++) {
        this->b(i) = TWOPI * prng_uniform.get();
    }
}

std::string RandomFeature::getStats() {
    
    std::ostringstream buffer;
    buffer << this->IFixedSizeTransformer::getStats();
    buffer << " gamma: " << this->gamma;
    return buffer.str();
}

std::string RandomFeature::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeTransformer::getConfigHelp();
    buffer << "  gamma val             Set gamma parameter" << std::endl;
    return buffer.str();
}

bool RandomFeature::configure(Searchable &config) {
    bool success = this->IFixedSizeTransformer::configure(config);

    // format: set gamma val
    if(config.find("gamma").isDouble() || config.find("gamma").isInt()) {
        this->setGamma(config.find("gamma").asDouble());
        success = true;
    }
    return success;
}


} // learningmachine
} // contrib
} // iCub

