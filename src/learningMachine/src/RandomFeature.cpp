/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation of the Random Feature transformer.
 *
 */

#include <cassert>
#include <sstream>
#include <math.h>

#include <yarp/math/Rand.h>

#include "iCub/RandomFeature.h"

#define TWOPI 6.2831853071795862

using namespace yarp::math;

namespace iCub {
namespace learningmachine {

RandomFeature::RandomFeature(int dom, int cod, double gamma) {
    this->setName("RandomFeature");
    this->setDomainSize(dom);
    this->setCoDomainSize(cod);
    // will initiate reset automatically
    this->setGamma(gamma);
}

RandomFeature::~RandomFeature() {
}


Vector RandomFeature::transform(const Vector& input) {
    Vector output = this->IFixedSizeTransformer::transform(input);

    // python: x_f = numpy.cos(numpy.dot(self.W, x) + self.bias) / math.sqrt(self.nproj)
    output = (this->W * input) + this->b;
    double nprojsq = sqrt((double)this->getCoDomainSize());
    for(int i = 0; i < output.size(); i++) {
        output(i) = cos(output(i)) / nprojsq;
    }
    return output;
}

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

void RandomFeature::writeBottle(Bottle& bot) {
    bot.addDouble(this->getGamma());

    // write bias b
    for(int i = 0; i < this->b.size(); i++) {
        bot.addDouble(this->b(i));    
    }
    bot.addInt(this->b.size());

    // write matrix W
    for(int r = 0; r < this->W.rows(); r++) {
        for(int c = 0; c < this->W.cols(); c++) {
            bot.addDouble(this->W(r, c));
        }
    }
    bot.addInt(this->W.rows());
    bot.addInt(this->W.cols());
    
    // make sure to call the superclass's method
    this->IFixedSizeTransformer::writeBottle(bot);
}

void RandomFeature::readBottle(Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeTransformer::readBottle(bot);
    
    // read matrix W
    this->W.resize(bot.pop().asInt(), bot.pop().asInt());
    for(int r = this->W.rows() - 1; r >= 0; r--) {
        for(int c = this->W.cols() - 1; c >= 0; c--) {
            this->W(r, c) = bot.pop().asDouble();
        }
    }

    // read bias
    this->b.resize(bot.pop().asInt());
    for(int i = this->b.size() - 1; i >= 0; i--) {
        this->b(i) = bot.pop().asDouble();
    }

    // do _not_ use public accessor, as it resets the matrix
    this->gamma = bot.pop().asDouble();
}



std::string RandomFeature::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeTransformer::getInfo();
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
} // iCub

