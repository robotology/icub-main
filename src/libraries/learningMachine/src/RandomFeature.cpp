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

#include <cassert>
#include <sstream>
#include <cmath>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include "iCub/learningMachine/RandomFeature.h"
#include "iCub/learningMachine/Serialization.h"
#include "iCub/learningMachine/Math.h"

#define TWOPI 6.2831853071795862

using namespace yarp::math;
using namespace iCub::learningmachine::serialization;
using namespace iCub::learningmachine::math;

namespace iCub {
namespace learningmachine {

RandomFeature::RandomFeature(unsigned int dom, unsigned int cod, double gamma) {
    this->setName("RandomFeature");
    this->setDomainSize(dom);
    this->setCoDomainSize(cod);
    // will initiate reset automatically
    this->setGamma(gamma);
}

yarp::sig::Vector RandomFeature::transform(const yarp::sig::Vector& input) {
    yarp::sig::Vector output = this->IFixedSizeTransformer::transform(input);

    // python: x_f = numpy.cos(numpy.dot(self.W, x) + self.bias) / math.sqrt(self.nproj)
    output = cosvec((this->W * input) + this->b) * (1. / std::sqrt((double) this->getCoDomainSize()));
    return output;
}

void RandomFeature::setDomainSize(unsigned int size) {
    // call method in base class
    this->IFixedSizeTransformer::setDomainSize(size);
    // rebuild projection matrix
    this->reset();
}

void RandomFeature::setCoDomainSize(unsigned int size) {
    // call method in base class
    this->IFixedSizeTransformer::setCoDomainSize(size);
    // rebuild projection matrix
    this->reset();
}

void RandomFeature::reset() {
    this->IFixedSizeTransformer::reset();

    // create pseudo random number generators
    yarp::math::RandnScalar prng_normal;
    yarp::math::RandScalar prng_uniform;

    // create new projection matrix
    this->W = sqrt(2 * this->gamma) * random(this->getCoDomainSize(), this->getDomainSize(), prng_normal);

    this->b = TWOPI * random(this->getCoDomainSize(), prng_uniform);
}

void RandomFeature::writeBottle(yarp::os::Bottle& bot) {
    bot << this->getGamma() << b << W;
    // make sure to call the superclass's method
    this->IFixedSizeTransformer::writeBottle(bot);
}

void RandomFeature::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeTransformer::readBottle(bot);
    // do _not_ use public accessor, as it resets the matrix
    bot >> W >> this->b >> this->gamma;
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

bool RandomFeature::configure(yarp::os::Searchable &config) {
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

