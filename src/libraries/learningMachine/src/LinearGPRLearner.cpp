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
#include <stdexcept>
#include <cmath>

#include <iostream>

#include <yarp/math/Math.h>

#include "iCub/learningMachine/RLSLearner.h"
#include "iCub/learningMachine/Math.h"
#include "iCub/learningMachine/Serialization.h"

using namespace yarp::math;
using namespace iCub::learningmachine::serialization;
using namespace iCub::learningmachine::math;


#include "iCub/learningMachine/LinearGPRLearner.h"

namespace iCub {
namespace learningmachine {

LinearGPRLearner::LinearGPRLearner(unsigned int dom, unsigned int cod, double sigma) {
    this->setName("LinearGPR");
    this->sampleCount = 0;
    // make sure to not use initialization list to constructor of base for
    // domain and codomain size, as it will not use overloaded mutators
    this->setDomainSize(dom);
    // slightly inefficient to use mutators, as we are initializing multiple times
    this->setCoDomainSize(cod);
    this->setSigma(sigma);
}

LinearGPRLearner::LinearGPRLearner(const LinearGPRLearner& other)
  : IFixedSizeLearner(other), sampleCount(other.sampleCount), R(other.R),
    B(other.B), W(other.W), sigma(other.sigma) {
}

LinearGPRLearner::~LinearGPRLearner() {
}

LinearGPRLearner& LinearGPRLearner::operator=(const LinearGPRLearner& other) {
    if(this == &other) return *this; // handle self initialization

    this->IFixedSizeLearner::operator=(other);
    this->sampleCount = other.sampleCount;

    this->R = other.R;
    this->B = other.B;
    this->W = other.W;
    this->sigma = other.sigma;

    return *this;
}

void LinearGPRLearner::feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output) {
    this->IFixedSizeLearner::feedSample(input, output);

    // update R
    cholupdate(this->R, input);

    // update B
    this->B = this->B + outerprod(output, input);

    // update W
    cholsolve(this->R, this->B, this->W);

    this->sampleCount++;
}

void LinearGPRLearner::train() {

}

Prediction LinearGPRLearner::predict(const yarp::sig::Vector& input) {
    this->checkDomainSize(input);

    yarp::sig::Vector output = (this->W * input);

    // note that all output dimensions share the same hyperparameters and input samples,
    // the predicted variance is therefore identical
    yarp::sig::Vector v = trsolve(this->R, input, true);
    yarp::sig::Vector std(output.size());
    std = this->sigma * sqrt(1. + dot(v,v));

    return Prediction(output, std);
}

void LinearGPRLearner::reset() {
    this->sampleCount = 0;
    this->R = eye(this->getDomainSize(), this->getDomainSize()) * this->sigma;
    this->B = zeros(this->getCoDomainSize(), this->getDomainSize());
    this->W = zeros(this->getCoDomainSize(), this->getDomainSize());
}

std::string LinearGPRLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "Sigma: " << this->getSigma() << " | ";
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    //for(unsigned int i = 0; i < this->machines.size(); i++) {
    //    buffer << "  [" << (i + 1) << "] ";
    //    buffer << "lambda: " << this->machines[i]->getLambda();
    //    buffer << std::endl;
    //}
    return buffer.str();
}

std::string LinearGPRLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getConfigHelp();
    buffer << "  sigma val             Signal noise sigma" << std::endl;
    return buffer.str();
}

void LinearGPRLearner::writeBottle(yarp::os::Bottle& bot) {
    bot << this->R << this->B << this->W << this->sigma << this->sampleCount;
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void LinearGPRLearner::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    bot >> this->sampleCount >> this->sigma >> this->W >> this->B >> this->R;
}

void LinearGPRLearner::setDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setDomainSize(size);
    this->reset();
}

void LinearGPRLearner::setCoDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setCoDomainSize(size);
    this->reset();
}

void LinearGPRLearner::setSigma(double s) {
    if(s > 0.0) {
        this->sigma = s;
        this->reset();
    } else{
        throw std::runtime_error("Signal noise sigma has to be larger than 0");
    }
}

double LinearGPRLearner::getSigma() {
    return this->sigma;
}


bool LinearGPRLearner::configure(yarp::os::Searchable& config) {
    bool success = this->IFixedSizeLearner::configure(config);

    // format: set sigma val
    if(config.find("sigma").isDouble() || config.find("sigma").isInt()) {
        this->setSigma(config.find("sigma").asDouble());
        success = true;
    }

    return success;
}

} // learningmachine
} // iCub


