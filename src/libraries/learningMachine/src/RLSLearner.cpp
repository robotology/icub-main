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
#include <stdexcept>
#include <cmath>

#include <yarp/math/Math.h>

#include "iCub/learningMachine/RLSLearner.h"
#include "iCub/learningMachine/Math.h"
#include "iCub/learningMachine/Serialization.h"

using namespace yarp::math;
using namespace iCub::learningmachine::serialization;
using namespace iCub::learningmachine::math;

namespace iCub {
namespace learningmachine {


RLSLearner::RLSLearner(unsigned int dom, unsigned int cod, double lambda) {
    this->setName("RLS");
    this->sampleCount = 0;
    // make sure to not use initialization list to constructor of base for
    // domain and codomain size, as it will not use overloaded mutators
    this->setDomainSize(dom);
    // slightly inefficient to use mutators, as we are initializing multiple times
    this->setCoDomainSize(cod);
    this->setLambda(lambda);
}

RLSLearner::RLSLearner(const RLSLearner& other)
  : IFixedSizeLearner(other), sampleCount(other.sampleCount), R(other.R),
    B(other.B), W(other.W), lambda(other.lambda) {
}

RLSLearner::~RLSLearner() {
}

RLSLearner& RLSLearner::operator=(const RLSLearner& other) {
    if(this == &other) return *this; // handle self initialization

    this->IFixedSizeLearner::operator=(other);
    this->sampleCount = other.sampleCount;

    this->R = other.R;
    this->B = other.B;
    this->W = other.W;
    this->lambda = other.lambda;

    return *this;
}

void RLSLearner::feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output) {
    this->IFixedSizeLearner::feedSample(input, output);

    // update R
    cholupdate(this->R, input);

    // update B
    this->B = this->B + outerprod(output, input);

    // update W
    cholsolve(this->R, this->B, this->W);

    this->sampleCount++;
}

void RLSLearner::train() {

}

Prediction RLSLearner::predict(const yarp::sig::Vector& input) {
    this->checkDomainSize(input);

    yarp::sig::Vector output = (this->W * input);

    return Prediction(output);
}

void RLSLearner::reset() {
    this->sampleCount = 0;
    this->R = eye(this->getDomainSize(), this->getDomainSize()) * sqrt(this->lambda);
    this->B = zeros(this->getCoDomainSize(), this->getDomainSize());
    this->W = zeros(this->getCoDomainSize(), this->getDomainSize());
}

std::string RLSLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "Lambda: " << this->getLambda() << " | ";
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    //for(unsigned int i = 0; i < this->machines.size(); i++) {
    //    buffer << "  [" << (i + 1) << "] ";
    //    buffer << "lambda: " << this->machines[i]->getLambda();
    //    buffer << std::endl;
    //}
    return buffer.str();
}

std::string RLSLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getConfigHelp();
    buffer << "  lambda val            Regularization parameter lambda" << std::endl;
    return buffer.str();
}

void RLSLearner::writeBottle(yarp::os::Bottle& bot) {
    bot << this->R << this->B << this->W << this->lambda << this->sampleCount;
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void RLSLearner::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    bot >> this->sampleCount >> this->lambda >> this->W >> this->B >> this->R;
}

void RLSLearner::setDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setDomainSize(size);
    this->reset();
}

void RLSLearner::setCoDomainSize(unsigned int size) {
    this->IFixedSizeLearner::setCoDomainSize(size);
    this->reset();
}

void RLSLearner::setLambda(double l) {
    if(l > 0.0) {
        this->lambda = l;
        this->reset();
    } else{
        throw std::runtime_error("Regularization parameter lamdba has to be larger than 0");
    }
}

double RLSLearner::getLambda() {
    return this->lambda;
}


bool RLSLearner::configure(yarp::os::Searchable& config) {
    bool success = this->IFixedSizeLearner::configure(config);

    // format: set lambda val
    if(config.find("lambda").isDouble() || config.find("lambda").isInt()) {
        this->setLambda(config.find("lambda").asDouble());
        success = true;
    }

    return success;
}

} // learningmachine
} // iCub


