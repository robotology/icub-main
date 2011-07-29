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

#include <iostream>

#include "iCub/learningMachine/DummyLearner.h"
#include "iCub/learningMachine/Serialization.h"

using namespace iCub::learningmachine::serialization;

namespace iCub {
namespace learningmachine {

// just here for debugging, since Vector.toString() cannot be applied to const Vector :(
std::string printVector(const yarp::sig::Vector& v) {
  std::ostringstream output;
  output << "[";
  for(int i = 0; i < v.size(); i++) {
    if(i > 0) output << ",";
    output << v[i];
  }
  output << "]";
  return output.str();
}

void DummyLearner::feedSample(const yarp::sig::Vector& input, const yarp::sig::Vector& output) {
    // call parent method to let it do some validation and processing with the transformers for us
    this->IFixedSizeLearner::feedSample(input, output);

    std::cout << "Received a sample: " << printVector(input) << " => " << printVector(output) << std::endl;

    this->inputs.push_back(input);
    this->outputs.push_back(input);
}

void DummyLearner::train() {
    this->sampleCount = this->inputs.size();
    this->trainCount++;
}

Prediction DummyLearner::predict(const yarp::sig::Vector& input) {
    this->checkDomainSize(input);
    std::cout << "Received a prediction sample: " << printVector(input) << " => ";
    yarp::sig::Vector output = input;
    output.resize(this->getCoDomainSize());
    for(int i = 0; i < output.size(); i++)
        output[i] += this->sampleCount;
    std::cout << "(" << printVector(output) << ")" << std::endl;
    return Prediction(output);
}

void DummyLearner::reset() {
    this->sampleCount = 0;
    this->trainCount = 0;
    this->inputs.clear();
    this->outputs.clear();
}

std::string DummyLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "Training Samples: " << this->sampleCount << std::endl;
    buffer << "Training Iterations: " << this->trainCount << std::endl;
    buffer << "Collected Samples: " << this->inputs.size() << std::endl;
    return buffer.str();
}

void DummyLearner::writeBottle(yarp::os::Bottle& bot) {
    bot << this->sampleCount << this->trainCount;
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void DummyLearner::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    bot >> this->trainCount >> this->sampleCount;
}


} // learningmachine
} // iCub
