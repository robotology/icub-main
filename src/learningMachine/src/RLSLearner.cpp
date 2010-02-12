/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#include "iCub/RLSLearner.h"

namespace iCub {
namespace learningmachine {

RLSLearner::RLSLearner(int dom, int cod, double lambda) {
    this->setName("RLS");
    this->sampleCount = 0;
    // make sure to not use initialization list to constructor of base for
    // domain and codomain size, as it will not use overloaded mutators
    this->setDomainSize(dom);
    // slightly inefficient to use mutators, as we are initializing twice
    this->setCoDomainSize(cod);
    this->setLambdaAll(lambda);
}

RLSLearner::RLSLearner(const RLSLearner& other)
  : IFixedSizeLearner(other), sampleCount(other.sampleCount) {
    this->machines.resize(other.machines.size());
    for(unsigned int i = 0; i < other.machines.size(); i++) {
        this->machines[i] = new RLS(*(other.machines[i]));
    }

}

RLSLearner::~RLSLearner() {
    this->deleteAll();
}

RLSLearner& RLSLearner::operator=(const RLSLearner& other) {
    if(this == &other) return *this; // handle self initialization

    this->IFixedSizeLearner::operator=(other);
    this->sampleCount = other.sampleCount;

    this->deleteAll(other.machines.size());
    for(unsigned int i = 0; i < other.machines.size(); i++) {
        this->machines[i] = new RLS(*(other.machines[i]));
    }

    return *this;
}

void RLSLearner::deleteAll() {
    this->deleteAll(this->machines.size());
}

void RLSLearner::deleteAll(int size) {
    for(unsigned int i = 0; i < this->machines.size(); i++) {
        this->deleteAt(i);
    }
    this->machines.clear();
    this->machines.resize(size);
}

void RLSLearner::deleteAt(int index) {
    assert(index < int(this->machines.size()));

    delete this->machines[index];
    this->machines[index] = (RLS*) 0;
}

void RLSLearner::initAll() {
    this->initAll(this->getCoDomainSize());
}

void RLSLearner::initAll(int size) {
    // clear current vector and set to correct size
    this->deleteAll(size);
    // create new machines
    for(unsigned int i = 0; i < this->machines.size(); i++) {
        this->machines[i] = this->createMachine();
    }
}

void RLSLearner::setLambdaAll(double l) {
    for(unsigned int i = 0; i < this->machines.size(); i++) {
        this->setLambdaAt(i, l);
    }
}

void RLSLearner::setLambdaAt(int index, double l) {
    RLS* machine = this->getAt(index);
    machine->setLambda(l);
    machine->reset();
}

RLS* RLSLearner::getAt(int index) {
    if(index >= 0 && index < int(this->machines.size())) {
        return this->machines[index];
    } else {
        throw std::runtime_error("Index out of bounds!");
    }
}


RLS* RLSLearner::createMachine() {
    return new RLS(this->getDomainSize());
}

void RLSLearner::feedSample(const Vector& input, const Vector& output) {
    this->IFixedSizeLearner::feedSample(input, output);

    for(int c = 0; c < output.size(); c++) {
        this->machines[c]->update(input, output(c));
    }

    this->sampleCount++;
}

void RLSLearner::train() {

}

Vector RLSLearner::predict(const Vector& input) {
    this->checkDomainSize(input);

    Vector output(this->getCoDomainSize());

    // feed to machines
    for(int c = 0; c < this->getCoDomainSize(); c++) {
        output[c] = this->machines[c]->predict(input);
    }

    return output;
}

void RLSLearner::reset() {
    this->sampleCount = 0;
    this->initAll();
}

std::string RLSLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    buffer << "Machines: " << std::endl;
    for(unsigned int i = 0; i < this->machines.size(); i++) {
        buffer << "  [" << (i + 1) << "] ";
        buffer << "lambda: " << this->machines[i]->getLambda();
        buffer << std::endl;
    }
    return buffer.str();
}

std::string RLSLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getConfigHelp();
    buffer << "  lambda idx|all val    Regularization parameter lambda" << std::endl;
    return buffer.str();
}

void RLSLearner::writeBottle(Bottle& bot) {
    for(int i = 0; i < this->getCoDomainSize(); i++) {
        bot.addString(this->getAt(i)->toString().c_str());
    }
    bot.addInt(this->sampleCount);
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void RLSLearner::readBottle(Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    this->sampleCount = bot.pop().asInt();
    for(int i = this->getCoDomainSize() - 1; i >=0 ; i--) {
        this->getAt(i)->fromString(bot.pop().asString().c_str());
    }
}

void RLSLearner::setDomainSize(int size) {
    this->IFixedSizeLearner::setDomainSize(size);

    this->initAll();
}

void RLSLearner::setCoDomainSize(int size) {
    this->IFixedSizeLearner::setCoDomainSize(size);
    this->initAll(this->getCoDomainSize());
}

/*bool RLSLearner::configureAt(Searchable& config) {
    if(config.find("lambda")
}*/

bool RLSLearner::configure(Searchable& config) {
    bool success = this->IFixedSizeLearner::configure(config);

    // format: set lambda (lambda1 .. lambdan)
    if(config.find("lambda").isList()) {
        Bottle* transList = config.find("lambda").asList();
        for(int i = 0; i < transList->size(); i++) {
            if(transList->get(i).isDouble() || transList->get(i).isInt()) {
                this->setLambdaAt(i, transList->get(i).asDouble());
                success = true;
            }
            // NOTE TO SELF: consider throwing an exception in the else clause
        }
    }

    // format: set lambda idx|all dbl
    if(!config.findGroup("lambda").isNull()) {
        //success = true;
        Bottle list = config.findGroup("lambda").tail();
        if(list.get(0).isInt() && (list.get(1).isDouble() || list.get(1).isInt())) {
            // shift index, since internal numbering in vector starts at 0, the user starts at 1
            this->setLambdaAt(list.get(0).asInt() - 1, list.get(1).asDouble());
            success = true;
        } else if(list.get(0).asString() == "all" && (list.get(1).isDouble() || list.get(1).isInt())) {
            this->setLambdaAll(list.get(1).asDouble());
            success = true;
        }
    }

    return success;
}

} // learningmachine
} // iCub


