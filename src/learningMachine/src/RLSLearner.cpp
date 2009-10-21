/*
 * Copyright (C) 2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * *TEMPORARY* implementation of Regularized Least Squares (aka Ridge Regression) 
 * Learner. 
 *
 */

#include "iCub/RLSLearner.h"
#include <cassert>
//#include <sstream>
//#include <stdexcept>
#include <iostream>

namespace iCub {
namespace contrib {
namespace learningmachine {

RLSLearner::RLSLearner(int size) {
    this->setName("RLS");
    this->sampleCount = 0;
    this->initAll(size);
}

RLSLearner::~RLSLearner() {
    this->deleteAll();
}

void RLSLearner::deleteAll() {
    this->deleteAll(this->machines.size());
}

void RLSLearner::deleteAll(int size) {
    for(int i = 0; i < this->machines.size(); i++) {
        this->deleteAt(i);
    }
    this->machines.clear();
    this->machines.resize(size);
}

void RLSLearner::deleteAt(int index) {
    assert(index < this->machines.size());

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
    for(int i = 0; i < this->machines.size(); i++) {
        this->machines[i] = this->createMachine();
    }
}

void RLSLearner::setLambdaAll(double l) {
    for(int i = 0; i < this->machines.size(); i++) {
        this->setLambdaAt(i, l);
    }
}

void RLSLearner::setLambdaAt(int index, double l) {
    if(index < this->machines.size()) {
        this->machines[index]->setLambda(l);
        this->machines[index]->reset();
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

IMachineLearner* RLSLearner::create() {
    return new RLSLearner(this->machines.size());
}

std::string RLSLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    buffer << "Machines: " << std::endl;
    for(int i = 0; i < this->machines.size(); i++) {
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
        std::ostringstream oStream;
        //this->machines[i]->save(oStream);
        bot.addString(oStream.str().c_str());
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
        std::istringstream iStream(bot.pop().asString().c_str());
        //this->machines[i]->load(iStream);
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

    // format: set lambda (KernelConfig1 .. KernelConfig)
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
} // contrib
} // iCub


