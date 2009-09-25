/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * LSSVM Learner implementation for using LibLSSVM over ports.
 *
 */

#include "iCub/LSSVMAtlasLearner.h"
#include "iCub/MercerKernels.h"
#include <cassert>
#include <sstream>
//#include <stdexcept>
#include <iostream>

namespace iCub {
namespace contrib {
namespace learningmachine {

bool LSSVMAtlasLearner::kernelsRegistered = false;

LSSVMAtlasLearner::LSSVMAtlasLearner(std::string name, int size, int type, std::string kernel) : IFixedSizeLearner(name) {
    this->defaultType = type;
    this->defaultKernel = kernel;
    this->sampleCount = 0;
    if(!LSSVMAtlasLearner::kernelsRegistered) {
        LSSVMAtlasLearner::kernelsRegistered = true;
        liblssvm::registerKernels();
    }

    this->initAll(size);
}

LSSVMAtlasLearner::~LSSVMAtlasLearner() {
    this->deleteAll();
}

void LSSVMAtlasLearner::deleteAll() {
    this->deleteAll(this->machines.size());
}

void LSSVMAtlasLearner::deleteAll(int size) {
    for(int i = 0; i < this->machines.size(); i++) {
        this->deleteAt(i);
    }
    this->machines.clear();
    this->machines.resize(size);
}

void LSSVMAtlasLearner::deleteAt(int index) {
    assert(index < this->machines.size());

    if(this->machines[index] != (LSSVM *) 0) {
        delete this->machines[index]->getKernel();
    }
    delete this->machines[index];
    this->machines[index] = 0;
}

void LSSVMAtlasLearner::initAll() {
    this->initAll(this->machines.size());
}

void LSSVMAtlasLearner::initAll(int size) {
    // clear current vector and set to correct size
    this->deleteAll(size);
    // create new machines
    for(int i = 0; i < this->machines.size(); i++) {
        this->machines[i] = this->createMachine(this->defaultType);
        this->setKernelAt(i, this->defaultKernel);
    }
}

void LSSVMAtlasLearner::setKernelAll(std::string kernel) {
    for(int i = 0; i < this->machines.size(); i++) {
        this->setKernelAt(i, kernel);
    }
}

void LSSVMAtlasLearner::setKernelAt(int index, std::string kernel) {
    if(index < this->machines.size()) {
        delete this->machines[index]->getKernel();
        this->machines[index]->setKernel(Kernel::fromString(kernel));
    } else {
        throw std::runtime_error("Index out of bounds!");
    }
}

void LSSVMAtlasLearner::setCAll(double c) {
    for(int i = 0; i < this->machines.size(); i++) {
        this->setCAt(i, c);
    }
}

void LSSVMAtlasLearner::setCAt(int index, double c) {
    if(index < this->machines.size()) {
        this->machines[index]->setC(c);
    } else {
        throw std::runtime_error("Index out of bounds!");
    }
}


LSSVM* LSSVMAtlasLearner::createMachine(int type) {
    switch(type) {
        case REFERENCE:
            return new ReferenceLSSVM();
            break;

        case PARTIAL:
            return new PartialLSSVM();
            break;

        case FULL:
        default:
            return new LSSVM();
            break;
    }
}

void LSSVMAtlasLearner::feedSample(const Vector& input, const Vector& output) {
    // call parent method to let it do some validation and processing with the transformers for us
    this->IFixedSizeLearner::feedSample(input, output);

    this->inputs.push_back(input);
    this->outputs.push_back(output);
}

void LSSVMAtlasLearner::train() {
    assert(this->inputs.size() == this->outputs.size());

    // save wasting some time
    if(inputs.size() == 0) {
        return;
    }

    // create input sample vectors using transformation
    std::vector<liblssvm::lssvm_vector> lssvmInputs(inputs.size());
    std::vector<std::vector<double> > lssvmOutputs(this->getCoDomainSize());

    // resize each output vector (one vector per output)
    for(int c = 0; c < this->getCoDomainSize(); c++) {
        lssvmOutputs[c].resize(this->outputs.size());
    }

    for(int i = 0; i < this->inputs.size(); i++) {
        // convert transformed input to lssvm_vector and add to array
        lssvmInputs[i].resize(this->inputs[i].size());
        for(int j = 0; j < this->inputs[i].size(); j++) {
            lssvmInputs[i][j] = this->inputs[i][j];
        }

        // add output label to appropriate vector
        for(int c = 0; c < lssvmOutputs.size(); c++) {
            lssvmOutputs[c][i] = this->outputs[i][c];
        }
    }

    // feed to machines
    for(int c = 0; c < this->getCoDomainSize(); c++) {
        //std::cout << "Sending outputs " << c << " of size " << lssvmOutputs[c].size() << std::endl;
        this->machines[c]->train(lssvmInputs, lssvmOutputs[c]);
    }

    this->sampleCount = lssvmInputs.size();
}

Vector LSSVMAtlasLearner::predict(const Vector& input) {
    this->checkDomainSize(input);

    Vector output(this->getCoDomainSize());
    
    // convert input to lssvm_vector type
    liblssvm::lssvm_vector lssvmInput(input.size());
    for(int i = 0; i < lssvmInput.size(); i++) {
        lssvmInput[i] = input[i];
    }

    // feed to machines
    for(int c = 0; c < this->getCoDomainSize(); c++) {
        output[c] = this->machines[c]->predict(lssvmInput);
    }

    return output;
}

void LSSVMAtlasLearner::reset() {
    this->deleteAll();
    this->inputs.clear();
    this->outputs.clear();
    this->initAll();
}

IMachineLearner* LSSVMAtlasLearner::create() {
    return new LSSVMAtlasLearner(this->getName(), this->machines.size(), this->defaultType, this->defaultKernel);
}

std::string LSSVMAtlasLearner::getStats() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getStats();
    buffer << "Machines: " << std::endl;
    for(int i = 0; i < this->machines.size(); i++) {
        buffer << "  [" << (i + 1) << "] ";
        buffer << this->machines[i]->toString() << " | Kernel: " << this->machines[i]->getKernel()->toString();
        buffer << " | LOO: " << this->machines[i]->getLOO();
        buffer << std::endl;
    }
    buffer << "Collected Samples: " << this->inputs.size() << std::endl;
    buffer << "Training Samples: " << this->sampleCount << std::endl;
    return buffer.str();
}

std::string LSSVMAtlasLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getConfigHelp();
    buffer << "  kernel idx|all cfg    Kernel configuration" << std::endl;
    buffer << "  c idx|all val         Tradeoff parameter C" << std::endl;
    return buffer.str();
}

void LSSVMAtlasLearner::writeBottle(Bottle& bot) {
    for(int i = 0; i < this->getCoDomainSize(); i++) {
        std::ostringstream oStream;
        this->machines[i]->save(oStream);
        bot.addString(oStream.str().c_str());
    }
    bot.addInt(this->sampleCount);
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void LSSVMAtlasLearner::readBottle(Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    this->sampleCount = bot.pop().asInt();
    for(int i = this->getCoDomainSize() - 1; i >=0 ; i--) {
        std::istringstream iStream(bot.pop().asString().c_str());
        this->machines[i]->load(iStream);
    }
}

void LSSVMAtlasLearner::setDomainSize(int size) {
    this->IFixedSizeLearner::setDomainSize(size);
}

void LSSVMAtlasLearner::setCoDomainSize(int size) {
    this->IFixedSizeLearner::setCoDomainSize(size);
    this->initAll(this->getCoDomainSize());
}

bool LSSVMAtlasLearner::configure(Searchable& config) {
    bool success = this->IFixedSizeLearner::configure(config);

    // format: set kernel KernelConfig
/*    if(config.find("kernel").isString()) {
        this->setKernelAll(config.find("kernel").asString().c_str());
        success = true;
    }*/

    // format: set kernel (KernelConfig1 .. KernelConfign)
    if(config.find("kernel").isList()) {
        Bottle* transList = config.find("kernel").asList();
        for(int i = 0; i < transList->size(); i++) {
            if(transList->get(i).isString()) {
                this->setKernelAt(i, transList->get(i).asString().c_str());
                success = true;
            }
            // NOTE TO SELF: consider throwing an exception in the else clause
        }
    }

    // format: set kernel idx|all KernelConfig
    if(!config.findGroup("kernel").isNull()) {
        //success = true;
        Bottle list = config.findGroup("kernel").tail();
        if(list.get(0).isInt() && list.get(1).isString()) {
            // shift index, since internal numbering in vector starts at 0, the user starts at 1
            this->setKernelAt(list.get(0).asInt() - 1, list.get(1).asString().c_str());
            success = true;
        } else if(list.get(0).asString() == "all" && list.get(1).isString()) {
            this->setKernelAll(list.get(1).asString().c_str());
            success = true;
        }
    }

    // format: set c (dbl .. dbl)
    if(config.find("c").isList()) {
        Bottle* transList = config.find("c").asList();
        for(int i = 0; i < transList->size(); i++) {
            if(transList->get(i).isDouble() || transList->get(i).isInt()) {
                this->setCAt(i, transList->get(i).asDouble());
                success = true;
            }
            // NOTE TO SELF: consider throwing an exception in the else clause
        }
    }

    // format: set c idx|all dbl
    if(!config.findGroup("c").isNull()) {
        //success = true;
        Bottle list = config.findGroup("c").tail();
        if(list.get(0).isInt() && (list.get(1).isDouble() || list.get(1).isInt())) {
            // shift index, since internal numbering in vector starts at 0, the user starts at 1
            this->setCAt(list.get(0).asInt() - 1, list.get(1).asDouble());
            success = true;
        } else if(list.get(0).asString() == "all" && (list.get(1).isDouble() || list.get(1).isInt())) {
            this->setCAll(list.get(1).asDouble());
            success = true;
        }
    }

    return success;
}

} // learningmachine
} // contrib
} // iCub


