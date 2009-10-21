/*
 * Copyright (C) 2007-2008 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * LSSVM Learner implementation for using LibLSSVM over ports.
 *
 */

#include <cassert>
#include <sstream>
//#include <stdexcept>
#include <iostream>

#include <yarp/math/SVD.h>

#include "iCub/LSSVMLearner.h"


namespace iCub {
namespace contrib {
namespace learningmachine {

LSSVMLearner::LSSVMLearner() {
    this->setName("LSSVM");
    this->kernel = new RBFKernel();
    this->setC(1.0);
}

LSSVMLearner::~LSSVMLearner() {
    delete this->kernel;
}


void LSSVMLearner::feedSample(const Vector& input, const Vector& output) {
    // call parent method to let it do some validation for us
    this->IFixedSizeLearner::feedSample(input, output);

    this->inputs.push_back(input);
    this->outputs.push_back(output);
}

void LSSVMLearner::train() {
    assert(this->inputs.size() == this->outputs.size());

    // save wasting some time
    if(inputs.size() == 0) {
        return;
    }
    
    // create kernel matrix
    Matrix K(inputs.size() + 1, inputs.size() + 1);
    for(int r = 0; r < K.rows() - 1; r++) {
        // symmetric matrix
        for(int c = 0; c <= r; c++) {
            K(r, c) = K(c, r) = this->kernel->evaluate(this->inputs[r], this->inputs[c]);
            if(r == c) K(r, c) += (1.0 / this->C);
        }
    }
    for(int i = 0; i < K.rows() - 1; i++) {
        K(i, K.cols() - 1) = K(K.rows() - 1, i) = 1.;
    }
    K(K.rows() - 1, K.cols() - 1) = 0.;
    
    // invert kernel matrix
    Matrix Kinv = luinv(K);
    
    // compute solution
    Matrix Y = zeros(this->outputs.size() + 1, this->getCoDomainSize());
    for(int r = 0; r < Y.rows() - 1; r++) {
        for(int c = 0; c < Y.cols(); c++) {
            Y(r, c) = this->outputs[r](c);
        }
    }
    
    Matrix result = Kinv * Y;
    this->alphas = result.submatrix(0, result.rows() - 2, 0, result.cols() - 1);
    this->bias = result.getRow(result.rows() - 1);
    
    // compute LOO
    this->LOO = zeros(this->getCoDomainSize());
    
    for(int i = 0; i < this->getCoDomainSize(); i++) {
        Vector alphas_i = this->alphas.getCol(i);
        for(int j = 0; j < alphas_i.size(); j++) {
            double err = alphas_i(j) / Kinv(j, j);
            this->LOO(i) += err * err;
        }
        this->LOO(i) /= alphas_i.size();
    }

}

Vector LSSVMLearner::predict(const Vector& input) {
    this->checkDomainSize(input);
    
    if(this->inputs.size() == 0) {
        return zeros(this->getCoDomainSize());
    }
    
    // compute kernel expansion
    Vector k(this->inputs.size());
    for(int i = 0; i < k.size(); i++) {
        k(i) = this->kernel->evaluate(this->inputs[i], input);
    }
    
    return (this->alphas.transposed() * k) + this->bias;
}

void LSSVMLearner::reset() {
    this->inputs.clear();
    this->outputs.clear();
    this->alphas = Matrix();
    this->LOO.clear();
    this->bias.clear();
}

IMachineLearner* LSSVMLearner::create() {
    return new LSSVMLearner();
}

std::string LSSVMLearner::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getInfo();
    buffer << "C: " << this->getC() << " | ";
    buffer << "Collected Samples: " << this->inputs.size() << " | ";
    buffer << "Training Samples: " << this->alphas.rows() << " | ";
    buffer << "Kernel: " << this->kernel->getInfo() << std::endl;
    buffer << "LOO: " << this->LOO.toString() << std::endl;
    return buffer.str();
}

std::string LSSVMLearner::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeLearner::getConfigHelp();
    //buffer << "  kernel idx|all cfg    Kernel configuration" << std::endl;
    buffer << "  c val                 Tradeoff parameter C" << std::endl;
    buffer << this->kernel->getConfigHelp() << std::endl;
    return buffer.str();
}

void LSSVMLearner::writeBottle(Bottle& bot) {
    // write kernel gamma
    bot.addDouble(this->kernel->getGamma());

    // write c
    bot.addDouble(this->getC());
    
    // write bias
    for(int i = 0; i < this->bias.size(); i++) {
        bot.addDouble(this->bias(i));    
    }
    bot.addInt(this->bias.size());
    
    // write alphas
    for(int r = 0; r < this->alphas.rows(); r++) {
        for(int c = 0; c < this->alphas.cols(); c++) {
            bot.addDouble(this->alphas(r, c));
        }
    }
    bot.addInt(this->alphas.rows());
    bot.addInt(this->alphas.cols());
    
    // write inputs
    for(int i = 0; i < this->inputs.size(); i++) {
        for(int d = 0; d < this->getDomainSize(); d++) {
            bot.addDouble(this->inputs[i](d));
        }
    }
    bot.addInt(this->inputs.size());
    
    // make sure to call the superclass's method
    this->IFixedSizeLearner::writeBottle(bot);
}

void LSSVMLearner::readBottle(Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeLearner::readBottle(bot);
    
    // read inputs
    this->inputs.resize(bot.pop().asInt());
    for(int i = this->inputs.size() - 1; i >= 0; i--) {
        this->inputs[i].resize(this->getDomainSize());
        for(int d = this->getDomainSize() - 1; d >= 0; d--) {
            this->inputs[i](d) = bot.pop().asDouble();
        }
    }

    // read alphas
    this->alphas.resize(bot.pop().asInt(), bot.pop().asInt());
    for(int r = this->alphas.rows() - 1; r >= 0; r--) {
        for(int c = this->alphas.cols() - 1; c >= 0; c--) {
            this->alphas(r, c) = bot.pop().asDouble();
        }
    }
    
    // read bias
    this->bias.resize(bot.pop().asInt());
    for(int i = this->bias.size() - 1; i >= 0; i--) {
        this->bias(i) = bot.pop().asDouble();
    }
    
    // read c
    this->setC(bot.pop().asDouble());
    
    // read gamma
    this->kernel->setGamma(bot.pop().asDouble());
}

void LSSVMLearner::setDomainSize(int size) {
    this->IFixedSizeLearner::setDomainSize(size);
}

void LSSVMLearner::setCoDomainSize(int size) {
    this->IFixedSizeLearner::setCoDomainSize(size);
    //this->initAll(this->getCoDomainSize());
}

bool LSSVMLearner::configure(Searchable& config) {
    bool success = this->IFixedSizeLearner::configure(config);

    // format: set c dbl
    if(config.find("c").isDouble() || config.find("c").isInt()) {
        double val = config.find("c").asDouble();
        if(val > 0) {
            this->setC(config.find("c").asDouble());
            success = true;
        }
    }
    
    success |= this->kernel->configure(config);
    
    return success;
}

} // learningmachine
} // contrib
} // iCub


