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
#include <algorithm>
#include <math.h>

#include <yarp/math/Rand.h>

#include "iCub/learningMachine/SparseSpectrumFeature.h"

//#define TWOPI 6.2831853071795862

using namespace yarp::math;

namespace iCub {
namespace learningmachine {

SparseSpectrumFeature::SparseSpectrumFeature(unsigned int dom, unsigned int cod, double sigma, Vector ell) {
    this->setName("SparseSpectrumFeature");
    this->setDomainSize(dom);
    this->setCoDomainSize(cod);
    // will initiate reset automatically
    this->setSigma(sigma);
    this->setEll(ell);
}

Vector SparseSpectrumFeature::transform(const Vector& input) {
    //Vector output = Vector
    Vector output = this->IFixedSizeTransformer::transform(input);

    // python: x_f = numpy.cos(numpy.dot(self.W, x) + self.bias) / math.sqrt(self.nproj)
    Vector inputW = (this->W * input);
    double factor = this->sigma / sqrt(this->getCoDomainSize() / 2);
    //double nprojsq = sqrt((double)this->getCoDomainSize());
    for(int i = 0; i < output.size(); i+=2) {
        output(i) = cos(inputW(i/2)) * factor;
        output(i+1) = sin(inputW(i/2)) * factor;
    }
    return output;
}

void SparseSpectrumFeature::setDomainSize(unsigned int size) {
    // call method in base class
    this->IFixedSizeTransformer::setDomainSize(size);
    // rebuild projection matrix
    this->reset();
}

void SparseSpectrumFeature::setCoDomainSize(unsigned int size) {
    assert (size & 0x1 == 0);
    // call method in base class
    this->IFixedSizeTransformer::setCoDomainSize(size);
    // rebuild projection matrix
    this->reset();
}

void SparseSpectrumFeature::reset() {
    this->IFixedSizeTransformer::reset();

    // create pseudo random number generators
    yarp::math::RandnScalar prng_normal;

    // create new projection matrix
    this->W.resize(this->getCoDomainSize() / 2, this->getDomainSize());

    // fill projection matrix with random and scaled data
    for(int r = 0; r < this->W.rows(); r++) {
        for(int c = 0; c < this->W.cols(); c++) {
            this->W(r, c) = prng_normal.get() / this->ell(c);
        }
    }
}

void SparseSpectrumFeature::writeBottle(Bottle& bot) {
    // write sigma
    bot.addDouble(this->getSigma());

    // write length scales ell
    for(int i = 0; i < this->ell.size(); i++) {
        bot.addDouble(this->ell(i));
    }
    bot.addInt(this->ell.size());

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

void SparseSpectrumFeature::readBottle(Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeTransformer::readBottle(bot);

    // read matrix Wyhj
    this->W.resize(bot.pop().asInt(), bot.pop().asInt());
    for(int r = this->W.rows() - 1; r >= 0; r--) {
        for(int c = this->W.cols() - 1; c >= 0; c--) {
            this->W(r, c) = bot.pop().asDouble();
        }
    }

    // read length scales ell
    this->ell.resize(bot.pop().asInt());
    for(int i = this->ell.size() - 1; i >= 0; i--) {
        this->ell(i) = bot.pop().asDouble();
    }

    // do _not_ use public accessor, as it resets the matrix
    this->sigma = bot.pop().asDouble();
}

void SparseSpectrumFeature::setEll(Vector& ell) {
    Vector ls = Vector(this->getDomainSize());
    ls = 1.;
    for(unsigned int i = 0; i < std::min(ell.size(), ls.size()); i++) {
        ls(i) = ell(i);
    }
    this->ell = ls;
    // rebuild projection matrix
    this->reset();
}

std::string SparseSpectrumFeature::getInfo() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeTransformer::getInfo();
    buffer << " sigma: " << this->sigma << " | ";
    buffer << " ell: " << this->ell.toString();
    return buffer.str();
}

std::string SparseSpectrumFeature::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeTransformer::getConfigHelp();
    buffer << "  sigma val             Set sigma parameter" << std::endl;
    buffer << "  ell (list)            Set lambda parameter" << std::endl;
    return buffer.str();
}

bool SparseSpectrumFeature::configure(Searchable &config) {
    bool success = this->IFixedSizeTransformer::configure(config);

    // format: set sigma val
    if(config.find("sigma").isDouble() || config.find("sigma").isInt()) {
        this->setSigma(config.find("sigma").asDouble());
        success = true;
    }

    // format: set ell val | set ell (val ... val)
    if(config.find("ell").isDouble() || config.find("ell").isInt()) {
        Vector ls = Vector(this->getDomainSize());
        ls = config.find("ell").asDouble();
        this->setEll(ls);
        success = true;
    } else if(config.find("ell").isList()) {
        Bottle* b = config.find("ell").asList();
        assert(b != 0x0);
        Vector ls = Vector(0);
        for(int i = 0; i < b->size(); i++) {
            if(b->get(i).isDouble() || b->get(i).isInt()) {
                ls.push_back(b->get(i).asDouble());
            }
        }
        this->setEll(ls);
        success = true;
    }
    return success;
}


} // learningmachine
} // iCub

