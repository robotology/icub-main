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
#include <sstream>
#include <algorithm>
#include <cmath>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include "iCub/learningMachine/SparseSpectrumFeature.h"
#include "iCub/learningMachine/Math.h"
#include "iCub/learningMachine/Serialization.h"

//#define TWOPI 6.2831853071795862

using namespace yarp::math;
using namespace iCub::learningmachine::math;
using namespace iCub::learningmachine::serialization;

namespace iCub {
namespace learningmachine {

SparseSpectrumFeature::SparseSpectrumFeature(unsigned int dom, unsigned int cod, double sigma,
                                             yarp::sig::Vector ell) {
    this->setName("SparseSpectrumFeature");
    // ell has to be initialized *prior* to anything that could reset this instance
    this->setEll(ell);
    this->setDomainSize(dom);
    this->setCoDomainSize(cod);
    // will initiate reset automatically
    this->setSigma(sigma);
}

yarp::sig::Vector SparseSpectrumFeature::transform(const yarp::sig::Vector& input) {
    yarp::sig::Vector output = this->IFixedSizeTransformer::transform(input);

    yarp::sig::Vector inputW = (this->W * input);
    int nproj = this->getCoDomainSize() >> 1;
    double factor = this->sigma / sqrt((double)nproj);
    for(int i = 0; i < nproj; i++) {
        output(i)       = cos(inputW(i)) * factor;
        output(i+nproj) = sin(inputW(i)) * factor;
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
    if((size & 0x1) == 0) {
        // call method in base class
        this->IFixedSizeTransformer::setCoDomainSize(size);
        // rebuild projection matrix
        this->reset();
    } else {
        throw std::runtime_error("Size of codomain of Sparse Spectrum Features has to be even");
    }
}

void SparseSpectrumFeature::reset() {
    this->IFixedSizeTransformer::reset();

    // create pseudo random number generators
    yarp::math::RandnScalar prng_normal;

    // create new projection matrix
    this->W = yarp::sig::Matrix((int) this->getCoDomainSize() / 2, (int) this->getDomainSize());

    // fill projection matrix with random and scaled data
    for(int r = 0; r < this->W.rows(); r++) {
        for(int c = 0; c < this->W.cols(); c++) {
            this->W(r, c) = prng_normal.get() / this->ell(c);
        }
    }
}

void SparseSpectrumFeature::writeBottle(yarp::os::Bottle& bot) {
    bot << this->getSigma() << this->ell << this->W;

    // make sure to call the superclass's method
    this->IFixedSizeTransformer::writeBottle(bot);
}

void SparseSpectrumFeature::readBottle(yarp::os::Bottle& bot) {
    // make sure to call the superclass's method
    this->IFixedSizeTransformer::readBottle(bot);

    // directly write to sigma to prevent resetting W
    bot >> this->W >> this->ell >> this->sigma;
    this->setSigma(sigma);
}

void SparseSpectrumFeature::setEll(yarp::sig::Vector& ell) {
    yarp::sig::Vector ls = yarp::sig::Vector(this->getDomainSize());
    ls = 1.;
    for(unsigned int i = 0; (int)i < std::min(ell.size(), ls.size()); i++) {
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

bool SparseSpectrumFeature::configure(yarp::os::Searchable &config) {
    bool success = this->IFixedSizeTransformer::configure(config);

    // format: set sigma val
    if(config.find("sigma").isDouble() || config.find("sigma").isInt()) {
        this->setSigma(config.find("sigma").asDouble());
        success = true;
    }

    // format: set ell val | set ell (val ... val)
    if(config.find("ell").isDouble() || config.find("ell").isInt()) {
        yarp::sig::Vector ls = yarp::sig::Vector(this->getDomainSize());
        ls = config.find("ell").asDouble();
        this->setEll(ls);
        success = true;
    } else if(config.find("ell").isList()) {
        yarp::os::Bottle* b = config.find("ell").asList();
        assert(b != (yarp::os::Bottle*) 0x0);
        yarp::sig::Vector ls(0);
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

