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

#include <stdexcept>
#include <sstream>

#include "iCub/learningMachine/IFixedSizeTransformer.h"

namespace iCub {
namespace learningmachine {

yarp::sig::Vector IFixedSizeTransformer::transform(const yarp::sig::Vector& input) {
    yarp::sig::Vector output = ITransformer::transform(input);
    output.resize(this->getCoDomainSize());
    this->validateDomainSizes(input, output);
    return output;
}

void IFixedSizeTransformer::validateDomainSizes(const yarp::sig::Vector& input, const yarp::sig::Vector& output) {
    if(!this->checkDomainSize(input)) {
        throw std::runtime_error("Input sample has invalid dimensionality");
    }
    if(!this->checkCoDomainSize(output)) {
        throw std::runtime_error("Output sample has invalid dimensionality");
    }
}

bool IFixedSizeTransformer::configure(yarp::os::Searchable& config) {
    bool success = false;
    // set the domain size (int)
    if(config.find("dom").isInt()) {
        this->setDomainSize(config.find("dom").asInt());
        success = true;
    }
    // set the codomain size (int)
    if(config.find("cod").isInt()) {
        this->setCoDomainSize(config.find("cod").asInt());
        success = true;
    }

    return success;
}

void IFixedSizeTransformer::writeBottle(yarp::os::Bottle& bot) {
    bot.addInt(this->getDomainSize());
    bot.addInt(this->getCoDomainSize());
}

void IFixedSizeTransformer::readBottle(yarp::os::Bottle& bot) {
    this->setCoDomainSize(bot.pop().asInt());
    this->setDomainSize(bot.pop().asInt());
}


std::string IFixedSizeTransformer::getInfo() {
    std::ostringstream buffer;
    buffer << this->ITransformer::getInfo();
    buffer << "Domain size: " << this->getDomainSize() << ", ";
    buffer << "Codomain size: " << this->getCoDomainSize() << std::endl;
    return buffer.str();
}

std::string IFixedSizeTransformer::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->ITransformer::getConfigHelp();
    buffer << "  dom size              Domain size" << std::endl;
    buffer << "  cod size              Codomain size" << std::endl;
    return buffer.str();
}


} // learningmachine
} // iCub
