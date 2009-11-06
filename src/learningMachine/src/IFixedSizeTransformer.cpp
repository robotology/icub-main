/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the fixed size transformer interface.
 *
 */

#include <stdexcept>
#include <sstream>

#include "iCub/IFixedSizeTransformer.h"

namespace iCub {
namespace learningmachine {

Vector IFixedSizeTransformer::transform(const Vector& input) {
    Vector output = ITransformer::transform(input);
    output.resize(this->getCoDomainSize());
    this->validateDomainSizes(input, output);
    return output;
}

void IFixedSizeTransformer::validateDomainSizes(const Vector& input, const Vector& output) {
    if(!this->checkDomainSize(input)) {
        throw std::runtime_error("Input sample has invalid dimensionality");
    }
    if(!this->checkCoDomainSize(output)) {
        throw std::runtime_error("Output sample has invalid dimensionality");
    }
}

bool IFixedSizeTransformer::configure(Searchable& config) {
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

void IFixedSizeTransformer::writeBottle(Bottle& bot) {
    bot.addInt(this->getDomainSize());
    bot.addInt(this->getCoDomainSize());
}

void IFixedSizeTransformer::readBottle(Bottle& bot) {
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
