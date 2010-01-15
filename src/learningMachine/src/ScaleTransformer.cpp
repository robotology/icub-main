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
#include <sstream>

#include "iCub/ScaleTransformer.h"
#include "iCub/FactoryT.h"

namespace iCub {
namespace learningmachine {

ScaleTransformer::ScaleTransformer(int dom) : IFixedSizeTransformer(dom, dom) {
    this->setName("Scaler");
    this->setDomainSize(dom);
}

ScaleTransformer::~ScaleTransformer() {
    this->deleteAll();
}

ScaleTransformer::ScaleTransformer(const ScaleTransformer& other)
  : IFixedSizeTransformer(other) {
    this->scalers.resize(other.scalers.size());
    for(int i = 0; i < other.scalers.size(); i++) {
        this->scalers[i] = other.scalers[i]->clone();
    }
}

ScaleTransformer& ScaleTransformer::operator=(const ScaleTransformer& other) {
    if(this == &other) return *this; // handle self initialization

    this->IFixedSizeTransformer::operator=(other);

    this->deleteAll(other.scalers.size());
    for(int i = 0; i < other.scalers.size(); i++) {
        this->scalers[i] = other.scalers[i]->clone();
    }

    return *this;
}

void ScaleTransformer::deleteAll() {
    this->deleteAll(this->scalers.size());
}

void ScaleTransformer::deleteAll(int size) {
    for(std::vector<IScaler*>::iterator it = this->scalers.begin(); it != this->scalers.end(); it++) {
        delete *it;
    }
    this->scalers.clear();
    this->scalers.resize(size);
    this->setAll("null");
}

void ScaleTransformer::setAt(int index, std::string type) {
    if(index >= 0 && index < this->scalers.size()) {
        delete this->scalers[index];
        this->scalers[index] = (IScaler *) 0;
        this->scalers[index] = FactoryT<std::string, IScaler>::instance().create(type);
    } else {
        throw std::runtime_error("Index for scaler out of bounds!");
    }
}

void ScaleTransformer::setAll(std::string type) {
    //this->clearVector();
    for(int i = 0; i < this->scalers.size(); i++) {
        this->setAt(i, type);
    }
}

Vector ScaleTransformer::transform(const Vector& input) {
    Vector output = this->IFixedSizeTransformer::transform(input);
    assert(input.size() == this->scalers.size());
    assert(output.size() == this->scalers.size());

    for(int i = 0; i < output.size(); i++) {
        output(i) = this->getAt(i)->transform(input(i));
    }
    return output;
}

void ScaleTransformer::setDomainSize(int size) {
    // domain size and codomain have to be equally sized
    this->IFixedSizeTransformer::setDomainSize(size);
    this->IFixedSizeTransformer::setCoDomainSize(size);
    this->reset();
}

void ScaleTransformer::setCoDomainSize(int size) {
    // domain size and codomain have to be equally sized
    this->setDomainSize(size);
}

void ScaleTransformer::reset() {
    this->ITransformer::reset();
    this->deleteAll(this->getDomainSize());
}

std::string ScaleTransformer::getInfo() {

    std::ostringstream buffer;
    buffer << this->IFixedSizeTransformer::getInfo();
    buffer << "Scalers:" << std::endl;
    for(int i = 0; i < this->scalers.size(); i++) {
        buffer << "  [" << (i + 1) << "] ";
        buffer << this->scalers[i]->getInfo() << std::endl;
    }
    return buffer.str();
}

std::string ScaleTransformer::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IFixedSizeTransformer::getConfigHelp();
    buffer << "  type idx|all id       Scaler type" << std::endl;
    buffer << "  config idx|all key v  Set scaler configuration option" << std::endl;
    //buffer << "  update idx|all        Toggles updating on data on or off" << std::endl;
    return buffer.str();
}

void ScaleTransformer::writeBottle(Bottle& bot) {
    // write all scalers
    for(int i = 0; i < this->getDomainSize(); i++) {
        bot.addString(this->getAt(i)->toString().c_str());
        bot.addString(this->getAt(i)->getName().c_str());
    }

    // make sure to call the superclass's method
    this->IFixedSizeTransformer::writeBottle(bot);
}

void ScaleTransformer::readBottle(Bottle& bot) {
    // make sure to call the superclass's method (will reset transformer)
    this->IFixedSizeTransformer::readBottle(bot);

    // read all scalers in reverse order
    for(int i = this->getDomainSize() - 1; i >= 0; i--) {
        this->setAt(i, bot.pop().asString().c_str());
        this->getAt(i)->fromString(bot.pop().asString().c_str());
    }
}

bool ScaleTransformer::configure(Searchable &config) {
    bool success = this->IFixedSizeTransformer::configure(config);

    // format: set type (ScalerName ScalerName)
    if(config.find("type").isList()) {
        Bottle* scaleList = config.find("type").asList();
        for(int i = 0; i < scaleList->size(); i++) {
            if(scaleList->get(i).isString()) {
                this->setAt(i, scaleList->get(i).asString().c_str());
                success = true;
            }
            // NOTE TO SELF: consider throwing an exception in the else clause
        }
    }

    // format: set type idx|all ScalerName
    if(!config.findGroup("type").isNull()) {
        //success = true;
        Bottle list = config.findGroup("type").tail();
        if(list.get(0).isInt() && list.get(1).isString()) {
            // shift index, since internal numbering in vector starts at 0, the user starts at 1
            this->setAt(list.get(0).asInt() - 1, list.get(1).asString().c_str());
            success = true;
        } else if(list.get(0).asString() == "all" && list.get(1).isString()) {
            this->setAll(list.get(1).asString().c_str());
            success = true;
        }
    }

    // format: set config idx|all key val
    if(!config.findGroup("config").isNull()) {
        Bottle property;
        Bottle list = config.findGroup("config").tail();
        property.addList() = list.tail();
        if(list.get(0).isInt()) {
            // format: set config idx key val
            int i = list.get(0).asInt() - 1;
            success = this->getAt(i)->configure(property);
        } else if(list.get(0).asString() == "all") {
            // format: set config all key val
            for(int i = 0; i < this->scalers.size(); i++) {
                success |= this->getAt(i)->configure(property);
            }
        }
    }

    return success;
}


} // learningmachine
} // iCub

