/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation of the linear scaling transformer.
 *
 */

#include <cassert>
#include <sstream>

#include "iCub/ScaleTransformer.h"
#include "iCub/FactoryT.h"

namespace iCub {
namespace learningmachine {

ScaleTransformer::ScaleTransformer(int dom) {
    this->setName("Scaler");
    this->setDomainSize(dom);
}

ScaleTransformer::~ScaleTransformer() {
    this->deleteAll();
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
        // the magic keyword null specifies that no scaler object will be created
        // this is useful to disable the scaler with minimal overhead
        if(type != "null") {
            this->scalers[index] = FactoryT<std::string, IScaler>::instance().clone(type);
        }
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

bool ScaleTransformer::isEmptyScaler(int index) {
    if(index < this->scalers.size()) {
        if(this->scalers[index] == (IScaler*) 0) {
            return true;
        } else {
            return false;
        }
    } else {
        return true;
    }
}

Vector ScaleTransformer::transform(const Vector& input) {
    Vector output = this->IFixedSizeTransformer::transform(input);
    assert(input.size() == this->scalers.size());
    assert(output.size() == this->scalers.size());

    for(int i = 0; i < output.size(); i++) {
        // only transform the sample if the scaler is actually set
        if(!this->isEmptyScaler(i)) {
            output(i) = this->getAt(i)->transform(input(i));
        } else {
            output(i) = input(i);
        }
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
        if(!this->isEmptyScaler(i)) {
            buffer << this->scalers[i]->getInfo();
        } else {
            buffer << "null";
        }
        buffer << std::endl;
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
        if(this->isEmptyScaler(i)) {
            bot.addString("null");
        } else {
            bot.addString(this->getAt(i)->toString().c_str());
            bot.addString(this->getAt(i)->getName().c_str());
        }
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
        if(!this->isEmptyScaler(i)) {
            this->getAt(i)->fromString(bot.pop().asString().c_str());
        }
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
            if(!this->isEmptyScaler(i)) {
                success = this->getAt(i)->configure(property);
            }
        } else if(list.get(0).asString() == "all") {
            // format: set config all key val
            for(int i = 0; i < this->scalers.size(); i++) {
                if(!this->isEmptyScaler(i)) {
                    success |= this->getAt(i)->configure(property);
                }
            }
        } else {
            throw std::runtime_error("Illegal index!");
        }
    }


    return success;
}


} // learningmachine
} // iCub

