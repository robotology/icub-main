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

#include <iomanip>
#include <sstream>

#include "iCub/DatasetRecorder.h"

namespace iCub {
namespace learningmachine {

DatasetRecorder& DatasetRecorder::operator=(const DatasetRecorder& other) {
    if(this == &other) return *this; // handle self initialization

    this->IMachineLearner::operator=(other);
    this->filename = other.filename;
    this->precision = other.precision;
    this->sampleCount = other.sampleCount;

    return *this;
}


void DatasetRecorder::feedSample(const Vector& input, const Vector& output) {
    // open stream if not opened yet
    if(!this->stream.is_open()) {
        // perhaps check if file already exists
        this->stream.open(this->filename.c_str(), std::ios_base::out | std::ios_base::app);
        // set precision
        this->stream.precision(this->precision);
    }

    // first write inputs
    for(int i = 0; i < input.size(); i++) {
        if(i > 0) this->stream << " ";
        this->stream << std::setw(this->precision + 4) << input[i];
    }
    this->stream << "  ";

    // then write outputs
    for(int i = 0; i < output.size(); i++) {
        if(i > 0) this->stream << " ";
        this->stream << std::setw(this->precision + 4) << output[i] << " ";
    }
    this->sampleCount++;

    this->stream << std::endl;

    this->stream.flush();
}


std::string DatasetRecorder::getInfo() {
    std::ostringstream buffer;
    buffer << this->IMachineLearner::getInfo();
    buffer << "Filename: " << this->filename << std::endl;
    buffer << "Precision: " << this->precision << std::endl;
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    return buffer.str();
}

void DatasetRecorder::writeBottle(Bottle& bot) {
    bot.addString(this->filename.c_str());
    bot.addInt(this->precision);
}

void DatasetRecorder::readBottle(Bottle& bot) {
    this->precision = bot.pop().asInt();
    this->filename = bot.pop().asString().c_str();
}

std::string DatasetRecorder::getConfigHelp() {
    std::ostringstream buffer;
    buffer << this->IMachineLearner::getConfigHelp();
    buffer << "  filename name         Filename to write to" << std::endl;
    buffer << "  precision n           Number of digits precision for doubles" << std::endl;
    return buffer.str();
}

bool DatasetRecorder::configure(Searchable& config) {
    bool success = false;

    // set the filename
    if(config.find("filename").isString()) {
        this->reset();
        this->filename = config.find("filename").asString().c_str();
        success = true;
    }

    // set the precision
    if(config.find("precision").isInt()) {
        this->precision = config.find("precision").asInt();
        success = true;
    }

    return success;
}

} // learningmachine
} // iCub
