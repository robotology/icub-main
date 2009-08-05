/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * 
 *
 */

#include "iCub/ITransformer.h"

#include <sstream>

namespace iCub {
namespace contrib {
namespace learningmachine {

std::string ITransformer::getStats() {
    std::ostringstream buffer;
    buffer << "Type: " << this->getName() << ", ";
    buffer << "Sample Count: " << this->sampleCount << std::endl;
    return buffer.str();
}

} // learningmachine
} // contrib
} // iCub

