/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for predict events.
 *
 */

#include <sstream>

#include "iCub/PredictEvent.h"

namespace iCub {
namespace learningmachine {


PredictEvent::PredictEvent(Vector input, Vector predicted) {
    this->input = input;
    this->predicted = predicted;
}

PredictEvent::~PredictEvent() {
}

void PredictEvent::visit(IEventListener& listener) {
    listener.handle(*this);
}

std::string PredictEvent::toString() {
    std::ostringstream buffer;
    buffer << "Input: [" << this->input.toString() << "]  Predicted: [" << this->predicted.toString() << "]";
    return buffer.str();
}


} // learningmachine
} // iCub
