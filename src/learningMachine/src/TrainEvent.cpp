/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for train events.
 *
 */

#include <sstream>

#include "iCub/TrainEvent.h"

namespace iCub {
namespace learningmachine {

TrainEvent::TrainEvent(Vector input, Vector desired, Vector predicted) {
    this->input = input;
    this->desired = desired;
    this->predicted = predicted;
}

TrainEvent::~TrainEvent() {
}

void TrainEvent::visit(IEventListener& listener) {
    listener.handle(*this);
}

std::string TrainEvent::toString() {
    std::ostringstream buffer;
    buffer << "Input: [" << this->input.toString() << "; " << "]  Desired: [" << this->desired.toString() << "; " << "]  Predicted: [" << this->predicted.toString() << "]";
    return buffer.str();
}


} // learningmachine
} // iCub
