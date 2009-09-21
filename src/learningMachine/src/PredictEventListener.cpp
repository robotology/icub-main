/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the standard event listener for predict events.
 *
 */

#include "iCub/PredictEventListener.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


PredictEventListener::PredictEventListener(std::string name) : IEventListener(name) {
}

PredictEventListener::~PredictEventListener() {
}

} // learningmachine
} // contrib
} // iCub
