/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the standard event listener for train events.
 *
 */

#include "iCub/TrainEventListener.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

TrainEventListener::TrainEventListener(std::string name) : IEventListener(name) {
}

TrainEventListener::~TrainEventListener() {
}

} // learningmachine
} // contrib
} // iCub
