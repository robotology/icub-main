/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for train events.
 *
 */

#include "iCub/TrainEvent.h"

namespace iCub {
namespace contrib {
namespace learningmachine {

TrainEvent::TrainEvent() {
}

TrainEvent::~TrainEvent() {
}

void TrainEvent::visit(IEventListener& listener) {
    listener.handle(*this);
}

} // learningmachine
} // contrib
} // iCub
