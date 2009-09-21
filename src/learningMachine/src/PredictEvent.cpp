/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for predict events.
 *
 */

#include "iCub/PredictEvent.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


PredictEvent::PredictEvent() {
}

PredictEvent::~PredictEvent() {
}

void PredictEvent::visit(IEventListener& listener) {
    listener.handle(*this);
}

} // learningmachine
} // contrib
} // iCub
