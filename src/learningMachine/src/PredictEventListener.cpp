/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the standard event listener for predict events.
 *
 */

#include <yarp/os/Bottle.h>

#include "iCub/PredictEventListener.h"

namespace iCub {
namespace learningmachine {

void PredictEventListener::handle(PredictEvent& e) {
    Bottle b;
    this->vectorToBottle(e.getInput(), b.addList());
    this->vectorToBottle(e.getPredicted(), b.addList());
    this->port.write(b);
}

} // learningmachine
} // iCub
