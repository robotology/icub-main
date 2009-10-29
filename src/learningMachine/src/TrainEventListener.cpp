/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the standard event listener for train events.
 *
 */

#include <yarp/os/Bottle.h>

#include "iCub/TrainEventListener.h"

namespace iCub {
namespace learningmachine {


void TrainEventListener::handle(TrainEvent& e) {
    Bottle b;
    this->vectorToBottle(e.getInput(), b.addList());
    this->vectorToBottle(e.getDesired(), b.addList());
    this->vectorToBottle(e.getPredicted(), b.addList());
    this->port.write(b);
}


} // learningmachine
} // iCub
