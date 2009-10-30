/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Simple header file for including various known machines to the factory.
 *
 * Sorry for using conditional compilation for adding the different types; it's
 * hard to implement the factory pattern neatly in C++.
 */

#ifndef __ICUB_EVENTLISTENERCATALOGUE__
#define __ICUB_EVENTLISTENERCATALOGUE__

#include "iCub/EventListenerFactory.h"
#include "iCub/TrainEventListener.h"
#include "iCub/PredictEventListener.h"


namespace iCub {
namespace learningmachine {

void registerEventListeners() {
    EventListenerFactory::instance().registerPrototype(new TrainEventListener());
    EventListenerFactory::instance().registerPrototype(new PredictEventListener());
}

} // learningmachine
} // iCub

#endif
