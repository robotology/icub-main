/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the event dispatcher.
 *
 */

#include <cassert>

#include "iCub/EventDispatcher.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


EventDispatcher::EventDispatcher() {
}

EventDispatcher::~EventDispatcher() {
    this->clear();
}


void EventDispatcher::removeListener(int idx) {
    assert(idx >= 0 && idx < this->listeners.size());
    std::list<IEventListener*>::iterator it = this->listeners.begin();
    std::advance(it, idx);
    delete *it;
    this->listeners.erase(it);
}

void EventDispatcher::removeListener(IEventListener* listener) {
    this->listeners.remove(listener);
    //delete listener;
}

IEventListener* EventDispatcher::getListener(int idx) {
    std::list<IEventListener*>::iterator it = this->listeners.begin();
    std::advance(it, idx);
    return *it;
}

void EventDispatcher::clear() {
    // remove from front while the list is not empty
    while(this->hasListeners()) {
        this->removeListener(0);
    }
}

void EventDispatcher::raise(IEvent& event) {
    std::list<IEventListener*>::iterator it;
    for(it = listeners.begin(); it != listeners.end(); it++) {
        event.visit(**it);
    }
}

} // learningmachine
} // contrib
} // iCub

