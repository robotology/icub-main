/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Implementation for the event dispatcher.
 *
 */

#include "iCub/EventDispatcher.h"


EventDispatcher::EventDispatcher() {
}

EventDispatcher::~EventDispatcher() {
    this->clear();
}


void EventDispatcher::removeListener(int idx) {
    std::list<EventListener*>::iterator it = this->listeners.begin();
    std::advance(it, idx);
    delete *it;
    this->listeners.erase(it);
}

EventListener* EventDispatcher::getListener(int idx) {
    std::list<EventListener*>::iterator it = this->listeners.begin();
    std::advance(it, idx);
    return *it;
}

void EventDispatcher::clear() {
    // remove from front while the list is not empty
    while(this->hasListener()) {
        this->removeListener(0);
    }
}

void EventDispatcher::raise(Event& event) {
    std::list<EventListener*>::iterator it;
    for(it = listeners.begin(); it != listeners.end(); it++) {
        e.visit(**it);
    }
}
