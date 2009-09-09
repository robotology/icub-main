/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The event dispatching class.
 *
 */

#ifndef __ICUB_EVENTDISPATCHER__
#define __ICUB_EVENTDISPATCHER__

#include <list>

#include "iCub/EventListener.h"


/**
 * The EventDispatcher manages the relation between the various instances of 
 * EventListeners and Events. It allows Events to get raised and dispatches 
 * these to the registered EventListeners. Internally, the EventDispatcher uses 
 * a double dispatching mechanism that allows extension of both EventListeners 
 * and Events.
 *
 * \see iCub::contrib::learningmachine::EventListener
 * \see iCub::contrib::learningmachine::Event
 *
 * \author Arjan Gijsberts
 *
 */

class EventDispatcher {
private:
    std::list<EventListener*> listeners;

public:
    /**
     * Empty Constructor
     */
    EventDispatcher();

    /**
     * Empty Destructor
     */
    virtual ~EventDispatcher();

    /**
     * Adds an EventListener to the list. Note that the EventDispatcher takes
     * over responsibility of the pointer and its deconstruction.
     * @param  listener The EventListener that is to be add.
     */
    virtual void addListener(EventListener* listener) {
        this->listeners.push_back(listener);
    }

    /**
     * Removes an EventListener from the list.
     * @param  idx The index of the EventListener that is to be removed.
     */
    virtual void removeListener(int idx);

    /**
     * Returns the EventListener at a specified index.
     * @return the EventListener
     * @param  idx The index of the EventListener.
     */
    virtual EventListener* getListener(int idx);
    
    /**
     * Clears all the EventListeners from the EventDispatcher.
     */
    virtual void clear();

    /**
     * Raises an Event, causing this Event to be dispatched to each registered
     * EventListener. Note that a double dispatching mechanism is used to 
     * determine the proper runtime types of both the Event and the 
     * EventListener using dynamic binding.
     * @param  event The Event instance that is to be raised.
     */
    virtual void raise(Event& event);

    /**
     * @return true if there are one of more registered EventListeners.
     */
    virtual bool hasListeners() {
        return (!this->listeners.empty());
    }
};

#endif
