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

#include "iCub/IEventListener.h"


/**
 * The EventDispatcher manages the relation between the various instances of 
 * IEventListeners and IEvents. It allows IEvents to get raised and dispatches 
 * these to the registered IEventListeners. Internally, the EventDispatcher uses 
 * a double dispatching mechanism that allows extension of both IEventListeners 
 * and IEvents.
 *
 * \see iCub::contrib::learningmachine::IEventListener
 * \see iCub::contrib::learningmachine::IEvent
 *
 * \author Arjan Gijsberts
 *
 */

class EventDispatcher {
private:
    std::list<IEventListener*> listeners;

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
     * Adds an IEventListener to the list. Note that the EventDispatcher takes
     * over responsibility of the pointer and its deconstruction.
     * @param  listener The IEventListener that is to be add.
     */
    virtual void addListener(IEventListener* listener) {
        this->listeners.push_back(listener);
    }

    /**
     * Removes an IEventListener from the list.
     * @param  idx The index of the IEventListener that is to be removed.
     */
    virtual void removeListener(int idx);

    /**
     * Returns the IEventListener at a specified index.
     * @return the IEventListener
     * @param  idx The index of the IEventListener.
     */
    virtual IEventListener* getListener(int idx);
    
    /**
     * Clears all the IEventListeners from the EventDispatcher.
     */
    virtual void clear();

    /**
     * Raises an IEvent, causing it to be dispatched to each registered
     * IEventListener. Note that a double dispatching mechanism is used to 
     * determine the proper runtime types of both the IEvent and the 
     * IEventListener using dynamic binding.
     * @param  event The IEvent instance that is to be raised.
     */
    virtual void raise(IEvent& event);

    /**
     * @return true if there are one of more registered IEventListeners.
     */
    virtual bool hasListeners() {
        return (!this->listeners.empty());
    }
};

#endif
