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
 * a double dispatching mechanism.
 *
 * \see iCub::contrib::learningmachine::EventListener
 * \see iCub::contrib::learningmachine::EventListener
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
     * over responsibility of the pointer and thus its deconstruction.
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
     * @return EventListener
     * @param  idx The index of the EventListener.
     */
    virtual EventListener* getListener(int idx);
    
    /**
     * Clears all the EventListeners.
     */
    virtual void clear();

    /**
     * Raises an Event, causing this Event to be dispatched to each registered
     * EventListener.
     * @param  event The Event instance that is to be raised.
     */
    virtual void raise(Event& event);

    /**
     * @return bool
     */
    virtual bool hasListener() {
        return (!this->listeners.empty());
    }

    /**
     * Set the value of listeners
     * A container for the list of EventListeners.
     * @param new_var the new value of listeners
     */
    /*void setListeners(std::list<EventListener> new_var) {
        listeners = new_var;
    }*/

    /**
     * Get the value of listeners
     * A container for the list of EventListeners.
     * @return the value of listeners
     */
    /*std::list<EventListener> getListeners() {
        return listeners;
    }*/
};

#endif
