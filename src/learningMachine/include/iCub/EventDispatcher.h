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

namespace iCub {
namespace contrib {
namespace learningmachine {


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
    /**
     * The list of IEventListeners.
     */
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
     * An instance retrieval method that follows the Singleton pattern.
     *
     * Note that this implementation should not be considered thread safe and 
     * that problems may arise. However, due to the nature of the expected use 
     * this will not be likely to result in problems.
     *
     * See http://www.oaklib.org/docs/oak/singleton.html for more information.
     *
     * @return the singleton factory instance
     */
    static EventDispatcher& instance() {
        static EventDispatcher instance;
        return instance;
    }

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
     * Removes an IEventListener from the list. This does _not_ delete the 
     * object itself. As the caller has access to the pointer, it can take 
     * responsibility for the proper destruction of the object.
     * @param  listener The IEventListener instance that is to be removed.
     */
    virtual void removeListener(IEventListener* listener);

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

} // learningmachine
} // contrib
} // iCub

#endif
