/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The event class interface.
 *
 */

#ifndef __ICUB_EVENT__
#define __ICUB_EVENT__

#include <string>

namespace iCub {
namespace learningmachine {

/**
 * The abstract base class for all Event types.
 *
 * \author Arjan Gijsberts
 */

// forward declaration to solve circular reference
class IEventListener;

class IEvent {
protected:


public:
    /**
     * Returns a string representation of the Event.
     * @return string  the string representation
     */
    virtual std::string toString() = 0;

    /**
     * Causes the Event to visit an EventListener. This method is part of the 
     * double dispatch mechanism. Child classes _need_ to override this 
     * function.
     * @param listener  the listener
     */
    virtual void visit(IEventListener& listener) = 0;

};

} // learningmachine
} // iCub

#endif
