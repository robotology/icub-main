/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The abstract event class.
 *
 */

#ifndef __ICUB_EVENT__
#define __ICUB_EVENT__

#include <string>

#include "iCub/IEventListener.h"

/**
 * The abstract base class for all Event types.
 *
 * \author Arjan Gijsberts
 */

class IEvent {
protected:


public:


    /**
     * Returns a string representation of the Event.
     * @return string
     */
    virtual std::string toString() = 0;


    /**
     * Causes the Event to visit an EventListener. This method is part of the 
     * double dispatch mechanism. Child classes _need_ to override this 
     * function.
     * @param  listener
     */
    virtual void visit(IEventListener& listener) = 0;


};

#endif
