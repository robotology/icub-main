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

/**
  * class Event
  *
  */

class Event {
protected:


public:


    /**
     * Returns a string representation of the Event.
     * @return string
     */
    virtual std::string toString() = 0;


    /**
     * Causes the Event to visit an EventListener. This double dispatch strategy allows
     * for determination of runtime types of both the Event and the EventListener. For
     * this to work, child classes need to implement this function.
     * @param  listener
     */
    virtual void visit(EventListener listener) = 0;


};

#endif
