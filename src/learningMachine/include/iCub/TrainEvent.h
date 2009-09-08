/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard train event class.
 *
 */
 
#ifndef __ICUB_TRAINEVENT__
#define __ICUB_TRAINEVENT__

#include "iCub/Event.h"

#include <string>

/**
  * class TrainEvent
  *
  */

class TrainEvent : virtual public Event {
public:
    /**
     * Empty Constructor
     */
    TrainEvent();

    /**
     * Empty Destructor
     */
    virtual ~TrainEvent();

    /*
     * Inherited from Event.
     */
    void visit(EventListener listener) {
    }

    /*
     * Inherited from Event.
     */
    std::string toString() {
    }

};

#endif
