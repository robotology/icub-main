/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard predict event class.
 *
 */

#ifndef __ICUB_PREDICTEVENT__
#define __ICUB_PREDICTEVENT__

#include "iCub/Event.h"

#include <string>

/**
  * class PredictEvent
  *
  */

class PredictEvent : virtual public Event {
protected:

public:
    /**
     * Empty Constructor
     */
    PredictEvent();

    /**
     * Empty Destructor
     */
    virtual ~PredictEvent();

    /*
     * Inherited from Event.
     */
    void visit(EventListener listener) {
    }


    /*
     * Inherited from Event.
     */
    virtual std::string toString() {
    }


};

#endif
