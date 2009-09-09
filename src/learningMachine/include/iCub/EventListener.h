/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The abstract event listening class.
 *
 */

#ifndef __ICUB_EVENTLISTENER__
#define __ICUB_EVENTLISTENER__

#include "iCub/Event.h"
#include "iCub/TrainEvent.h"
#include "iCub/PredictEvent.h"

/**
 * The abstract base class for EventListener handlers. Due to technical 
 * limitations of C++, this base class needs to implement an empty, virtual 
 * handle method for each Event type.
 *
 * \author Arjan Gijsberts
 */

class EventListener {
protected:

public:
    /**
     * Default handler for any Event, which means the Event is ignored.
     * @param  e The Event.
     */
    virtual void handle(Event& e) {
    }

    /**
     * Handling of a TrainEvent.
     * @param  e The TrainEvent.
     */
    virtual void handle(TrainEvent& e) {
    }

    /**
     * Handling of a PredictEvent.
     * @param  e The PredictEvent.
     */
    virtual void handle(PredictEvent& e) {
    }


};

#endif
