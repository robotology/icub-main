/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard predict event class.
 *
 */

#ifndef __ICUB_PREDICTEVENT__
#define __ICUB_PREDICTEVENT__

#include <string>

#include "iCub/Event.h"

/**
 * A PredictEvent is raised when the machine makes a prediction. It contains 
 * the input and predicted output vectors.
 *
 * \see iCub::contrib::learningmachine::Event
 * 
 * \author Arjan Gijsberts
 */

class PredictEvent : virtual public Event {
protected:

public:
    /**
     * Constructor
     */
    PredictEvent();

    /**
     * Destructor
     */
    virtual ~PredictEvent();

    /*
     * Inherited from Event.
     */
    virtual void visit(EventListener& listener) {
        listener.handle(*this);
    }

    /*
     * Inherited from Event.
     */
    virtual std::string toString() {
        return std::string("TrainEvent.toString()");
    }


};

#endif
