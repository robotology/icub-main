/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard train event class.
 *
 */
 
#ifndef __ICUB_TRAINEVENT__
#define __ICUB_TRAINEVENT__

#include <string>

#include "iCub/IEvent.h"

/**
 * A TrainEvent is raised when the machine handles a training sample. It 
 * contains the input, and predicted and actual output vectors.
 *
 * \see iCub::contrib::learningmachine::IEvent
 *
 * \author Arjan Gijsberts
 */

class TrainEvent : virtual public IEvent {
public:
    /**
     * Constructor
     */
    TrainEvent();

    /**
     * Destructor
     */
    virtual ~TrainEvent();

    /*
     * Inherited from Event.
     */
    virtual void visit(IEventListener& listener) {
        listener.handle(*this);
    }

    /*
     * Inherited from Event.
     */
    std::string toString() {
        return std::string("TrainEvent.toString()");
    }

};

#endif
