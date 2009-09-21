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

#include "iCub/IEvent.h"
#include "iCub/IEventListener.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 * A PredictEvent is raised when the machine makes a prediction. It contains 
 * the input and predicted output vectors.
 *
 * \see iCub::contrib::learningmachine::IEvent
 * 
 * \author Arjan Gijsberts
 */

class IEventListener;

class PredictEvent : virtual public IEvent {
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
     * Inherited from IEvent.
     */
    virtual void visit(IEventListener& listener);

    /*
     * Inherited from IEvent.
     */
    virtual std::string toString() {
        return std::string("TrainEvent.toString()");
    }


};

} // learningmachine
} // contrib
} // iCub

#endif
