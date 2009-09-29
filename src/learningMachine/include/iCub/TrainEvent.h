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

#include <yarp/sig/Vector.h>

#include "iCub/IEvent.h"
#include "iCub/IEventListener.h"

using namespace yarp::sig;

namespace iCub {
namespace contrib {
namespace learningmachine {

class IEventListener;

/**
 * A TrainEvent is raised when the machine handles a training sample. It 
 * contains the input, and predicted and actual output vectors.
 *
 * \see iCub::contrib::learningmachine::IEvent
 *
 * \author Arjan Gijsberts
 */

class TrainEvent : virtual public IEvent {
protected:
    Vector input;
    Vector desired;
    Vector predicted;
public:
    /**
     * Constructor
     */
    TrainEvent(Vector input, Vector desired, Vector predicted);

    /**
     * Destructor
     */
    virtual ~TrainEvent();

    /*
     * Inherited from IEvent.
     */
    virtual void visit(IEventListener& listener);

    /*
     * Inherited from IEvent.
     */
    std::string toString();
    
    Vector& getInput() {
        return this->input;
    }

    Vector& getDesired() {
        return this->desired;
    }

    Vector& getPredicted() {
        return this->predicted;
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
