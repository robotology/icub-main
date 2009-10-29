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
namespace learningmachine {

class IEventListener;

/**
 * A TrainEvent is raised when the machine handles a training sample. It 
 * contains the input, and predicted and actual output vectors.
 *
 * \see iCub::learningmachine::IEvent
 *
 * \author Arjan Gijsberts
 */

class TrainEvent : public IEvent {
protected:
    /**
     * Vector of inputs.
     */
    Vector input;
    
    /**
     * Vector of desired outputs.
     */
    Vector desired;
    
    /**
     * Vector of predicted outputs.
     */
    Vector predicted;

public:
    /**
     * Constructor
     *
     * @param input the vector of inputs
     * @param desired the vector of desired outputs
     * @param predicted the vector of predicted outputs
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
    
    /**
     * Accessor for the vector of inputs.
     * @return a reference to the registered vector of inputs
     */
    Vector& getInput() {
        return this->input;
    }

    /**
     * Accessor for the vector of inputs.
     * @return a reference to the registered vector of inputs
     */
    Vector& getDesired() {
        return this->desired;
    }

    /**
     * Accessor for the vector of inputs.
     * @return a reference to the registered vector of inputs
     */
    Vector& getPredicted() {
        return this->predicted;
    }

};

} // learningmachine
} // iCub

#endif
