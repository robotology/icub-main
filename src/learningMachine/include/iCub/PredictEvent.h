/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard predict event class.
 *
 */

#ifndef LM_PREDICTEVENT__
#define LM_PREDICTEVENT__

#include <string>

#include <yarp/sig/Vector.h>

#include "iCub/IEvent.h"
#include "iCub/IEventListener.h"

using namespace yarp::sig;

namespace iCub {
namespace learningmachine {


/**
 * A PredictEvent is raised when the machine makes a prediction. It contains
 * the input and predicted output vectors.
 *
 * \see iCub::learningmachine::IEvent
 *
 * \author Arjan Gijsberts
 */

class IEventListener;

class PredictEvent : public IEvent {
protected:
    /**
     * Vector of inputs.
     */
    Vector input;

    /**
     * Vector of predicted outputs.
     */
    Vector predicted;

public:
    /**
     * Constructor.
     *
     * @param input the vector of inputs
     * @param predicted the vector of predicted outputs
     */
    PredictEvent(const Vector& input, const Vector& predicted);

    /**
     * Destructor (empty).
     */
    virtual ~PredictEvent() { }

    /*
     * Inherited from IEvent.
     */
    virtual void visit(IEventListener& listener);

    /*
     * Inherited from IEvent.
     */
    virtual std::string toString();

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
    Vector& getPredicted() {
        return this->predicted;
    }

};

} // learningmachine
} // iCub

#endif
