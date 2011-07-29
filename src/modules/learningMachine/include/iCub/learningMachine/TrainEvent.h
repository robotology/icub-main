/*
 * Copyright (C) 2007-2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef LM_TRAINEVENT__
#define LM_TRAINEVENT__

#include <string>

#include <yarp/sig/Vector.h>

#include "iCub/learningMachine/Prediction.h"
#include "iCub/learningMachine/IEvent.h"
#include "iCub/learningMachine/IEventListener.h"

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
    yarp::sig::Vector input;

    /**
     * Vector of desired outputs.
     */
    yarp::sig::Vector desired;

    /**
     * Predicted outputs.
     */
    Prediction predicted;

public:
    /**
     * Constructor.
     *
     * @param input the vector of inputs
     * @param desired the vector of desired outputs
     * @param predicted the vector of predicted outputs
     */
    TrainEvent(const yarp::sig::Vector& input, const yarp::sig::Vector& desired, const Prediction& predicted);

    /**
     * Destructor (empty).
     */
    virtual ~TrainEvent() { }

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
    yarp::sig::Vector& getInput() {
        return this->input;
    }

    /**
     * Accessor for the vector of desired outputs.
     * @return a reference to the registered vector of inputs
     */
    yarp::sig::Vector& getDesired() {
        return this->desired;
    }

    /**
     * Accessor for the prediction.
     * @return a reference to the prediction
     */
    Prediction& getPredicted() {
        return this->predicted;
    }

};

} // learningmachine
} // iCub

#endif
