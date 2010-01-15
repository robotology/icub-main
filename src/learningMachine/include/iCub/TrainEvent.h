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
     * Constructor.
     *
     * @param input the vector of inputs
     * @param desired the vector of desired outputs
     * @param predicted the vector of predicted outputs
     */
    TrainEvent(const Vector& input, const Vector& desired, const Vector& predicted);

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
