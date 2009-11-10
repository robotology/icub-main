/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard event listener class for predict events.
 *
 */

#ifndef LM_PREDICTEVENTLISTENER__
#define LM_PREDICTEVENTLISTENER__

#include <string>

#include <yarp/sig/Vector.h>

#include "iCub/IPortEventListener.h"
#include "iCub/PredictEvent.h"

namespace iCub {
namespace learningmachine {

/**
 *
 * \see iCub::learningmachine::IPortEventListener
 *
 * \author Arjan Gijsberts
 */

class PredictEventListener : public IPortEventListener {
protected:
    void vectorToBottle(const Vector& vec, Bottle& bot) {
        for(int i = 0; i < vec.size(); i++) {
            bot.addDouble(vec[i]);
        }
    }

public:
    /**
     * Constructor
     *
     * @param pp default port prefix
     */
    PredictEventListener(std::string pp = "/lm/event/predict") : IPortEventListener(pp) {
        this->setName("Predict");
    }

    /*
     * Inherited from IEventListener.
     */
    void handle(PredictEvent& e);

    /*
     * Inherited from IEventListener.
     */
    PredictEventListener* clone() {
        return new PredictEventListener(*this);
    }

};

} // learningmachine
} // iCub

#endif
