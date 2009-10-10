/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard event listener class for predict events.
 *
 */

#ifndef __ICUB_PREDICTEVENTLISTENER__
#define __ICUB_PREDICTEVENTLISTENER__

#include <string>

#include <yarp/sig/Vector.h>

#include "iCub/IPortEventListener.h"
#include "iCub/PredictEvent.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 * 
 * \see iCub::contrib::learningmachine::IEventListener
 * 
 * \author Arjan Gijsberts
 */

class PredictEventListener : virtual public IPortEventListener {
protected:
    void vectorToBottle(const Vector& vec, Bottle& bot) {
        for(int i = 0; i < vec.size(); i++) 
            bot.addDouble(vec[i]);
    }

public:
    /**
     * Constructor
     */
    PredictEventListener(std::string name = "Predict", std::string pp = "/lm/event/predict") : IPortEventListener(name, pp) { }

    /*
     * Inherited from IEventListener.
     */
    void handle(PredictEvent& e);

    /*
     * Inherited from IEventListener.
     */
    IEventListener* create() {
        return new PredictEventListener(this->getName());
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
