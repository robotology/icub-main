/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard event listener class for train events.
 *
 */

#ifndef __ICUB_TRAINEVENTLISTENER__
#define __ICUB_TRAINEVENTLISTENER__

#include <string>

#include <yarp/sig/Vector.h>

#include "iCub/IPortEventListener.h"
#include "iCub/TrainEvent.h"

namespace iCub {
namespace learningmachine {

/**
 * 
 * \see iCub::learningmachine::IEventListener
 * 
 * \author Arjan Gijsberts
 */

class TrainEventListener : public IPortEventListener {
protected:
    void vectorToBottle(const Vector& vec, Bottle& bot) {
        for(int i = 0; i < vec.size(); i++) 
            bot.addDouble(vec[i]);
    }

public:
    /**
     * Constructor
     *
     * @param pp default port prefix
     */
    TrainEventListener(std::string pp = "/lm/event/train") : IPortEventListener(pp) {
        this->setName("Train");
      }

    /*
     * Inherited from IEventListener.
     */
    void handle(TrainEvent& e);

    /*
     * Inherited from IEventListener.
     */
    IEventListener* clone() {
        return new TrainEventListener(*this);
    }

};

} // learningmachine
} // iCub

#endif
