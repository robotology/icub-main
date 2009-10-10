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

using namespace yarp::sig;
using namespace yarp::os;

namespace iCub {
namespace contrib {
namespace learningmachine {

/**
 * 
 * \see iCub::contrib::learningmachine::IEventListener
 * 
 * \author Arjan Gijsberts
 */

class TrainEventListener : virtual public IPortEventListener {
protected:
    void vectorToBottle(const Vector& vec, Bottle& bot) {
        for(int i = 0; i < vec.size(); i++) 
            bot.addDouble(vec[i]);
    }

public:
    /**
     * Constructor
     */
    TrainEventListener(std::string name = "Train", std::string pp = "/lm/event/train") 
      : IPortEventListener(name, pp) {}

    /*
     * Inherited from IEventListener.
     */
    void handle(TrainEvent& e);

    /*
     * Inherited from IEventListener.
     */
    IEventListener* create() {
        return new TrainEventListener(this->getName());
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
