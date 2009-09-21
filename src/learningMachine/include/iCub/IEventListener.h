/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The abstract event listening class.
 *
 */

#ifndef __ICUB_EVENTLISTENER__
#define __ICUB_EVENTLISTENER__

#include <yarp/os/IConfig.h>

#include "iCub/IEvent.h"
#include "iCub/TrainEvent.h"
#include "iCub/PredictEvent.h"

using namespace yarp::os;

namespace iCub {
namespace contrib {
namespace learningmachine {

/**
 * The abstract base class for EventListener handlers. Due to technical 
 * limitations of C++, this base class needs to implement an empty, virtual 
 * handle method for each Event type.
 *
 * \author Arjan Gijsberts
 */
class IEvent;
class TrainEvent;
class PredictEvent;

class IEventListener : virtual public IConfig {
protected:
    /**
     * The name of this type of EventListener.
     */
    std::string name;

public:
    /**
     * Constructor.
     *
     * @param name the name under which this EventListener will be registered
     */
    IEventListener(std::string name);

    /**
     * Destructor.
     */
    ~IEventListener() {}

    /**
     * Default handler for any Event, which means the Event is ignored.
     * @param  e The Event.
     */
    virtual void handle(IEvent& e) {
    }

    /**
     * Handling of a TrainEvent.
     * @param  e The TrainEvent.
     */
    virtual void handle(TrainEvent& e) {
    }

    /**
     * Handling of a PredictEvent.
     * @param  e The PredictEvent.
     */
    virtual void handle(PredictEvent& e) {
    }

    /**
     * Asks the event listener to return a new object of its type.
     *
     * @return a fresh instance of the current class
     */
    virtual IEventListener* create() = 0;

    /**
     * Retrieve the name of this IEventListener.
     *
     * @return the name of this IEventListener
     */
    std::string getName() const {
        return this->name;
    }

    /**
     * Set the name of this IEventListener.
     *
     * @param name the new name
     */
    void setName(std::string name) {
        this->name = name;
    }


};

} // learningmachine
} // contrib
} // iCub

#endif
