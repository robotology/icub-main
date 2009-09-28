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

    /**
     * Boolean switch to disable or enable event the listener.
     */
    bool enabled;

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

    /**
     * Tells whether dispatching of events is enabled.
     *
     * @return  true if dispatching is enabled.
     */
    virtual bool isEnabled() {
        return this->enabled;
    }

    /**
     * Enables or disables dispatching of events.
     *
     * @param val the desired state
     */
    virtual void setEnabled(bool val) {
        this->enabled = val;
    }

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);

    /**
     * Asks the event listener to return a string containing information on 
     * its configuration so far.
     *
     * @return the statistics of the machine
     */
    virtual std::string getInfo() { 
        return std::string("Type: ") + this->getName() + std::string("\n"); 
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
