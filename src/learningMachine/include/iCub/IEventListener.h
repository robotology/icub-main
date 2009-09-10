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

/**
 * The abstract base class for EventListener handlers. Due to technical 
 * limitations of C++, this base class needs to implement an empty, virtual 
 * handle method for each Event type.
 *
 * \author Arjan Gijsberts
 */

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
    IEventListener(std::string name) {
        this->setName(name);
        this->sampleCount = 0;
    }

    /**
     * Destructor.
     */
    ~IEventListener() {}

    /**
     * Default handler for any Event, which means the Event is ignored.
     * @param  e The Event.
     */
    virtual void handle(Event& e) {
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
     * Retrieve the name of this EventListener.
     *
     * @return the name of this EventListener
     */
    std::string getName() const {
        return this->name;
    }

    /**
     * Set the name of this EventListener.
     *
     * @param name the new name
     */
    void setName(std::string name) {
        this->name = name;
    }


};

#endif
