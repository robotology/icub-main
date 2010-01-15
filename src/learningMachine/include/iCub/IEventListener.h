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

#ifndef LM_IEVENTLISTENER__
#define LM_IEVENTLISTENER__

#include <yarp/os/IConfig.h>

#include "iCub/IEvent.h"
#include "iCub/TrainEvent.h"
#include "iCub/PredictEvent.h"

using namespace yarp::os;

namespace iCub {
namespace learningmachine {

/**
 * The abstract base class for EventListener handlers. Due to technical
 * limitations of C++, this base class needs to implement an empty, virtual
 * handle method for each Event type.
 *
 * \author Arjan Gijsberts
 */

// forward prototype declarations
class IEvent;
class TrainEvent;
class PredictEvent;

class IEventListener : public IConfig {
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
     */
    IEventListener();

    /**
     * Destructor (empty).
     */
    virtual ~IEventListener() { }

    /**
     * Default handler for any Event, which means the Event is ignored.
     * @param  e the base event
     */
    virtual void handle(IEvent& e) { }

    /**
     * Handling of a TrainEvent.
     * @param  e the TrainEvent
     */
    virtual void handle(TrainEvent& e) { }

    /**
     * Handling of a PredictEvent.
     * @param  e the PredictEvent
     */
    virtual void handle(PredictEvent& e) { }

    /**
     * Asks the event listener to return a new object of its type.
     *
     * @return a fresh instance of the current class
     */
    virtual IEventListener* clone() = 0;

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
     * Starts the IEventListener, such that it can do perform initialization
     * (e.g. opening ports).
     */
    virtual void start() { }

    /**
     * Tells whether dispatching of events is enabled.
     *
     * @return  true if dispatching is enabled
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
     * @return the information on the IEventListener
     */
    virtual std::string getInfo() {
        return this->getName() + (this->isEnabled() ? " (enabled)" : " (disabled)");
    }

};

} // learningmachine
} // iCub

#endif
