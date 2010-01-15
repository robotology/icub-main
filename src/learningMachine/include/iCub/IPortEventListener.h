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

#ifndef LM_IPORTEVENTLISTENER__
#define LM_IPORTEVENTLISTENER__

#include <string>

#include <yarp/os/Port.h>

#include "iCub/IEventListener.h"


using namespace yarp::os;

namespace iCub {
namespace learningmachine {

/**
 * The abstract base class for EventListeners that output to a port. This class
 * is introduced so that base functionality to register ports can be inherited
 * by multiple subclasses.
 *
 * \see iCub::learningmachine::PredictEventListener
 * \see iCub::learningmachine::TrainEventListener
 *
 * \author Arjan Gijsberts
 */

class IPortEventListener : public IEventListener {
protected:
    /**
     * The outgoing port for the events.
     */
    Port port;

    /**
     * A prefix path for the ports that will be registered.
     */
    std::string portPrefix;

    /**
     * Resets the port and opens it at the specified name. If passed an empty
     * string the port is opened at the first free port with the predefined
     * prefix.
     *
     * @param portName the name of the port
     */
    void resetPort(std::string portName);

public:
    /**
     * Constructor.
     *
     * @param pp the standard prefix for opening the ports
     */
    IPortEventListener(std::string pp) : portPrefix(pp) { }

    /**
     * Copy Constructor.
     */
    IPortEventListener(const IPortEventListener& other)
      : IEventListener(other), portPrefix(other.portPrefix) { }

    /**
     * Destructor.
     */
    virtual ~IPortEventListener() {
        this->port.interrupt();
        this->port.close();
    }

    /**
     * Assignment operator.
     */
    virtual IPortEventListener& operator=(const IPortEventListener& other);

    /**
     * Starts the IEventListener.
     */
    virtual void start() {
        this->resetPort("");
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
        return this->IEventListener::getInfo() + " [port: " + this->port.where().toString().c_str() + "]";
    }

};

} // learningmachine
} // iCub

#endif
