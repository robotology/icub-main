/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard event listener class for predict events.
 *
 */

#ifndef __ICUB_TRAINEVENTLISTENER__
#define __ICUB_TRAINEVENTLISTENER__

#include <string>

#include <yarp/os/Port.h>
#include <yarp/sig/Vector.h>

#include "iCub/IEventListener.h"
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

class TrainEventListener : virtual public IEventListener {
protected:
    /**
     * The outgoing port for train events.
     */
    Port port;
    
    /**
     * Resets the port and opens it at the specified name.
     *
     * @param portName the name of the port
     */
    void resetPort(std::string portName);
    
    void vectorToBottle(const Vector& vec, Bottle& bot) {
        for(int i = 0; i < vec.size(); i++) 
            bot.addDouble(vec[i]);
    }

public:

    /**
     * Constructor
     */
    TrainEventListener(std::string name = "Train");

    /**
     * Destructor
     */
    virtual ~TrainEventListener();

    /*
     * Inherited from IEventListener.
     */
    void handle(TrainEvent& e);

    /*
     * Inherited from IConfig.
     */
    virtual bool configure(Searchable& config);

    /*
     * Inherited from IEventListener.
     */
    virtual void start() {
        this->resetPort("");
    }

    /*
     * Inherited from IEventListener.
     */
    virtual std::string getInfo() { 
        return this->IEventListener::getInfo() + " [port: " + this->port.where().toString().c_str() + "]"; 
    }

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
