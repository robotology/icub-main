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

#include <yarp/os/Port.h>

#include "iCub/IEventListener.h"
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

class PredictEventListener : virtual public IEventListener {
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
    PredictEventListener(std::string name = "Predict");

    /**
     * Destructor
     */
    virtual ~PredictEventListener();

    /*
     * Inherited from IEventListener.
     */
    void handle(PredictEvent& e);

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
        return new PredictEventListener(this->getName());
    }

};

} // learningmachine
} // contrib
} // iCub

#endif
