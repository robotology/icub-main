/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * A YARP-aware managing interface for the EventDispatcher
 *
 */

#ifndef __ICUB_DISPATCHERMANAGER__
#define __ICUB_DISPATCHERMANAGER__

#include <yarp/os/Bottle.h>

#include "iCub/EventDispatcher.h"
#include "iCub/EventListenerFactory.h"


namespace iCub {
namespace learningmachine {


/**
 * The DispatcherManager provides a YARP-based configuration interface for the 
 * EventDispatcher. 
 *
 * \see iCub::learningmachine::EventDispatcher
 *
 * \author Arjan Gijsberts
 *
 */

class DispatcherManager {
private:
    /**
     * Cached pointer to (singleton) instance of EventDispatcher.
     */
    EventDispatcher* dispatcher;
    
    /**
     * Cached pointer to (singleton) instance of EventListenerFactory.
     */
    EventListenerFactory* factory;

public:
    /**
     * Empty Constructor
     */
    DispatcherManager();

    /**
     * Respond to a command or configuration message.
     * @param command the message received
     * @param reply the response you wish to make
     * @return true if there was no critical failure
     */
    bool respond(const Bottle& cmd, Bottle& reply);

};

} // learningmachine
} // iCub

#endif
