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


#ifndef LM_DISPATCHERMANAGER__
#define LM_DISPATCHERMANAGER__

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
