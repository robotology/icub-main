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

#ifndef LM_EVENT__
#define LM_EVENT__

#include <string>

namespace iCub {
namespace learningmachine {

/**
 * The abstract base class for all Event types.
 *
 * \author Arjan Gijsberts
 */

// forward declaration to solve circular reference
class IEventListener;

class IEvent {
protected:


public:
    /**
     * Returns a string representation of the Event.
     * @return string  the string representation
     */
    virtual std::string toString() = 0;

    /**
     * Causes the Event to visit an EventListener. This method is part of the
     * double dispatch mechanism. Child classes _need_ to override this
     * function.
     * @param listener  the listener
     */
    virtual void visit(IEventListener& listener) = 0;

};

} // learningmachine
} // iCub

#endif
