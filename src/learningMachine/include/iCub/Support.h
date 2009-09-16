/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * Support class for shared objects, such as factories.
 *
 */

#ifndef __ICUB_SUPPORT__
#define __ICUB_SUPPORT__

#include <string>

//#include "iCub/EventDispatcher.h"

namespace iCub {
namespace contrib {
namespace learningmachine {


/**
 * The Support class provides a unified access point to the Event Dispatcher 
 * and forms a base for the specialized MachineSupport and TransformerSupport
 * classes. 
 *
 * \see iCub::contrib::learningmachine::MachineSupport
 * \see iCub::contrib::learningmachine::TransformerSupport
 * \see iCub::contrib::learningmachine::EventDispatcher
 *
 * \author Arjan Gijsberts
 */

class Support {
protected:
    // event dispatcher

public:
    /**
     * Constructor.
     *
     * @param init indicates whether the factories should be filled with the
     * default list of objects
     */
    Support() {
    }

    /**
     * Destructor.
     */
    ~Support() {}
    

};

} // learningmachine
} // contrib
} // iCub

#endif
