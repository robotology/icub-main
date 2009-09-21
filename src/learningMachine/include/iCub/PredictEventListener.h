/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard event listener class for predict events.
 *
 */

#ifndef __ICUB_PREDICTEVENTLISTENER__
#define __ICUB_PREDICTEVENTLISTENER__

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

public:
    /**
     * Constructor
     */
    PredictEventListener(std::string name = "predict");

    /**
     * Destructor
     */
    virtual ~PredictEventListener();


    /*
     * Inherited from EventListener.
     */
    void handle(PredictEvent& e) {
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
