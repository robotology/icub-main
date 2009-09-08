/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard event listener class for predict events.
 *
 */

#ifndef __ICUB_PREDICTEVENTLISTENER__
#define __ICUB_PREDICTEVENTLISTENER__

#include "iCub/EventListener.h"


/**
  * class PredictEventListener
  *
  */

class PredictEventListener : virtual public EventListener {
protected:

public:
    /**
     * Empty Constructor
     */
    PredictEventListener();

    /**
     * Empty Destructor
     */
    virtual ~PredictEventListener();


    /*
     * Inherited from EventListener.
     */
    void handle(PredictEvent e) {
    }


};

#endif
