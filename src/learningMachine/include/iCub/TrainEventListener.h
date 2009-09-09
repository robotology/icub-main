/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The standard event listener class for predict events.
 *
 */

#ifndef __ICUB_TRAINEVENTLISTENER__
#define __ICUB_TRAINEVENTLISTENER__

#include "iCub/EventListener.h"
#include "iCub/TrainEvent.h"

/**
 * class TrainEventListener
 *
 */

class TrainEventListener : virtual public EventListener {
protected:

public:

    /**
     * Constructor
     */
    TrainEventListener();

    /**
     * Destructor
     */
    virtual ~TrainEventListener();

    /*
     * Inherited from EventListener.
     */
    void handle(TrainEvent& e) {
    }

};

#endif
