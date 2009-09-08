/*
 * Copyright (C) 2007-2009 Arjan Gijsberts @ Italian Institute of Technology
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * The abstract event listening class.
 *
 */

#ifndef __ICUB_EVENTLISTENER__
#define __ICUB_EVENTLISTENER__


/**
  * class EventListener
  *
  */

class EventListener {
protected:

public:
    /**
     * Default handler for an Event, which means the Event is ignored.
     * @param  e The Event.
     */
    virtual void handle(Event e) {
    }


    /**
     * Handling of a TrainEvent.
     * @param  e The TrainEvent.
     */
    virtual void handle(TrainEvent e) {
    }


    /**
     * Handling of a PredictEvent.
     * @param  e The PredictEvent.
     */
    virtual void handle(PredictEvent e) {
    }


};

#endif
