
#ifndef EVENTLISTENER_H
#define EVENTLISTENER_H

#include <string>
#include vector



/**
  * class EventListener
  * 
  */

/******************************* Abstract Class ****************************
EventListener does not have any pure virtual methods, but its author
  defined it as an abstract class, so you should not use it directly.
  Inherit from it instead and create only objects from the derived classes
*****************************************************************************/

class EventListener
{
public:


    // Public attribute accessor methods
    //  


    // Public attribute accessor methods
    //  



    /**
     * Default handler for an Event, which means the Event is ignored.
     * @param  e The Event.
     */
    virtual void handle (Event e )
    {
    }


    /**
     * Handling of a TrainEvent.
     * @param  e The TrainEvent.
     */
    virtual void handle (TrainEvent e )
    {
    }


    /**
     * Handling of a PredictEvent.
     * @param  e The PredictEvent.
     */
    virtual void handle (PredictEvent e )
    {
    }

protected:

public:


    // Protected attribute accessor methods
    //  

protected:

public:


    // Protected attribute accessor methods
    //  

protected:


private:

public:


    // Private attribute accessor methods
    //  

private:

public:


    // Private attribute accessor methods
    //  

private:



};

#endif // EVENTLISTENER_H
