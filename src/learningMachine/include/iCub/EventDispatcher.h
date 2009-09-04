
#ifndef EVENTDISPATCHER_H
#define EVENTDISPATCHER_H

#include <string>
#include vector



/**
  * class EventDispatcher
  * 
  */

class EventDispatcher
{
public:

    // Constructors/Destructors
    //  


    /**
     * Empty Constructor
     */
    EventDispatcher ( );

    /**
     * Empty Destructor
     */
    virtual ~EventDispatcher ( );

    // Static Public attributes
    //  

    // Public attributes
    //  


    // Public attribute accessor methods
    //  


    // Public attribute accessor methods
    //  



    /**
     * Adds an EventListener to the list.
     * @param  listener The EventListener that is to be add.
     */
    void addListener (EventListener listener )
    {
    }


    /**
     * Removes an EventListener from the list.
     * @param  idx The index for the EventListener that is to be removed.
     */
    void removeListener (int idx )
    {
    }


    /**
     * Returns the EventListener at a specified index.
     * @return EventListener
     * @param  idx The index of the EventListener.
     */
    EventListener getListener (int idx )
    {
    }


    /**
     * Raises an Event, causing this Event to be dispatched to each registered
     * EventListener.
     * @param  event The Event instance that is to be raised.
     */
    void raise (Event event )
    {
    }


    /**
     * @return bool
     */
    bool hasListener ( )
    {
    }

protected:

    // Static Protected attributes
    //  

    // Protected attributes
    //  

public:


    // Protected attribute accessor methods
    //  

protected:

public:


    // Protected attribute accessor methods
    //  

protected:


private:

    // Static Private attributes
    //  

    // Private attributes
    //  

    // A container for the list of EventListeners.
    vector<EventListener> listeners;
public:


    // Private attribute accessor methods
    //  

private:

public:


    // Private attribute accessor methods
    //  


    /**
     * Set the value of listeners
     * A container for the list of EventListeners.
     * @param new_var the new value of listeners
     */
    void setListeners ( vector<EventListener> new_var )     {
            listeners = new_var;
    }

    /**
     * Get the value of listeners
     * A container for the list of EventListeners.
     * @return the value of listeners
     */
    vector<EventListener> getListeners ( )     {
        return listeners;
    }
private:


    void initAttributes ( ) ;

};

#endif // EVENTDISPATCHER_H
