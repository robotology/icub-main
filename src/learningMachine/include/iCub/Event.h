
#ifndef EVENT_H
#define EVENT_H

#include <string>

/**
  * class Event
  * 
  */

class Event
{
public:


    // Public attribute accessor methods
    //  


    // Public attribute accessor methods
    //  



    /**
     * Returns a string representation of the Event.
     * @return string
     */
    virtual string toString ( )
    {
    }


    /**
     * Causes the Event to visit an EventListener. This double dispatch strategy allows
     * for determination of runtime types of both the Event and the EventListener. For
     * this to work, child classes need to implement this function.
     * @param  listener
     */
    virtual void visit (EventListener listener )
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

#endif // EVENT_H
