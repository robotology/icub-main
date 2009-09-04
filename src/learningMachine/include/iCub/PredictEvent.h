
#ifndef PREDICTEVENT_H
#define PREDICTEVENT_H
#include "Event.h"

#include <string>

/**
  * class PredictEvent
  * 
  */

class PredictEvent : virtual public Event
{
public:

    // Constructors/Destructors
    //  


    /**
     * Empty Constructor
     */
    PredictEvent ( );

    /**
     * Empty Destructor
     */
    virtual ~PredictEvent ( );

    // Static Public attributes
    //  

    // Public attributes
    //  


    // Public attribute accessor methods
    //  


    // Public attribute accessor methods
    //  



    /**
     * @param  listener
     */
    void visit (EventListener listener )
    {
    }


    /**
     * @return string
     */
    string toString ( )
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

public:


    // Private attribute accessor methods
    //  

private:

public:


    // Private attribute accessor methods
    //  

private:



};

#endif // PREDICTEVENT_H
