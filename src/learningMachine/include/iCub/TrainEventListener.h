
#ifndef TRAINEVENTLISTENER_H
#define TRAINEVENTLISTENER_H
#include "EventListener.h"

#include <string>

/**
  * class TrainEventListener
  * 
  */

class TrainEventListener : virtual public EventListener
{
public:

    // Constructors/Destructors
    //  


    /**
     * Empty Constructor
     */
    TrainEventListener ( );

    /**
     * Empty Destructor
     */
    virtual ~TrainEventListener ( );

    // Static Public attributes
    //  

    // Public attributes
    //  


    // Public attribute accessor methods
    //  


    // Public attribute accessor methods
    //  



    /**
     * @param  e
     */
    void handle (TrainEvent e )
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

#endif // TRAINEVENTLISTENER_H
