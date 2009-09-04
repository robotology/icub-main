
#ifndef PREDICTEVENTLISTENER_H
#define PREDICTEVENTLISTENER_H
#include "EventListener.h"

#include <string>

/**
  * class PredictEventListener
  * 
  */

class PredictEventListener : virtual public EventListener
{
public:

    // Constructors/Destructors
    //  


    /**
     * Empty Constructor
     */
    PredictEventListener ( );

    /**
     * Empty Destructor
     */
    virtual ~PredictEventListener ( );

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
    void handle (PredictEvent e )
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

#endif // PREDICTEVENTLISTENER_H
