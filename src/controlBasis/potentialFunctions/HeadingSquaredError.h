// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _HEADING_SQUARED_ERROR__H_
#define _HEADING_SQUARED_ERROR__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a Squared Error quadratic potential function
     * for a Heading resources, s.t., \phi = (1/2)*gamma^T*gamma.  
     * Implemented as gamma_reference - gamma_current.
     **/
    class HeadingSquaredError : public ControlBasisPotentialFunction {
        
    public:
        
        /**
         * Empty Constructor, needs to set configuration info
         **/
        HeadingSquaredError() :
            ControlBasisPotentialFunction("squared_error_pf", "heading", true)        
        {                        
            size = 2;
            gradient.resize(size);           
        }

        /**
         * Destructor
         **/        
        ~HeadingSquaredError() { }

        /**
         * inherited update function
         **/
        virtual bool updatePotentialFunction();

        /**
         * inherited start function
         **/
        virtual void startPotentialFunction();
        
        /**
         * inherited stop function
         **/
        virtual void stopPotentialFunction();
        
        /**
         * inherited connect function
         **/
        virtual bool connectToInputs();


    };
    
}

#endif
