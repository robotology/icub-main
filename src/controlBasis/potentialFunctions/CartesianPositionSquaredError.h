// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CARTESIAN_POSITION_SQUARED_ERROR__H_
#define _CARTESIAN_POSITION_SQUARED_ERROR__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a Squared Error quadratic potential function
     * for Cartesian Position resources, s.t., \phi = (1/2)*x^T*X.  
     * Implemented as x_current - x_reference.
     **/
    class CartesianPositionSquaredError : public ControlBasisPotentialFunction {
        
    public:
        
        /**
         * Empty Constructor, needs to set configuration info
         **/
        CartesianPositionSquaredError() :
            ControlBasisPotentialFunction("squared_error_pf", "cartesianposition", true)        
        {                        
            size = 3;
            gradient.resize(size);           
        }

        /**
         * Destructor
         **/        
        ~CartesianPositionSquaredError() { }

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
