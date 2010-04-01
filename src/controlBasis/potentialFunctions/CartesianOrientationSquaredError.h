// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CARTESIAN_ORIENTATION_SQUARED_ERROR__H_
#define _CARTESIAN_ORIENTATION_SQUARED_ERROR__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a Squared Error quadratic potential function
     * for Cartesian Orientation resources, s.t., \phi = (1/2)*x^T*X.  
     * Implemented as x_reference - x_current.
     **/
    class CartesianOrientationSquaredError : public ControlBasisPotentialFunction {
        
    public:
        
        /**
         * Empty Constructor, needs to set configuration info
         **/
        CartesianOrientationSquaredError() :
            ControlBasisPotentialFunction("squared_error_pf", "cartesianorientation", true)        
        {                        
            size = 3;
            gradient.resize(size);           
        }

        /**
         * Destructor
         **/        
        ~CartesianOrientationSquaredError() { }

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


        inline double min(double a, double b) {
            return (a < b) ? a : b;
        }

    };
    
}

#endif
