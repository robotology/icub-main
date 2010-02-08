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
         * Constructor.
         * \param inName the current value of the device
         * \param refName the reference value of the device
         **/
        CartesianPositionSquaredError(std::string inName, std::string refName) {
            
            size = 3;
            inputName[0] = inName;
            inputName[1] = refName;

            std::cout << "Creating new CartesianPositionSquaredError PotentialFunction (cur=" 
                      << inputName[0].c_str() << ", ref=" << inputName[1].c_str() << "), size=" << size << std::endl;
            
            input[0].resize(size);
            input[1].resize(size);

            potential = 0;
            gradient.resize(size);

            inputSpace = "cartesianposition";
            pfTypeName = "squared_error_pf";

            hasReference = true;           
            connectedToInputs = false;
            running = false;
           
        }

        /**
         * Destructor
         **/        
        ~CartesianPositionSquaredError() { }

        /**
         * inherited update function
         **/
        bool updatePotentialFunction();

        /**
         * inherited start function
         **/
        void startPotentialFunction();
        
        /**
         * inherited stop function
         **/
        void stopPotentialFunction();
        
        /**
         * inherited connect function
         **/
        bool connectToInputs();


    };
    
}

#endif
