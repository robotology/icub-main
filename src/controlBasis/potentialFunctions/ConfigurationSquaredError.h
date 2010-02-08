// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONFIGURATION_SQUARED_ERROR__H_
#define _CONFIGURATION_SQUARED_ERROR__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a Squared Error quadratic potential function, 
     * for Configuration rsources s.t., \phi = (1/2)*q^T*q.  
     * Implemented as q_current - q_reference.
     **/    
    class ConfigurationSquaredError : public ControlBasisPotentialFunction {
        
    public:
        
        /** 
         * Constructor.
         * \param inName the current value of the device
         * \param refName the reference value of the device
         **/
        ConfigurationSquaredError(std::string inName, std::string refName, int n=0) {
            
            size = n;

            inputName[0] = inName;
            inputName[1] = refName;

            std::cout << "Creating new ConfigurationSquaredError PotentialFunction (cur=" 
                      << inputName[0].c_str() << ", ref=" << inputName[1].c_str() << ", size=" << size << std::endl;

            if(n!=0) {
                input[0].resize(size);
                input[1].resize(size);
            }

            // temp allocation, will resize later when connect to inputs
            gradient.resize(1);
            potential = 0;

            hasReference = true;
            running = false;
            connectedToInputs = false;

            inputSpace = "configuration";          
            pfTypeName = "squared_error_pf";

        }
        
        /**
         * Destructor
         **/        
        ~ConfigurationSquaredError() { }

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
