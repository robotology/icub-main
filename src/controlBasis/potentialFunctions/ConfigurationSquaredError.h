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
         * Empty Constructor, needs to set configuration info
         **/
        ConfigurationSquaredError() :
            ControlBasisPotentialFunction("squared_error_pf", "configuration", true)        
        {            
            // temp allocation, will resize later when connect to inputs
            gradient.resize(1);
            std::cout << "Created new ConfigurationSquaredError..." << std::endl;        
        }

        /**
         * Destructor
         **/        
        ~ConfigurationSquaredError() { }

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
