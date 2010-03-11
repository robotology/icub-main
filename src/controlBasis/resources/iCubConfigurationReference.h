// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _ICUB_CONFIGURATION_REFERENCE__H_
#define _ICUB_CONFIGURATION_REFERENCE__H_

#include "ConfigurationVariables.h"

namespace CB {

    /**
     * this class impliments a configuration resource to provide references values to the iCub.
     **/    
    class iCubConfigurationReference : public ConfigurationVariables {
        
    public:
       
        /** 
         * constructor
         **/
        iCubConfigurationReference(std::string name, int dofs);        

        /** 
         * destructor
         **/
        ~iCubConfigurationReference() { }

        /** 
         * implements update for the parent class (does nothing here)
         **/
        bool updateResource() { return true; }
        
        /** 
         * starts the resource
         **/
        void startResource() { start(); }

        /** 
         * stops the resource
         **/
        void stopResource() { stop(); }
        
        /** 
         * sets the value of the referene from the input
         * \param ref the reference vector
         **/
        void setVals(yarp::sig::Vector ref); 

    };
    
}

#endif
