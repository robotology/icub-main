// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _HEADING_FOVEA__H_
#define _HEADING_FOVEA__H_

#include "Heading.h"
#include <vector>

namespace CB {
    
    /**
     * Implements the Heading abstract interface to return the 
     * fovea positon (i.e., [0 0])
     **/
    class HeadingFovea : public Heading {
       
        
    public:
        
        /**
         * Constructor
         */
        HeadingFovea(std::string name="") {
        
            deviceName=name+"/fovea";
            updateDelay=1.0;

            // mandatory inherit function
            initPorts();
            
            // set fovea position ([0 0])
            values.zero();
            
        }
        
        /** 
         * Destructor
         **/
        ~HeadingFovea() {  }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        bool updateResource() { return true; }

        /**
         * Inherited start function.
         **/
        void startResource() { }

        /**
         * Inherited stop function.
         **/
        void stopResource() { }

        
    };
    
}

#endif
