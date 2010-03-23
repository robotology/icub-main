// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _STEREO_HEADING_FOVEA__H_
#define _STEREO_HEADING_FOVEA__H_

#include "StereoHeading.h"
#include <vector>

namespace CB {
    
    /**
     * Implements the StereoHeading abstract interface to return the 
     * fovea positon (i.e., [0 0 0 0])
     **/
    class StereoHeadingFovea : public StereoHeading {
       
        
    public:
        
        /**
         * Constructor
         */
        StereoHeadingFovea(std::string name="") {
        
            deviceName=name+"/fovea";
            updateDelay=1.0;

            // mandatory inherit function
            initPorts();
            
            // set fovea position ([0 0 0 0])
            values.zero();
            
        }
        
        /** 
         * Destructor
         **/
        ~StereoHeadingFovea() {  }
        
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
