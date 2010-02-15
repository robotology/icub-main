// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CARTESIAN_POSITION_REFERENCE__H_
#define _CARTESIAN_POSITION_REFERENCE__H_

#include "CartesianPosition.h"
#include <string>
#include <iostream>

namespace CB {
    
    class CartesianPositionReference : public CartesianPosition {
        
    public:
        
        /**
         * Constructor
         **/        
        CartesianPositionReference(std::string name) {
            
            // set configuration info
            deviceName = name + "/ref";
            running = false;

            initPorts();            
            std::cout << "Creating new CartesianPositionReference, name=" << deviceName.c_str() << std::endl;
            
        }
        
        /**
         * Destructor
         **/        
        ~CartesianPositionReference() { }

        
        // functions from ControlBasisResource
        bool updateResource() { return true; }
        void startResource() { running = true; start(); }
        void stopResource() { stop(); }
        
        // new functions
        void setVals(yarp::sig::Vector ref) {
            if(ref.size() != values.size()) {
                fprintf(stderr, "Couldn't set reference position for %s!!\n", resourceName.c_str());
                return;
            } else {
                values = ref;
            }
        }

    };
    
}

#endif
