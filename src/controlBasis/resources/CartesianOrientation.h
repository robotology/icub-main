// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CARTESIAN_ORIENTATION__H_
#define _CARTESIAN_ORIENTATION__H_

#include "ControlBasisResource.h"

namespace CB {

    /**
     * This class instantiates the abstract ControlBasisResource class for a CartesianOrientation 
     * type resource. It is still abstract, in that it doesn't implement a runnable resource. 
     * This class must be extended for specific cartesian orientation device that implements the 
     * start, update, and stop functions.  
     *
     * It will be assumed that the orientation is represented in 4D axis-angle cooridinates
     **/
    class CartesianOrientation : public ControlBasisResource {       
        
    public:
        
        /**
         * Constructor
         **/
        CartesianOrientation() :
            ControlBasisResource("cartesianorientation", 0, 1) 
        {        
            std::cout << "setting type of CartesianOrientation to " << type.c_str() << std::endl;            
            size = 4;
            values.resize(size);         
            outputName.push_back("data");            
        }   
 
        /**
         * Destructor
         **/        
        ~CartesianOrientation() { };      
 
       /**
         * This is the function that posts the resource data to the output port.
         * it is type specific, so it is defined here.  it is automatically called 
         * after the update() function in the main thread loop.
         **/
        void postData() {
            yarp::os::Bottle &b = outputPort[0]->prepare();
            b.clear();
            b.addString(resourceName.c_str());
            b.addDouble(values[0]);
            b.addDouble(values[1]);
            b.addDouble(values[2]);
            b.addDouble(values[3]);
            outputPort[0]->write();
        }

        /**   
         * getInputData() function. 
         */
        virtual void getInputData() { }
        
    };
    
}

#endif


