// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _HEADING__H_
#define _HEADING__H_

#include "ControlBasisResource.h"

namespace CB {

    /**
     * This class instantiates the abstract ControlBasisResource class for a Heading 
     * type resource. This type of resource provides interesting image coordinates to 
     * control actions.  It is still abstract, in that it doesn't implement a runnable resource. 
     * This class must be extended for specific cartesian position device that implements the 
     * start, update, and stop functions.
     **/
    class Heading : public ControlBasisResource {       
        
    public:
        
        /**
         * returns the horizontal angle of the image coordinate of the position
         **/    
        double getGamma_u() { return values[0]; }

        /**
         * returns the vertical angle of the image coordinate of the position
         **/    
        double getGamma_v() { return values[1]; }

        /**
         * Constructor
         **/
        Heading() :
            ControlBasisResource("heading", 0, 1) 
        {        
            std::cout << "setting type of Heading to " << type.c_str() << std::endl;            
            size = 2;
            values.resize(size);         
            outputName.push_back("data");            
        }   
 
        /**
         * Destructor
         **/        
        ~Heading() { };      
 
       /**
         * This is the function that posts the resource data to the output port.
         * it is type specific, so it is defined here.  it is automatically called 
         * after the update() function in the main thread loop.
         **/
        void postData() {
            yarp::os::Bottle &b = outputPort[0]->prepare();
            b.clear();
            b.addString(resourceName.c_str());
            b.addDouble(getGamma_u());
            b.addDouble(getGamma_v());
            outputPort[0]->write();
        }

        /**   
         * getInputData() function. 
         */
        virtual void getInputData() { }
        
    };
    
}

#endif


