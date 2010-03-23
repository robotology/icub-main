// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _FORCE__H_
#define _FORCE__H_

#include "ControlBasisResource.h"

namespace CB {

    /**
     * This class instantiates the abstract ControlBasisResource class for a Cartesian Force 
     * resource. This type of resource provides xyz-force vectors that can be assimulated in
     * force control actions.  It is still abstract, in that it doesn't implement a runnable resource. 
     * This class must be extended for specific force sensor device that implements the 
     * start, update, and stop functions.
     **/
    class Force : public ControlBasisResource {       
        
    private:

        yarp::sig::Vector origin;

    public:
        
        /**
         * returns the contact normal of the force
         **/    
        yarp::sig::Vector get_normal() { 
            double mag = sqrt(values[0]*values[0]+values[1]*values[1]+values[2]*values[2]);
            yarp::sig::Vector normal(3);
            normal.zero();
            if(mag!=0) {
                normal[0] = values[0]/mag;
                normal[1] = values[1]/mag;
                normal[2] = values[2]/mag;
            }
            return normal;
        }

        /**
         * getter for the origin of the coordinate system for the force. the default will be 0.
         **/
        yarp::sig::Vector get_origin() { 
            return origin;
        }
        /**
         * setter for the origin of the coordinate system for the force.
         **/
        void set_origin(yarp::sig::Vector o) {
            origin = o;
        }

        /**
         * Constructor
         **/
        Force() :
            origin(3),
            ControlBasisResource("force", 0, 1) 
        {        
            std::cout << "setting type of Force to " << type.c_str() << std::endl;            
            size = 3;
            values.resize(size);         
            outputName.push_back("data");            
            origin.zero();
        }   
 
        /**
         * Destructor
         **/        
        ~Force() { };      
 
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
            outputPort[0]->write();
        }

        /**   
         * getInputData() function. 
         */
        virtual void getInputData() { }
        
    };
    
}

#endif


