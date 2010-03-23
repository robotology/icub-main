// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONTACT_SET__H_
#define _CONTACT_SET__H_

#include "ControlBasisResource.h"

namespace CB {

    /**
     * This class instantiates the abstract ControlBasisResource class for a set of contact sensors 
     * resource. This type of resource provides a single value for (possibly real-values) N contacts.
     * It is still abstract, in that it doesn't implement a runnable resource. 
     * This class must be extended for specific force sensor device that implements the 
     * start, update, and stop functions.
     **/
    class ContactSet : public ControlBasisResource {       
        
    private:

        //yarp::sig::Vector contactValues;

    public:
        
        /**
         * getter for the number of contacts
         **/
        int getNumContacts() { 
            return size;
        }

        /**
         * Constructor
         **/
        ContactSet() :
            ControlBasisResource("contactset", 0, 1) 
        {        
            std::cout << "setting type of ContactSet to " << type.c_str() << std::endl;            
            outputName.push_back("data");            
        }   
 
        /**
         * Destructor
         **/        
        ~ContactSet() { };      
 
       /**
         * This is the function that posts the resource data to the output port.
         * it is type specific, so it is defined here.  it is automatically called 
         * after the update() function in the main thread loop.
         **/
        void postData() {
            yarp::os::Bottle &b = outputPort[0]->prepare();
            b.clear();
            b.addString(resourceName.c_str());
            for(int i=0; i<size; i++) {
                b.addDouble(values[i]);
            }
            outputPort[0]->write();
        }

        /**   
         * getInputData() function. 
         */
        virtual void getInputData() { }
        
    };
    
}

#endif


