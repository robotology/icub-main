// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _NEW_RESOURCE_TYPE_TEMPLATE__H_
#define _NEW_RESOURCE_TYPE_TEMPLATE__H_

#include "ControlBasisResource.h"

namespace CB {
    
    /**
     * This is a template for classes that instantiate the abstract ControlBasisResource class for a new type of resource.
     * This class should also be abstract, because users will have to implement specific resources of this type as subclasses.
     * Example types (that already exist) include ConfigurationVariables or CartesianPositions.  
     *
     **/
    class NewResourcesTypeTemplate : public ControlBasisResource {
        
    public:
        

        /**
         * constructor.  sets type and port names.
         **/
        NewResourcesTypeTemplate() :
            ControlBasisResource("typename", 2, 3)  // the name of the type, the number of inputs (that other resources will modify), the number of outputs (this resource will publish) 
        {        

            // in this example template, we have 2 inputs and 3 outputs (as specified in the constructor).
            // it is the job of this type class to set the names of these inputs and outputs. Its probably a safe
            // bet that the first output is the "data" port...

            // fill in string names for inputs and outputs
            inputName.push_back("input1");
            inputName.push_back("input2");        

            outputName.push_back("data");
            outputName.push_back("output2");
            outputName.push_back("output3");

        }    

        /** 
         * Destructor
         **/
        ~NewResourcesTypeTemplate() { }
        
        /**
         * This is the function that posts the resource data to the output port.
         * it is type specific, so it is defined here.  it is automatically called 
         * after the update() function in the main thread loop.
         **/
        virtual void postData() {  }

        /**
         * This function will handle reading the input ports and setting the relevent 
         * member variables with the received data.  This could be generic to the type
         * and not specific to the actual resource.  
         **/ 
        virtual void getInputData() {  }
        
    };
    
}

#endif

