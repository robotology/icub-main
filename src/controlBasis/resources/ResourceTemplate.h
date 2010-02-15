// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _RESOURCE_TEMPLATE__H_
#define _RESOURCE_TEMPLATE__H_

#include "ResourceTypeTemplate.h"

namespace CB {

    /**
     * This class implements the a  control basis resource of the type specified
     * by the TypeTemplate (not a real type, but used for this example template).
     * This resource will be run to provide a  resource for control basis 
     * applications that can be used in control programs.  Existing resources do things
     * like publish the configuration data of a robot manipulator or the Cartesian
     * position of the end-effector of a manipulator (computed via the Forward Kinematics).
     **/    
    class ResourceTemplate : public ResourceTypeTemplate {
        
    public:
        
        /**
         * Constructor.  This will configure the ports and the names for the device.
         **/
        ResourceTemplate(std::string devName, std::string yarpDevName, 
                            int dofs=0, int links=0) :
            ResourceTypeTemplate()
        {

            // init the input/output ports
            initPorts();

        }
        
        /**
         * Destructor
         **/
        ~ResourceTemplate() { }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        virtual bool updateResource() {
            // update the data that this service is publishing
            return true;
        }

        /**
         * Inherited start function
         **/
        virtual void startResource() {

            // do any connections to hardware or other services that might be necessary
            running = true;
            start();     // mandatory start function
        }
        /**
         * Inherited stop function
         **/
        virtual void stopResource() {
            stop(); // mandatory stop function
            // disconnect any connections to hardware or other service
        }


    };
    
}

#endif
