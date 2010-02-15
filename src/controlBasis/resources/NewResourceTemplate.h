// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _NEW_RESOURCE_TEMPLATE__H_
#define _NEW_RESOURCE_TEMPLATE__H_

#include "NewResourceTypeTemplate.h"

namespace CB {

    /**
     * This class implements the a new control basis resource of the type specified
     * by the TypeTemplate (not a real type, but used for this example template).
     * This resource will be run to provide a new resource for control basis 
     * applications that can be used in control programs.  Existing resources do things
     * like publish the configuration data of a robot manipulator or the Cartesian
     * position of the end-effector of a manipulator (computed via the Forward Kinematics).
     **/    
    class NewResourceTemplate : public NewResourceTypeTemplate {
        
    public:
        
        /**
         * Constructor.  This will configure the ports and the names for the device.
         **/
        NewResourceTemplate(std::string devName, std::string yarpDevName, 
                                   int dofs=0, int links=0)  :           
        {

            // init the input/output ports
            initPorts();

        }
        
        /**
         * Destructor
         **/
        ~NewResourceTemplate() { }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        bool updateResource() {

            // update the data that this service is publishing

            return true;
        }

        /**
         * Inherited start function
         **/
        void startResource() {

            // do any connections to hardware or other services that might be necessary

            running = true;
            start();     // mandatory start function
        }
        /**
         * Inherited stop function
         **/
        void stopResource() {

            stop(); // mandatory stop function

            // disconnect any connectiosn to hardware or other service

        }


    };
    
}

#endif
