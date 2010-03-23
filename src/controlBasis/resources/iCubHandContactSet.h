// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _ICUB_HAND_CONTACT_SET__H_
#define _ICUB_HAND_CONTACT_SET__H_

#include "ContactSet.h"

namespace CB {
    
    /**
     * This class extendes the ContactSet resource type for
     * the iCub hand contact sensors.
     **/    
    class iCubHandContactSet : public ContactSet {

    private:

        /**
         * connected to device flag
         **/
        bool connectedToDevice;

    protected:

        /**
         * use iCub simulator flag
         **/
        bool simulationMode;

        /**
         * which hand (left=0, right=1)
         **/
        int id;

    public:
        
        /**
         * Constructor.  
         * \param useVerge turns on vergence control.
         **/
        iCubHandContactSet(bool simMode,std::string hand)  :           
            simulationMode(simMode)
        {
            
            std::cout << "iCubHandContactSet created sim=" << simMode << ", hand=" << hand << std::endl;  

            if(hand=="left") id = 0;
            else if(hand=="right") id = 1;
            else {
                std::cout << "iCubHanContactSet hand identifier incorrect, must be \'left\' or \'right\'!!!"<<std::endl;
                return;
            }

            connectedToDevice=false;

            size=6;

            std::string robot_prefix;
            if(simulationMode) {
                robot_prefix = "/icubSim";
            } else {
                robot_prefix = "/icub";
            }            

            deviceName = robot_prefix + "/" + hand;

            // zero data
            values.resize(size);
            values.zero();

            numInputs = 1;
            inputName.push_back("contacts");

            // init the input/output ports
            initPorts();

        }
        
        /**
         * Destructor
         **/
        ~iCubHandContactSet() { 
        }
        
        virtual void startResource();

        virtual void stopResource();

        virtual bool updateResource();

        virtual void postData();

        virtual void getInputData();

        virtual bool connectToDevice();
    };
    
}

#endif
