// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _ENDEFFECTOR_CARTESIANPOSITION__H_
#define _ENDEFFECTOR_CARTESIANPOSITION__H_

#include "CartesianPosition.h"
#include <iCub/iKinFwd.h>
#include <vector>
#include <deque>
#include <cb.h>

namespace CB {
    
    /**
     * Implements the CartesianPosition abstract interface to retun the 
     * EndEffector position of a serial Kinematic chain.
     **/
    class EndEffectorCartesianPosition : public CartesianPosition {
        
    protected:
        
        /**
         * flag for whether the resource is connected to a configuration resource.
         **/
        bool connectedToConfiguration;
        
        /**
         * a ForwardKinematics helper classes.
         **/
        iKin::iKinLimb kinLimb;
        iKin::iKinChain *kinChain;

        /*
         * the configuration variable values.
         **/
        yarp::sig::Vector configVals;

        /** 
         * storage for DH parameters
         **/
        yarp::sig::Matrix DHParams;

        /**
         * storage for link type information
         **/
        yarp::sig::Vector LinkTypes;

        /**
         * indicator flag concerning whether DH parameter info has been set
         **/
        bool paramsSet;

        /**
         * linked list to the kinematic links
         **/
        std::deque<iKin::iKinLink*> linkList;

    public:
        
        /**
         * Constructor
         */
        EndEffectorCartesianPosition(std::string name) 
        {

            deviceName = name;
            connectedToConfiguration = false;
            paramsSet = false;
            updateDelay = 0.01;
            
            numInputs = 3;
            inputName.push_back("config");
            inputName.push_back("params");
            inputName.push_back("limits");
            initPorts();

        }
        
        /** 
         * Destructor
         **/
        ~EndEffectorCartesianPosition() { }
        
        // functions from ControlBasisResource
        bool updateResource();
        void startResource();
        void stopResource();
        
        bool connectToConfiguration();
        
    };
    
}

#endif
