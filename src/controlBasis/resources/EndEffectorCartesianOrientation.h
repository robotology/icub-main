// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _ENDEFFECTOR_CARTESIANORIENTATION__H_
#define _ENDEFFECTOR_CARTESIANORIENTATION__H_

#include "CartesianOrientation.h"
#include <iCub/iKinFwd.h>
#include <vector>
#include <deque>
#include <cb.h>

namespace CB {
    
    /**
     * Implements the CartesianOrientation abstract interface to retun the 
     * EndEffector orientation of a serial Kinematic chain.  This is determined
     * using the iCub iKin library.
     **/
    class EndEffectorCartesianOrientation : public CartesianOrientation {
        
    protected:
        
        /**
         * flag for whether the resource is connected to a configuration resource.
         **/
        bool connectedToConfiguration;
        
        /**
         * the iKinLimb to store kinematic data.
         **/
        iKin::iKinLimb kinLimb;

        /**
         * the pointer to the iKinChain storing kinematic data.
         **/
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
        EndEffectorCartesianOrientation(std::string name) 
            : connectedToConfiguration(false),
              paramsSet(false)
        {

            deviceName=name;

            numInputs = 3;
            inputName.push_back("config");
            inputName.push_back("params");
            inputName.push_back("limits");

            updateDelay=0.01;

            // mandatory inherit function
            initPorts();

        }
        
        /** 
         * Destructor
         **/
        ~EndEffectorCartesianOrientation() { }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        bool updateResource();

        /**
         * Inherited start function.
         **/
        void startResource();

        /**
         * Inherited stop function.
         **/
        void stopResource();
        
        /**
         * A functon that connects the EndEffector resource to 
         * the CB configuration device resource device it represents.
         **/
        bool connectToConfiguration();
        
    };
    
}

#endif
