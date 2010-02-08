// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _MANIPULATOR_POSITION_JACOBIAN__H_
#define _MANIPULATOR_POSITION_JACOBIAN__H_

#include "ControlBasisJacobian.h"
#include <iCub/iKinFwd.h>
#include <deque>
#include <cb.h>

namespace CB {

    /**
     * This class implements the generic Jacobian class for a Manipulator Jacobian.
     * the Jacobian is a 3xN matrix where N is the number of DOFs/Joints of the (serial) manipulator.
     **/
    class ManipulatorPositionJacobian : public ControlBasisJacobian {
        
    protected:

        /**
         * a ForwardKinematics helper class.
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
         * Constructor.
         **/        
        ManipulatorPositionJacobian(std::string name, int n=0) {           
 
            inputSize = n;
            outputSize = 3;

            deviceName = name;

            std::cout << "Creating new ManipulatorPositionJacobian for " << deviceName.c_str() << std::endl;
            
            if(inputSize!=0) {
                J.resize(outputSize,inputSize);
                configVals.resize(inputSize);
            } 

            running = false;

            inputSpace = "configuration";
            outputSpace = "cartesianposition";
            
            paramsSet = false;

            // requires the DH parameters and current config values from the Configuration resource
            numInputs = 2;
            inputNames.push_back("params");
            inputNames.push_back("vals");

            inputPorts.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);
            inputPorts.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);

            connectedToInputs = false;
        }
        
        /**
         * destructor
         **/
        ~ManipulatorPositionJacobian() { }

        /**
         * inherited update function
         **/
        bool updateJacobian();

        /**
         * inherited start function
         **/
        void startJacobian();

        /**
         * inherited stop function
         **/
        void stopJacobian();

        /**
         * inherited connect to inputs function
         **/
        bool connectToInputs();

    };
    
}

#endif
