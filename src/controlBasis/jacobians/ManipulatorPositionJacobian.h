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

        /**
         * Port to get the configuration information of the device
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> paramPort;

    public:

        /**
         * Constructor.
         **/        
        ManipulatorPositionJacobian(int n=0) :
            ControlBasisJacobian("configuration", "cartesianposition", n, 3)
        {           

            if(n!=0) {
                inputSize=n;
                J.resize(outputSize,inputSize);
                configVals.resize(inputSize);                
            }  else {
                configVals.resize(1);
            }

            paramsSet = false;

        }
        
        /**
         * destructor
         **/
        ~ManipulatorPositionJacobian() { 
            paramPort.close();
        }

        /**
         * inherited update function
         **/
        virtual bool updateJacobian();

        /**
         * inherited start function
         **/
        virtual void startJacobian();

        /**
         * inherited stop function
         **/
        virtual void stopJacobian();

        /**
         * inherited connect to inputs function
         **/
        virtual bool connectToInputs();

    };
    
}

#endif
