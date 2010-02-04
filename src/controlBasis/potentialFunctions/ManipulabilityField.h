// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _MANIPULABILITY_FIELD__H_
#define _MANIPULABILITY_FIELD__H_

#include "ControlBasisPotentialFunction.h"
#include <iCub/iKinFwd.h>
#include <deque>
#include <cb.h>

namespace CB {
    
    class ManipulabilityField : public ControlBasisPotentialFunction {
        
    protected:

        /**o
         * a ForwardKinematics helper class.
         **/
        iKin::iKinLimb kinLimb;
        iKin::iKinChain *kinChain;

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
        
        ManipulabilityField(std::string inName) {
            
            inputName[0] = inName;

            std::cout << "Creating new ManipulabilityField PotentialFunction for " << inputName[0].c_str() << std::endl;

            input[0].resize(1);
            DHParams.resize(1,1);
            LinkTypes.resize(1);
            gradient.resize(1);

            hasReference = false;
            running = false;

            potential = 0;
            inputSpace = "configuration";
            pfTypeName = "manipulability_pf";

            connectedToInputs = false;

        }
        
        ~ManipulabilityField() { }

        // functions from ControlBasisPotentialFunction
        bool updatePotentialFunction();
        void startPotentialFunction();
        void stopPotentialFunction();
        bool connectToInputs();

        double getManipulability(yarp::sig::Matrix J);


    private:

        // determinant and helper functions
        double determinant(const yarp::sig::Matrix &M);
        double ZeroLLTri(yarp::sig::Matrix M);
        yarp::sig::Matrix RowSwap(yarp::sig::Matrix M, int r1, int r2);
        yarp::sig::Matrix RowMulS(yarp::sig::Matrix M, int row, double value);
        yarp::sig::Matrix RowAddMulS(yarp::sig::Matrix M, int r1, int r2, double value);

    };
    
}

#endif
