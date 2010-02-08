// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _MANIPULABILITY_FIELD__H_
#define _MANIPULABILITY_FIELD__H_

#include "ControlBasisPotentialFunction.h"
#include <iCub/iKinFwd.h>
#include <deque>
#include <cb.h>

namespace CB {

    /**
     * This potential function implements Yoshikawa's Manipulability field for a manipulator.
     * This field is defined as \phi= sqrt(det(J*J^T)), where J is the manipulator jacobian.
     **/
    class ManipulabilityField : public ControlBasisPotentialFunction {
        
    protected:

        /**
         * The iKin Limb
         **/
        iKin::iKinLimb kinLimb;

        /**
         * The pionter to the iKin Chain
         **/
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
        
        /**
         * Constructor
         * \param inName, the name of the manipulator
         **/
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
        
        /**
         * Destructor
         **/
        ~ManipulabilityField() { }

        /**
         * inherited update function
         **/
        bool updatePotentialFunction();

        /**
         * inherited start function
         **/
        void startPotentialFunction();

        /**
         * inherited stop function
         **/
        void stopPotentialFunction();

        /**
         * inherited connect function
         **/
        bool connectToInputs();

        /**
         * gets the manipulability metric for the specified Jacobian matrix
         **/
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
