// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _COSINE_FIELD__H_
#define _COSINE_FIELD__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a CB Potential Function that puts a Sum-of-Cosines
     * around the joint centers of a configuration.  Moving wrt this function will 
     * put the manipulator in the center of its range-of-motion.
     **/    
    class CosineField : public ControlBasisPotentialFunction {
        
    protected:

        /**
         * the min limits of the manipulator
         **/
        yarp::sig::Vector minLimits;

        /**
         * the max limits of the manipulator
         **/
        yarp::sig::Vector maxLimits;

        /**
         * the ranges of the manipulator
         **/
        yarp::sig::Vector ranges;

        /**
         * the joint centers of the manipulator
         **/
        yarp::sig::Vector centers;

    public:
        
        /**
         * Constructor.
         * \param inName the name of the manipulator device 
         **/
        CosineField(std::string inName) {
            
            inputName[0] = inName;

            std::cout << "Creating new CosineField PotentialFunction for " << inputName[0].c_str() << std::endl;

            // temp allocation, will resize later when connect to inputs
            input[0].resize(1);
            minLimits.resize(1);
            maxLimits.resize(1);
            ranges.resize(1);
            centers.resize(1);
            gradient.resize(1);

            hasReference = false;
            running = false;

            potential = 0;
            inputSpace = "configuration";
            pfTypeName = "cosfield_pf";

            connectedToInputs = false;

        }

        /**
         * Destructor
         **/        
        ~CosineField() { }

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


    };
    
}

#endif
