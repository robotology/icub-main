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

        /**
         * port to get limit information
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> limitsInputPort;

    public:
        
        /**
         * Empty Constructor, needs to set configuration info
         **/
        CosineField() :
            ControlBasisPotentialFunction("cosfield_pf","configuration",false)
        {

            // temp allocation, will resize later when connect to inputs
            inputs.resize(1);
            minLimits.resize(1);
            maxLimits.resize(1);
            ranges.resize(1);
            centers.resize(1);
            gradient.resize(1);

            std::cout << "Created new CosineField..." << std::endl;
        }
        
        /**
         * Destructor
         **/        
        ~CosineField() { }

        /**
         * inherited update function
         **/
        virtual bool updatePotentialFunction();
        
        /**
         * inherited start function
         **/
        virtual void startPotentialFunction();
        
        /**
         * inherited stop function
         **/
        virtual void stopPotentialFunction();
        
        /**
         * inherited connect function
         **/
        virtual bool connectToInputs();

    };
    
}

#endif
