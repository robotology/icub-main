// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _COSINE_FIELD__H_
#define _COSINE_FIELD__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {
    
    class CosineField : public ControlBasisPotentialFunction {
        
    protected:

        yarp::sig::Vector minLimits;
        yarp::sig::Vector maxLimits;
        yarp::sig::Vector ranges;
        yarp::sig::Vector centers;

    public:
        
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
        
        ~CosineField() { }

        // functions from ControlBasisPotentialFunction
        bool updatePotentialFunction();
        void startPotentialFunction();
        void stopPotentialFunction();
        bool connectToInputs();


    };
    
}

#endif
