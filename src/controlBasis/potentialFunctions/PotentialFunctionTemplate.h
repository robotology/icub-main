// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _POTENTIAL_FUNCTION_TEMPLATE__H_
#define _POTENTIAL_FUNCTION_TEMPLATE__H_

#include "ControlBasisPotentialFunction.h"

namespace CB {

    /**
     * This class implements a CB Potential Function that puts a Sum-of-Cosines
     * around the joint centers of a configuration.  Moving wrt this function will 
     * put the manipulator in the center of its range-of-motion.
     **/    
    class PotentialFunctionTemplate : public ControlBasisPotentialFunction {
        
    public:
        
        /**
         * Empty Constructor, needs to set configuration info
         **/
        PotentialFunctionTemplate() :
            ControlBasisPotentialFunction("name_pf","space",false) // a name for the PF, the space it operates in, and whether the PF requires a reference input
        {

            // temp allocation, will resize later when connect to inputs
            inputs.resize(1);
            gradient.resize(1);

            // if the reference flag is true, inputs should be at least size 2. if not, size should be 1
            // how big each input vector is, depends on the situation.  This should be the same size as
            // the gradient.
        }
        
        /**
         * Destructor
         **/        
        ~PotentialFunctionTemplate() { }

        /**
         * inherited update function
         **/
        virtual bool updatePotentialFunction() {

            // do what needs to be done to compute the following variables

            // 1) gradient  (probably needs to make sure this is the right size)
            // 2) potential

            return true;
        }
        
        /**
         * inherited start function
         **/
        virtual void startPotentialFunction() {
            if(!connectedToInputs) {
                if(!connectToInputs()) {
                    cout << "Couldn't connect to input ports in startPotentialFunction()..." << endl;
                    return;
                }
            }
            running = true;
            start();     // mandatory start function
        }
        
        /**
         * inherited stop function
         **/
        virtual void stopPotentialFunction() {
            stop();     // mandatory stop function
        }
        
        /**
         * inherited connect function
         **/
        virtual bool connectToInputs() {
            // do what needs to be done to connect to inputs...
            return true;
        }

    };
    
}

#endif
