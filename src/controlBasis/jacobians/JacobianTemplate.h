// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _JACOBIAN_TEMPLATE__H_
#define _JACOBIAN_TEMPLATE__H_

#include "ControlBasisJacobian.h"

namespace CB {

    /**
     * This is an example class for implementing the generic Jacobian.
     * It will need to specify input and output types, as well as the 
     * the sizes (though this could be done in the connectToInputs() or update()
     * functions.  
     **/
    class JacobianTemplate : public ControlBasisJacobian {
        
    public:

        /**
         * Constructor.
         **/        
        JacobianTemplate(int inSize=0, int outSize=0) :
            ControlBasisJacobian("input_type", "output_type", inSize, outSize)
        {           

            // set the size of the Jacobian
            if((inSize!=0) && (outSize!=0)) {
                inputSize=inSize;
                outputSize=outSize;
                J.resize(outputSize,inputSize);
            }  

        }
        
        /**
         * destructor
         **/
        ~JacobianTemplate() {  }

        /**
         * inherited update function
         **/
        virtual bool updateJacobian() {
            // update J here
            return truel;
        }

        /**
         * inherited start function
         **/
        virtual void startJacobian() {
            start();     // mandatory start function
        }

        /**
         * inherited stop function
         **/
        virtual void stopJacobian() {
            stop();     // mandatory stop function
        }

        /**
         * inherited connect to inputs function
         **/
        virtual bool connectToInputs() {
            // do what needs to be done to go to the appropriate CB resources to get input information
            // necessary to compute Jacobian.  For example, the Manipulator Jacobian needs to connect 
            // to the Configuration data.
            return true;
        }

    };
    
}

#endif
