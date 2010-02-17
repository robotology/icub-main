// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _HEADING_JACOBIAN__H_
#define _HEADING_JACOBIAN__H_

#include "ControlBasisJacobian.h"

namespace CB {

    /**
     * This class implements the generic Jacobian class for a simple 2-DOF Heading Jacobian.
     * This Jacobian is a simple 2x2 matrix that maps headings to pan/tilt angles (it, therefore
     * does not consiser vergence angles...)
     **/
    class HeadingJacobian : public ControlBasisJacobian {
        

    public:

        /**
         * Constructor.
         **/        
        HeadingJacobian() :
            ControlBasisJacobian("heading", "configuration", 2, 2)
        {                      
            J.resize(outputSize,inputSize);
            J.zero();
        }
        
        /**
         * destructor
         **/
        ~HeadingJacobian() {  }

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
