// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-  
#ifndef _JACOBIAN_REGISTER__H_
#define _JACOBIAN_REGISTER__H_

#include <JacobianFactory.h>

// jacobians
#include <ManipulatorPositionJacobian.h>
//#include <ManipulatorOrientationJacobian.h>
#include <HeadingJacobian.h>

namespace CB {

    void registerJacobians() {
        JacobianFactory::instance().registerClass<ManipulatorPositionJacobian>(new ManipulatorPositionJacobian());
        //      JacobianFactory::instance().registerClass<ManipulatorOrientationJacobian>(new ManipulatorOrientationJacobian());
        JacobianFactory::instance().registerClass<HeadingJacobian>(new HeadingJacobian());
    }

}

#endif
