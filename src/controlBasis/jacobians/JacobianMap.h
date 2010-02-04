// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _JACOBIAN_MAP__H_
#define _JACOBIAN_MAP__H_

// jacobians
#include  <ManipulatorPositionJacobian.h>

namespace CB {
    
    class JacobianMap {
        
    public:
        JacobianMap() { }
        ~JacobianMap() { }
        
        ControlBasisJacobian * getJacobian(std::string inputSpace, std::string outputSpace, std::string deviceName) {
            
            ControlBasisJacobian *jacobian;
            if( (inputSpace == "configuration") && (outputSpace == "cartesianposition")) {
                jacobian = new ManipulatorPositionJacobian(deviceName);
            } else if( (inputSpace == "cartesianposition") && (outputSpace == "configuration")) {
                jacobian = new ManipulatorPositionJacobian(deviceName);
            } else {
                std::cout << "JacobianMap -- requesting  unknown Jacobian..." << std::endl;
                return NULL;	
            }
            return jacobian;            
        }
        
    bool needsInverse(std::string inputSpace, std::string outputSpace) {
        
        bool inv;
        if( (inputSpace == "configuration") && (outputSpace == "cartesianposition")) {
            inv = false;
        } else if( (inputSpace == "cartesianposition") && (outputSpace == "configuration")) {
            inv = true;
        } else {
            std::cout << "JacobianMap -- requesting unknown Jacobian inverse info..." << std::endl;
            return false;	
        }
        return inv;
        
    }
       
  };
}
#endif
