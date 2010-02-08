// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _JACOBIAN_MAP__H_
#define _JACOBIAN_MAP__H_

// jacobians
#include  <ManipulatorPositionJacobian.h>

namespace CB {

    /**
     * This class provides a factory for instantiating Jacobians (from the possible set).
     * It will make finding and configuring the Jacobians easier for the controllers.
     **/     
    class JacobianMap {
        
    public:

        /** 
         * Constructor/
         **/
        JacobianMap() { }

        /**
         * Destructor
         **/
        ~JacobianMap() { }

        /**
         * This functon returns a pointer to a Jacobian class with the specified input/output spaces for the device of the specified name
         * \param inputSpace the input space of the jacobian (e.g., configuration, cartesianposition, etc.)
         * \param outputSpace the output space of the jacobian (e.g., configuration, cartesianposition, etc.)
         * \param devicename the name of the device (or robot) the jacobian will pertain to.
         **/
        ControlBasisJacobian * getJacobian(std::string inputSpace, 
                                           std::string outputSpace, 
                                           std::string deviceName) {
            
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

        /**
         * This function specifies whether the Jacobian requested is an inverse of the one 
         * returned by the ControlBasisJacobian that will compute it.
         * \param inputSpace the input space of the requested jacobian (e.g., configuration, cartesianposition, etc.)
         * \param outputSpace the output space of the requested jacobian (e.g., configuration, cartesianposition, etc.)
         **/
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
