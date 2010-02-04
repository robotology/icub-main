// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _YARP_CONFIGURATION_VARIABLES__H_
#define _YARP_CONFIGURATION_VARIABLES__H_

#include "ConfigurationVariables.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <cb.h>

namespace CB {
    
    class YARPConfigurationVariables : public ConfigurationVariables {
        
    protected:

        std::string localDevPort;
        std::string remoteDevPort;
        std::string yarpDeviceName;
        bool connectedToDevice;

        yarp::os::Property options;
        yarp::dev::PolyDriver *dd;

        // Motor Driver pointers
        yarp::dev::IPositionControl *pos;
        yarp::dev::IVelocityControl *vel;
        yarp::dev::IEncoders *enc;
        yarp::dev::IPidControl *pid;
        yarp::dev::IAmplifierControl *amp;
        yarp::dev::IControlLimits *lim;     

        yarp::sig::Vector mask;

        yarp::os::BufferedPort<yarp::sig::Vector> velocityPort;
        yarp::os::Port velocityRPCPort;
        std::string velocityPortName;
        std::string velocityRPCPortName;
        bool velocityControlMode;
        double velocityGain;

    public:
        
        YARPConfigurationVariables(std::string devName, std::string yarpDevName, int dofs=0, int links=0) {

            std::cout << "YARPConfigurationVariables created..." << std::endl;  
            // set configuration info
            deviceName = devName;
            if(yarpDevName=="") {
                yarpDeviceName = devName;
            } else {
                yarpDeviceName = yarpDevName;
            }

            if( (yarpDevName == devName) || (yarpDevName=="") ) {
                localDevPort = yarpDeviceName;
            } else {
                localDevPort = yarpDeviceName + deviceName;
            }
            remoteDevPort = yarpDeviceName;

            numDOFs = dofs;
            numLinks = links;
            size = dofs;

            connectedToDevice = false;
            moveable = true;
            lock = true;
            
            maxSetVal = 0.01;
            velocityGain = 10;

            // initiallize storage vectors and matrices
            if(numDOFs != 0) {
                values.resize(numDOFs);
                desiredValues.resize(numDOFs);
                maxLimits.resize(numDOFs);
                minLimits.resize(numDOFs);
                mask.resize(numDOFs);
                values.zero();
                desiredValues.zero();
                minLimits.zero();
                maxLimits.zero();
                for(int i=0; i<numDOFs; i++) mask[i] = 1;
            }            
            if(numLinks != 0) {
                DHParameters.resize(4,numLinks);
                LinkTypes.resize(numLinks);
                DHParameters.zero();
                LinkTypes.zero();
                mask.resize(1);
                mask[0] = 1;
            }
            
            std::cout << "Creating new YARPConfigurationVariables(name=" << resourceName.c_str() <<
                "dof=" << numDOFs << ", links=" << numLinks << std::endl;

            velocityControlMode = false;
            initPorts();

        }
        
        ~YARPConfigurationVariables() { 
            std::cout << "YARPConfigurationVariables() destructor..." << std::endl;
        }
        
                
        // functions from ControlBasisResource
        bool updateResource();
        void startResource();
        void stopResource();
        
        // new functions
        bool connectToDevice();
        void loadConfig(std::string fname);
        void setMask(yarp::sig::Vector m);
        void setVelocityControlMode(bool mode, std::string portName);
        bool getVelocityControlMode();

    };
    
}

#endif
