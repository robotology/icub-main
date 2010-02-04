// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _ICUB_FULL_ARM_CONFIGURATION_VARIABLES__H_
#define _ICUB_FULL_ARM_CONFIGURATION_VARIABLES__H_

#include "ConfigurationVariables.h"
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <cb.h>

namespace CB {
    
    class iCubFullArmConfigurationVariables : public ConfigurationVariables {
        
    protected:

        std::string which_arm;
        std::string armDevPort;
        std::string torsoDevPort;
        bool connectedToDevice;
        bool simulationMode;

        yarp::os::Property options[2];
        yarp::dev::PolyDriver *dd[2];

        // Motor Driver pointers
        yarp::dev::IPositionControl *pos[2];
        yarp::dev::IVelocityControl *vel[2];
        yarp::dev::IEncoders *enc[2];
        yarp::dev::IPidControl *pid[2];
        yarp::dev::IAmplifierControl *amp[2];
        yarp::dev::IControlLimits *lim[2];     


    public:
        
        iCubFullArmConfigurationVariables(std::string arm_id, bool sim, std::string configFile) {

            // set configuration info
            if( (arm_id != "left") && (arm_id != "right") ) {
                std::cout << "iCubFullArmConfigurationVariables -- " << arm_id << " not an appropriate id name!!" << std::endl;
                return;
            } 
            which_arm = arm_id;
            simulationMode = sim;
            
            // set up the resource name
            deviceName = "/icub";
            if(simulationMode) deviceName += "Sim";
            deviceName += ("/full_" + arm_id + "_arm");

            // set up names to talk to YARP ports
            armDevPort = "/icub";
            if(simulationMode) armDevPort += "Sim";
            armDevPort += ("/" + arm_id + "_arm");

            torsoDevPort = "/icub";
            if(simulationMode) torsoDevPort += "Sim";
            torsoDevPort += "/torso";

            numDOFs = 10;
            numLinks = 12;
            size = numDOFs;

            connectedToDevice = false;
            moveable = true;
            lock = true;
            
            maxSetVal = 0.01;

            values.resize(numDOFs);
            desiredValues.resize(numDOFs);
            maxLimits.resize(numDOFs);
            minLimits.resize(numDOFs);
            values.zero();
            desiredValues.zero();
            minLimits.zero();
            maxLimits.zero();

            DHParameters.resize(4,numLinks);
            LinkTypes.resize(numLinks);
            DHParameters.zero();
            LinkTypes.zero();
                       
            initPorts();

            std::cout << "Creating new iCubFullArmConfigurationVariables(name=" << resourceName.c_str() <<
                "dof=" << numDOFs << ", links=" << numLinks << std::endl;

            loadConfig(configFile);
            
        }
        
        ~iCubFullArmConfigurationVariables() { 
            std::cout << "iCubFullArmConfigurationVariables() destructor..." << std::endl;
        }
        
                
        // functions from ControlBasisResource
        bool updateResource();
        void startResource();
        void stopResource();
        
        // new functions
        bool connectToDevice();
        void loadConfig(std::string fname);

    };
    
}

#endif
