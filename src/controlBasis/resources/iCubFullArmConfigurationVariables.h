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

        // bookkeeping stuff
        std::string which_arm;
        std::string armDevPort;
        std::string torsoDevPort;
        bool connectedToDevice;
        bool simulationMode;

        // Motor Driver pointers
        yarp::os::Property options[2];
        yarp::dev::PolyDriver *dd[2];
        yarp::dev::IPositionControl *pos[2];
        yarp::dev::IVelocityControl *vel[2];
        yarp::dev::IEncoders *enc[2];
        yarp::dev::IPidControl *pid[2];
        yarp::dev::IAmplifierControl *amp[2];
        yarp::dev::IControlLimits *lim[2];     

        /**
         * If using this resource with a velocityController to control 
         * the device, this is the port to connect to.
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> velocityPort[2];


        /**
         * The RPC port for the velocity controller.  This is necessary
         * to connect to to specify the gains and maxVel of the device.
         * By default, the velocityControl module sets these to 0.
         **/
        yarp::os::Port velocityRPCPort[2];

        /**
         * The local port name of the velocityController specification data.
         **/
        std::string velocityPortName[2];

        /**
         * The local port name of the velocityController RPC port for configuring.
         **/
        std::string velocityRPCPortName[2];

        /**
         * Boolean flag specifying if this resource will set positions through 
         * a velocityControl module or through the motor interface driver.
         **/
        bool velocityControlMode;

        /** 
         * The gain for the velocityControl module.
         **/
        double velocityGain[2];

    public:
        
        /**
         * Constructor
         **/
        iCubFullArmConfigurationVariables(std::string arm_id, bool sim, std::string configFile) 
        {

            // set initial velocity control stuff
            velocityControlMode = false;
            velocityGain[0] = 10;
            velocityGain[1] = 10;


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
        
        /**
         * Destructor
         **/
        ~iCubFullArmConfigurationVariables() { 
            std::cout << "iCubFullArmConfigurationVariables() destructor..." << std::endl;

            // close down polydrivers
            if(dd[0]!=NULL) dd[0]->close();
            if(dd[1]!=NULL) dd[1]->close();

            // if in velocity control mode, disconnect the connections to the velocityControl module.
            if(velocityControlMode) {
                velocityRPCPort[0].close();
                velocityRPCPort[1].close();
                velocityPort[0].close();
                velocityPort[1].close();
            }

        }
        
                
        /**
         * Inherited update function
         **/
        bool updateResource();

        /**
         * Inherited start function
         **/
        void startResource();

        /**
         * Inherited stop function
         **/
        void stopResource();
        
        /**
         * Function for setting velocity control mode.  If set to true, must connect 
         * to a velocityControl module running for the device.
         * \param mode Boolean for specifying velocity controller mode.
         * \param torsoPortName The command port name of the torso velocityControl module.
         * \param armPortName The command port name of the arm velocityControl module.
         **/
        void setVelocityControlMode(bool mode, std::string torsoPortName, std::string armPortName);

        /**
         * Gets the velocity control mode flag.
         **/
        bool getVelocityControlMode();        // new functions

        /**
         * Function to connect to the YARP device.
         **/
        bool connectToDevice();

        /**
         * Function to load config data from a file (e.g., DH Parameter info)
         **/
        void loadConfig(std::string fname);




    };
    
}

#endif
