// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _YARP_CONFIGURATION_VARIABLES__H_
#define _YARP_CONFIGURATION_VARIABLES__H_

#include "ConfigurationVariables.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <cb.h>

namespace CB {

    /**
     * This class implements the ConfigurationVariables abstract resource 
     * interface for a generic YARP motor interface.  It allows to control the 
     * device using either the position controller (using a PolyDriver) or
     * the velocity controller (using a velocityControl module).  Both accept
     * position commands, but which is used is dependent on the lower level 
     * control properties of the device.  For reading the encoder positions of 
     * the device, the PolyDriver is used in both cases (regardless of how the 
     * device is controlled).
     **/    
    class YARPConfigurationVariables : public ConfigurationVariables {
        
    protected:

        /**
         * the name of the device port (local) to connect to that provides data
         **/
        std::string localDevPort;

        /**
         * the name of the device port (remote) to connect to that provides data
         **/
        std::string remoteDevPort;

        /**
         * the name of the device
         **/
        std::string yarpDeviceName;

        /**
         * boolean flag specifying if resource is connected to device
         **/ 
        bool connectedToDevice;

        /**
         * Properties for configuring the YARP Motor driver
         **/
        yarp::os::Property options;

        /**
         * The YARP Motor Driver
         **/
        yarp::dev::PolyDriver *dd;

        /**
         * Pointer to the Position Controller 
         **/
        yarp::dev::IPositionControl *pos;

        /**
         * Pointer to the Encoder data
         **/
        yarp::dev::IEncoders *enc;

        /**
         * Pointer to the joint limit data from the driver
         **/
        yarp::dev::IControlLimits *lim;     

        /**
         * Vectorial mask to specify which of the resource 
         * DOFs get mapped to the motor driver DOFs.  
         * For example: if you only want to use every other 
         * (even) DOF of a 6 DOF yarp device, then you could 
         * set this mask to [0 1 0 1 0 1]. The resulting 
         * ConfigurationResource should be specfied as a 3 DOF system. 
         **/
        yarp::sig::Vector mask;

        /**
         * If using this resource with a velocityController to control 
         * the device, this is the port to connect to.
         **/
        yarp::os::BufferedPort<yarp::sig::Vector> velocityPort;

        /**
         * The RPC port for the velocity controller.  This is necessary
         * to connect to to specify the gains and maxVel of the device.
         * By default, the velocityControl module sets these to 0.
         **/
        yarp::os::Port velocityRPCPort;

        /**
         * The local port name of the velocityController specification data.
         **/
        std::string velocityPortName;

        /**
         * The local port name of the velocityController RPC port for configuring.
         **/
        std::string velocityRPCPortName;

        /**
         * Boolean flag specifying if this resource will set positions through 
         * a velocityControl module or through the motor interface driver.
         **/
        bool velocityControlMode;

        /** 
         * The gain for the velocityControl module.
         **/
        double velocityGain;

    public:
        
        /**
         * Constructor.  This will configure the ports and the names for the device.
         * \param devName The name of the resource.  Could be the YARP device name or not.
         * \param yarpDevName The yarp device name (if different from devName). If it's the same, can set to "".
         * \param dofs The number of DOFs of the resource (default to 0, if specified using a config file).
         * \param links The number of links of the resource (default to 0, if specified using a config file).
         **/
        YARPConfigurationVariables(std::string devName, std::string yarpDevName, 
                                   int dofs=0, int links=0)  :           
              connectedToDevice(false),
              velocityGain(10),
              velocityControlMode(false)
        {

            std::cout << "YARPConfigurationVariables created..." << std::endl;  

            numDOFs = dofs;
            numLinks = links;
            size=dofs;
            moveable = true;
            lock = true;
            maxSetVal=0.01;

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

            // initialize storage vectors and matrices
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

            // init the input/output ports
            initPorts();

        }
        
        /**
         * Empty Constructor
         **/
        YARPConfigurationVariables() :
              connectedToDevice(false),
              velocityGain(10),
              velocityControlMode(false)              
        {
            numDOFs = 0;
            numLinks = 0;
            size=0;
            moveable = false;
            lock = true;
            maxSetVal=0.0;
        }

        /**
         * Destructor
         **/
        ~YARPConfigurationVariables() { }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        virtual bool updateResource();

        /**
         * Inherited start function
         **/
        virtual void startResource();

        /**
         * Inherited stop function
         **/
        virtual void stopResource();

        /**
         * Function to connect to the YARP device.
         **/
        virtual bool connectToDevice();

        /**
         * Function to load config data from a file (e.g., DH Parameter info)
         **/
        void loadConfig(std::string fname);

        /**
         * Function for setting the DOF mask vector.
         **/
        void setMask(yarp::sig::Vector m);

        /**
         * Function for setting velocity control mode.  If set to true, must connect 
         * to a velocityControl module running for the device.
         * \param mode Boolean for specifying velocity controller mode.
         * \param portName The command port name of the velocityControl module.
         **/
        void setVelocityControlMode(bool mode, std::string portName);

        /**
         * Gets the velocity control mode flag.
         **/
        bool getVelocityControlMode();

    };
    
}

#endif
