// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _ICUB_EYE_CONFIGURATION_VARIABLES__H_
#define _ICUB_EYE_CONFIGURATION_VARIABLES__H_

#include "YARPConfigurationVariables.h"

namespace CB {
    
    /**
     * This class extendes the YARPConfigurationVariables resource type for
     * the iCub eye subsystem.  Because the eyes have a coupled pan/tilt/verge 
     * set of DOFs. If Vergence is on, this resource will accept 4 inputs
     * (corresponding to (pan_left, tilt_left, pan_right, tilt_right).  If
     * vergence is turned off, it will accept 2 inputs (pan, tilt).  
     * If using verge, it will set all 3 eye DOFs (p/t/v).  
     * if not, it will just set the pan and tilt angles.
     **/    
    class iCubEyeConfigurationVariables : public YARPConfigurationVariables {

    private:

        int iterations;

    protected:

        /**
         * accept verge command or just pan/tilt commands.
         **/
        bool useVergence;
        
        /**
         * use iCub simulator flag
         **/
        bool simulationMode;

        /**
         * the number of "virtual" DOFs (either 2=[p,t] or 4=[p_l,t_l,p)r,t_l])
         **/
        int numVirtualDOFs;

        /**
         * Matrix to map virutal input angles to head p/t and (maybe) v angles
         **/
        yarp::sig::Matrix mapping;

        /**
         * Input Vector (either 2 or 4 DOFs)
         **/
        yarp::sig::Vector virtualInputData;

    public:
        
        /**
         * Constructor.  
         * \param useVerge turns on vergence control.
         **/
        iCubEyeConfigurationVariables(bool simMode=false,bool useVerge=true)  :           
            useVergence(useVerge),
            mapping(1,1),
            virtualInputData(1),
            iterations(0)
        {
            
            std::cout << "iCubEyeConfigurationVariables created, vergence=" << useVergence<< std::endl;  

            connectedToDevice=false;
            velocityControlMode=false;              
            velocityGain=10;
            simulationMode=simMode;
            
            if(useVergence) {
                numVirtualDOFs = 4; // [p_l, t_l, p_r, t_r]
                numDOFs = 3; // [t, p, v]
            } else {
                numVirtualDOFs = 2; // [p, t]
                numDOFs = 2; // [t,p]
            }
            
            numLinks = numVirtualDOFs;
            size=numVirtualDOFs;
            moveable = true;
            lock = true;
            maxSetVal=10;

            std::string robot_prefix;
            if(simulationMode) {
                robot_prefix = "/icubSim";
            } else {
                robot_prefix = "/icub";
            }            
            yarpDeviceName = robot_prefix+"/head";

            if(useVergence) {
                deviceName = robot_prefix+"/eyes-ptv";
            } else {
                deviceName = robot_prefix+"/eyes-pt";
            }
            localDevPort = yarpDeviceName + deviceName;            
            remoteDevPort = yarpDeviceName;

            // mask must deal with full 6-DOF head system
            mask.resize(6);     

            // set the mask values to allow only the DOFs we want to control
            mask[0] = 0; // head[0]
            mask[1] = 0; // head[1]
            mask[2] = 0; // head[2]
            mask[3] = 1; // tilt
            mask[4] = 1; // pan
            if(useVergence) mask[5] = 1; // verge

            // virtual DOFs
            virtualInputData.resize(numVirtualDOFs);
            mapping.resize(numDOFs,numVirtualDOFs);
            minLimits.resize(numVirtualDOFs);
            maxLimits.resize(numVirtualDOFs);
            DHParameters.resize(4,numLinks);
            LinkTypes.resize(numLinks);

            // 2- or 3-DOF t/p/(v) commands
            values.resize(numDOFs);
            desiredValues.resize(numDOFs);
            
            // zero data
            values.zero();
            desiredValues.zero();
            DHParameters.zero();
            virtualInputData.zero();
            mapping.zero();
            minLimits.zero();
            maxLimits.zero();

            for(int i=0; i<LinkTypes.size(); i++) {
                LinkTypes[i] = (int)LINK_TYPE_REVOLUTE;
            }

            // init the input/output ports
            initPorts();

        }
        
        /**
         * Destructor
         **/
        ~iCubEyeConfigurationVariables() { 
        }
        
        virtual void startResource();

        virtual bool updateResource();

        virtual void postData();

        virtual void getInputData();

        virtual bool connectToDevice();
    };
    
}

#endif
