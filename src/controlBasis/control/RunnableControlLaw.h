// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _RUNNABLE_CONTROL_LAW__H_
#define _RUNNABLE_CONTROL_LAW__H_

#include "ControlBasisAction.h"
#include "Controller.h"

#include <map>

namespace CB {
    
    /**
     * a class that implements a prioritized (composite) control law.
     **/    
    class RunnableControlLaw : ControlBasisAction {

    protected:

        /** 
         * The list of controllers in the law (in order of decreasing priority)
         **/
        std::vector<Controller *> controllers;

        /**
         * the number of controllers in the law
         **/
        int numControllers;

        /**
         * the composite output signals (one for each synergy)
         **/
        std::vector<yarp::sig::Vector> Vout;

        /**
         * the composite output signals spaces
         **/
        std::vector<std::string> VoutSpaces;

        /**
         * the output device names
         **/
        std::vector<std::string> VoutDeviceNames;

        /**
         * the composite configuration-space output signals (one for each synergy)
        **/
        std::vector<yarp::sig::Vector> VoutConfig;

        /**
         * the list of output (effector) synergies of each controller
         **/
        std::vector<std::string> controllerDeviceNames;

        /**
         * the list of output (effector) synergy spaces of each controller
         **/
        std::vector<std::string> controllerOutputSpaces;

        /**
         * hash map to determine output indices for devices
         **/
        std::map<std::string, int> deviceMap;

        /**
         * hash map to determine jacobians for computing control signal
         **/
        std::map<std::string, int> jacobianMap;

        /**
         * initialliziation flag
         **/
        bool initiallized;

        /**
         * jacobians for transforming control signals
         **/
        std::vector<ControlBasisJacobian *>helperJacobians;

        /**
         * list of whether we need inverse jacobians or not for the transforming Jacobians
         **/
        std::vector<bool> jacobianInverseStore;

        /**
         * the list of output ports to devices that will accept commands
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> devicePorts;

        /**
         * the list of ports to device locks (need to unlock to set values)
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> deviceLockPorts;

        /** 
         * flag determining whether control law has connected to devices yet.
         **/
        bool connectedToDevices;

    public:
        
        /**
         * constructor
         **/
        RunnableControlLaw();
        
        /**
         * destructor
         **/
        ~RunnableControlLaw();

        /**
         * gets the combined controller outputs for the list of synergies
         * \returns Vout the delta output list
         **/
        std::vector<yarp::sig::Vector> getControlOutput() { return VoutConfig; }

        /**
         * gets the combined controller output for the specified device
         * \returns Vout the delta output list
         **/
        yarp::sig::Vector getControlOutput(std::string deviceName) { 
            int id = deviceMap[deviceName];
            return VoutConfig[id]; 
        }

        /**
         * add controller method, from pointer
         **/
        void addController(Controller *c);

        /**
         * gets the number of controllers in the law
         **/
        int getNumControllers() { return numControllers; }
        
        /**
         * add controller method, from names (with reference)
         **/
        void addController(std::string sen, std::string ref, std::string pf, std::string eff, double gain);
        
        /**
         * add controller method, from names 
         **/
        void addController(std::string sen, std::string pf, std::string eff, double gain);
        
        /**
         * function that returns the appropriate jacobian to 
         * transform control objectives
         **/
        yarp::sig::Matrix getJacobian(std::string deviceName, std::string outSpace, std::string inSpace, bool useTranspose);

        /**
         * function to connect control law to devices taht are to be controlled
         **/
        bool connectToDevicePorts();

        /**
         * function to send control signals to devices
         **/
        bool sendOutputsToDevices();

        /**
         * reset function
         **/
        void resetControlLaw();

        /**
         * delete function to delete controller pointers
         **/
        void deleteControllers();

        /**
         * gets the potential of controller n
         **/
        double getControllerPotential(int n) {
            if((n >= 0) && (n < controllers.size()))  {
                return controllers[n]->getControllerPotential();
            } else {
                return 0;
            }        
        }

        /**
         * gets the estimated change in potential of controller n
         **/
        double getControllerPotentialDot(int n) {
            if((n >= 0) && (n<controllers.size()))
                return controllers[n]->getControllerPotentialDot();
            else
                return 0;
        }

        /**
         * gets the state of conroller n
         **/
        int getControllerState(int n) {
            if((n >= 0) && (n<controllers.size()))
                return controllers[n]->getState();
            else
                return 0;
        }

        /**
         * gets the state of conroller n in string format s=(X,-,0,1)
         **/
        std::string getControllerStateString(int n) {
            std::string s_str = "X";
            int s;
            if((n >= 0) && (n<controllers.size())) {
                s = controllers[n]->getState();
                if(s==UNDEFINED) s_str="-";
                else if(s==UNCONVERGED) s_str="0";
                else if(s==CONVERGED) s_str="1";
            }            
            return s_str;
        }

        /** 
         * sets whether to use J^T or J^#
         **/
        void useTranspose(bool b) {
            for(int i=0; i<controllers.size(); i++) {
                controllers[i]->useTranspose(b);
            }
        }

        // inherited functions
        virtual bool updateAction();
        virtual void startAction();
        virtual void stopAction();
        virtual void postData();  

    protected:

        /**
         * initialization function
         **/
        void init();

  };


}

#endif
