// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONTROL_BASIS_ACTION__H_
#define _CONTROL_BASIS_ACTION__H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>

#include <string>
#include <iostream>
#include <vector>

namespace CB {
    
    /**
     * The Abstract Control Basis Action Class
     */
    enum ActionState {
        UNKNOWN=0,
        UNDEFINED,
        UNCONVERGED,
        CONVERGED
    };

    class ControlBasisAction : public yarp::os::Thread {
        
    protected:

        /**
         * The dynamic state of the action
         **/
        ActionState dynamicState;

        /**
         * the number of interations the 
         * action has been running
         **/
        int iterations;

        /**
         * running flag
         **/
        bool running;

        /**
         * update rate delay
         **/
        double updateDelay;

        /**
         * YARP output port
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> outputPorts;
        
        /**
         * YARP output port name
         **/
        std::vector<std::string> outputPortNames;

        /**
         * YARP output names
         **/
        std::vector<std::string> outputNames;

        /**
         * num output ports
         **/
        int numOutputs;

        /**
         * the name of the action
         **/ 
        std::string actionName;
        
    public:
        
        /** 
         * Getter for the action name
         * \return the name
         **/
        std::string getActionName() { return actionName; } 

        /** 
         * Getter for action state
         * \return the state
         **/
        ActionState getState() { return dynamicState; }
                
        /** 
         * is the resource running?
         * \return running
         **/
        bool isActionRunning() { return running; }        
                
        /** 
         * virtual update function
         **/
        virtual bool updateAction()=0;

        /**
         * virtual start function
         **/
        virtual void startAction()=0;

        /**
         * virtual stop function
         **/
        virtual void stopAction()=0;

        /**
         * virtual post data function
         **/
        virtual void postData()=0;
               
        /**
         * run function
         **/
        void run() {

            std::cout << "configuring action output information..." << std::endl;
            outputPorts.clear();
            outputPortNames.clear();
            for(int i=0; i<numOutputs; i++) {
                outputPortNames.push_back(actionName + "/" + outputNames[i] +  ":o");
                outputPorts.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);
                outputPorts[i]->open(outputPortNames[i].c_str());
            }

            std::cout << "starting action update loop...." << std::endl;
            while(!isStopping()) {
                if(!updateAction()) {
                    std::cout << "Problem updating control action!!" << std::endl;
                    break;
                }
                
                // post input/output data to ports
                if(numOutputs > 0) postData();
                //if(numInputs > 0) getInputData();
                
                yarp::os::Time::delay(updateDelay);
            }
            std::cout << "ControlBasisAction::run() -- setting running flag to false and closing ports" << std::endl;
            running = false;

            // close ports
            std::cout << "ControlBasisAction closing outputports" << std::endl;
            //for(int i=0; i<numInputs; i++) input[i]->close(); 
            for(int i=0; i<outputPorts.size(); i++) outputPorts[i]->close(); 
                        
        }
      
        /** 
         * reset function.  closes and clears the ports.
         **/
        void reset() {
            std::cout << "ControlBasisAction reset..." << std::endl;
            for(int i=0; i<outputPorts.size(); i++) {
                outputPorts[i]->close();
            }
            outputPorts.clear();
            outputPortNames.clear();
            outputNames.clear();
            dynamicState=UNKNOWN;
            running = false;
            iterations = 0;
        }

        /**
         * Constructor
         **/
        ControlBasisAction() :
            dynamicState(UNKNOWN),
            updateDelay(0.01),
            running(false),
            numOutputs(0),
            iterations(0)
        {
        }

        /**
         * Destructor
         **/
        ~ControlBasisAction() {  
            std::cout << "ControlBasisAction destructor..." << std::endl;
            reset();
        }

    };
    
}

#endif
