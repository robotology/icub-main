// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONTROL_BASIS_RESOURCE__H_
#define _CONTROL_BASIS_RESOURCE__H_

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <vector>
#include <iostream>

namespace CB {
    
#define TODEG 180.0/M_PI
#define TORAD M_PI/180.0

    /**
     * The Abstract Control Basis Sensory and Motor Resource Class.  
     */
    class ControlBasisResource : public yarp::os::Thread {
        
    protected:
        
        /**
         * the name of the device
         **/
        std::string deviceName;
        
        /**
         * the name of the resource
         **/
        std::string resourceName;
        
        /**
         * running flag
         **/
        bool running;
        
        /** 
         * The formal type of the resource (e.g., CartesianPosition, ConfigurationVariable, etc.)
         **/
        std::string type;

        /**
         * The resource data vals
         **/
        yarp::sig::Vector values;

        /**
         * the size of the data values
         **/
        int size;
        
        /**
         * The Yarp output ports
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> outputPort;
        
        /**
         * The Yarp input Ports
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> inputPort;
        
        /**
         * num output ports
         **/
        int numOutputs;
        
        /** 
         * num input ports
         **/
        int numInputs;
        
        /**
         * The Yarp output port names
         **/
        std::vector<std::string> outputPortName;
        
        /** 
         * The YARP input port name
         **/
        std::vector<std::string> inputPortName;

        /**
         * The output names
         **/
        std::vector<std::string> outputName;

        /**
         * The input names
         **/
        std::vector<std::string> inputName;
        
        /**
         * Lock flag to prevent writing vals to hardware
         **/
        bool lock;
        
        /**
         * update delay (in seconds)
         **/
        double updateDelay;

    public:
        
        /** 
         * Getter for the resource name
         * \return the name
         **/
        std::string getResourceName() { return resourceName; }  
        
        /** 
         * Getter for the device name
         * \return the name
         **/
        std::string getDeviceName() { return deviceName; }  
        
        /** 
         * is the resource running?
         * \return running
         **/
        bool isResourceRunning() { return running; }
        
        /** 
         * Getter for the resource type
         * \return the type
         **/
        std::string getResourceType() { return type; }

        /**
         * Getter for the resource data
         **/
        yarp::sig::Vector getResourceData() { return values; }

        /**
         * Getter for the resource data size
         **/
        int getResourceDataSize() { return size; }
        
        /**
         * Setter for updated delay
         * \param delay (in seconds)
         **/
        void setUpdateDelay(double t) { updateDelay = t; }

        /**
         * Setter for locking/unlocking the ability to set 
         * output variables.
         * \param lock
         **/
        void setLock(bool l) { lock = l; }
        
        /** 
         * virtual update function
         **/
        virtual bool updateResource()=0;

        /**
         * virtual start function
         **/
        virtual void startResource()=0;

        /**
         * virtual stop function
         **/
        virtual void stopResource()=0;
        
        /**
         * virtual post data function to be filled in by instantiation
         **/
        virtual void postData()=0;

        /**
         * virtual set data function to be filled in by instantiation
         **/
        virtual void getInputData()=0;
        
        /**
         * Initiallization function for starting ports
         **/
        void initPorts() {
                        
            // set up port names
            resourceName = "/cb/" + type + deviceName;
            
            std::cout << "ControlBasisResource::initPorts() name=" << resourceName.c_str() << std::endl;
            
            std::cout << "configuring " << numOutputs << " outputs for " << resourceName.c_str() << std::endl;
            outputPort.clear();
            outputPortName.clear();
            for(int i=0; i<numOutputs; i++) {
                outputPortName.push_back(resourceName + "/" + outputName[i] +  ":o");
                outputPort.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);
                outputPort[i]->open(outputPortName[i].c_str());
            }

            std::cout << "configuring " << numInputs << " inputs for " << resourceName.c_str() << std::endl;
            inputPort.clear();
            inputPortName.clear();
            for(int i=0; i<numInputs; i++) {
                inputPortName.push_back(resourceName + "/" + inputName[i] +  ":i");
                inputPort.push_back(new yarp::os::BufferedPort<yarp::os::Bottle>);
                inputPort[i]->open(inputPortName[i].c_str());
            }

            std::cout << "ControlBasisResource::initPorts() initialized" << std::endl;
        };

        /**
         * main run function for resource (instatiates for thread)
         **/
        void run() {

            std::cout << "starting update loop for " << resourceName.c_str() << std::endl;
            running = true;
            while(!isStopping() && running) {
                if(!updateResource()) {                    
                    running = false;
                    std::cout << "Problem updating resource: " << resourceName.c_str() << "!!" << std::endl;
                    break;
                }
                // post input/output data to ports
                //                printf("PROCESSING IN/OUT DATA FOR %s\n", resourceName.c_str());
                if(numOutputs > 0) postData();
                if(numInputs > 0) getInputData();
                
                yarp::os::Time::delay(updateDelay);
            }
            std::cout << "ControlBasisResource::run() -- setting running flag to false and closing ports" << std::endl;
            running = false;

            // close ports
            for(int i=0; i<inputPort.size(); i++) inputPort[i]->close(); 
            for(int i=0; i<outputPort.size(); i++) outputPort[i]->close(); 
            std::cout << "ControlBasisResource::run() -- closed ports..." << std::endl;

        }

        /**
         * onStop function
         **/
        void onStop() {
            running = false;
        }
      
        /**
         * Constructor
         **/
        ControlBasisResource(std::string type, int numInputs, int numOutputs) :
            type(type),
            numInputs(numInputs),
            numOutputs(numOutputs),
            updateDelay(0.01),
            lock(true),
            running(false)
        {            
        }

        /** 
         * Destructor
         **/
        ~ControlBasisResource() {          

            for(int i=0; i<inputPort.size(); i++) {
                inputPort[i]->close(); 
                delete inputPort[i];
            }

            for(int i=0; i<outputPort.size(); i++) {
                outputPort[i]->close(); 
                delete outputPort[i];
            }

        }
        
    };
    
}

#endif
