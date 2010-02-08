// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONTROL_BASIS_POTENTIAL_FUNCTION__H_
#define _CONTROL_BASIS_POTENTIAL_FUNCTION__H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>
#include <iostream>

namespace CB {

    /**
     * The Abstract ControlBasis Potential Function class.  Returns potential 
     * and gradient based on input. Implmenters of this class will be required to
     * compute a potential functon evaluated at a certain point, as well as the 
     * gradient of the function at that point.  The type (i.e., cartesianposition) 
     * of the PF must be specified, as well as if this PF needs a reference to
     * compute its current value.
     */
    class ControlBasisPotentialFunction : public yarp::os::Thread {
        
    protected:
        
        /**
         * the name of the potential function
         **/
        std::string pfName;

        /**
         * the name of the potential function type
         **/
        std::string pfTypeName;

        /**
         * running flag
         **/
        bool running;
        
        /**
         * The Yarp output port
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> outputPort;
        
        /**
         * The Yarp output port name
         **/
        std::string outputPortName;
        
        /**
         * update delay (in seconds)
         **/
        double updateDelay;
        
        /**
         * the potential value
         **/
        double potential;
        
        /**
         * the gradient of the potential wrt the input space
         **/
        yarp::sig::Vector gradient;
        
        /**
         * the input vector
         **/
        yarp::sig::Vector input[2];

        /**
         * the name of the input resources
         **/
        std::string inputName[2];
        
        /**
         * the input ports
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> inputPort[2];
        
        /** 
         * The formal type of the potential (e.g., CartesianPosition, ConfigurationVariable, etc.)
         **/
        std::string inputSpace;
        
        /**
         * the size of the input space
         **/
        int size;
        
        /**
         * whether the potential needs a reference signal
         **/
        bool hasReference;
        
        /**
         * connected to inputs flag
         **/
        bool connectedToInputs;
        
    public:
        
        /** 
         * Getter for the potential function name
         * \return the name
         **/
        std::string getPotentialName() { return pfName; }  
        
        /** 
         * Getter for the potential
         * \return the potential
         **/
        double getPotential() { return potential; }  
        
        /** 
         * Getter for the potential gradient
         * \return the gradient
         **/
        yarp::sig::Vector getPotentialGradient() { return gradient; }
        
        /** 
         * is the potential function running?
         * \return running
         **/
        bool isPotentialRunning() { return running; }
        
        /** 
         * Getter for the potential space
         * \return the space
         **/
        std::string getInputSpace() { return inputSpace; }  
        
        /** 
         * Getter for the potential size
         * \return the size
         **/
        int getInputSize() { return size; }  
        
        /**
         * Setter for updated delay
         * \param delay (in seconds)
         **/
        void setUpdateDelay(double t) { updateDelay = t; }
        
        /**
         * connect to input function
         **/
        virtual bool connectToInputs() {
            std::cout << "cb pf connect" << std::endl;
        }
        
        /** 
         * virtual update function
         **/
        virtual bool updatePotentialFunction() {
            std::cout << "cb pf update" << std::endl;
        }

        /**
         * virtual start function
         **/
        virtual void startPotentialFunction() {
            std::cout << "cb pf start" << std::endl;
        }
        
        /**
         * virtual stop function
         **/
        virtual void stopPotentialFunction() {
            std::cout << "cb pf start" << std::endl;
        }
        
        /**
         * virtual post data function to be filled in by abstract interface
         **/
        virtual void postData() {

            yarp::os::Bottle &b = outputPort.prepare();
            b.clear();

            b.addString(pfName.c_str());
            b.addDouble(potential);
            
            for(int i = 0; i < gradient.size(); i++) {
                b.addDouble(gradient[i]);
            }
           
            outputPort.write();
        }
        
        /**
         * main run function for pf (instatiates for thread)
         **/
        void run() {

            // set up port name
            pfName = "/cb/" + inputSpace + "/" + pfTypeName;
            
            std::string prefixStr = "/cb/" + inputSpace;
            std::string tmp = inputName[0];
            int s = prefixStr.size();
            
            tmp.erase(0,s);
            pfName += tmp;
            
            // if the PF has a reference, store its name in the portname as well
            if(hasReference) {
                tmp = inputName[1];
                tmp.erase(0,s);
                pfName += tmp;
            }
            
            std::cout << "ControlBasisPotentialFunction::run() name=" << pfName.c_str() << std::endl;
            
            outputPortName = pfName + ":o";
            outputPort.open(outputPortName.c_str());
            
            while(!isStopping()) {
                if(!updatePotentialFunction()) {
                    std::cout << "Problem updating potential function: " << pfName.c_str() << std::endl;
                    break;
                }
                postData();
                yarp::os::Time::delay(updateDelay);
            }
            std::cout << "ControlBasisPotentialFunction::run() -- setting running flag to false and closing ports" << std::endl;
            running = false;
            
            inputPort[0].close();
            inputPort[1].close();
            outputPort.close(); 
            
        }
        
        /**
         * Constructor
         **/
        ControlBasisPotentialFunction() {
            updateDelay = 0.01;
            running = false;
            size = 1;
        }
        
        /**
         * Destructor
         **/
        ~ControlBasisPotentialFunction() { }
        
    };
    
}

#endif
