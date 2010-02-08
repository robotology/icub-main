// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONTROL_BASIS_JACOBIAN__H_
#define _CONTROL_BASIS_JACOBIAN__H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Random.h>
#include <yarp/math/SVD.h>

#include <iostream>
#include <string>
#include <vector>

namespace CB {
    
    /**
     * The Abstract Control Basis Potential Function class.  Returns potential and gradient based on input.
     */
    class ControlBasisJacobian : public yarp::os::Thread {
        
    protected:
        
        /**
         * the name of the jacobian     
         **/
        std::string jName;

        /**
         * the name of the input
         **/
        std::string deviceName;

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
         * the Jacobian
         **/
        yarp::sig::Matrix J;

        /**
         * the names of the input resource
         **/
        std::vector<std::string> inputNames;
        
        /**
         * the input ports
         **/
        std::vector<yarp::os::BufferedPort<yarp::os::Bottle> *> inputPorts;
        
        /** 
         * The formal input type of the jacobian (e.g., CartesianPosition, ConfigurationVariable, etc.)
         **/
        std::string inputSpace;
        
        /** 
         * The formal output type of the jacobian (e.g., CartesianPosition, ConfigurationVariable, etc.)
         **/
        std::string outputSpace;
        
        /**
         * the size of the input space
         **/
        int inputSize;
        
        /**
         * the size of the output space
         **/
        int outputSize;
        
        /**
         * connected to inputs flag
         **/
        bool connectedToInputs;

        /**
         * the number of inputs needed to compute jacobian
         **/ 
        int numInputs;
        
    public:
        
        /** 
         * Getter for the potential function name
         * \return the name
         **/
        std::string getJacobianName() { return jName; }  
        
        /** 
         * Getter for the jacobian
         * \return the Jacobian
         **/
        yarp::sig::Matrix getJacobian() { return J; }  

        /** 
         * Getter for the inverse of the jacobian
         * \return the Jacobian Inverse
         **/
        yarp::sig::Matrix getJacobianInverse() { 
            
            yarp::sig::Matrix JT;
            yarp::sig::Matrix Jinv;
            yarp::sig::Matrix JinvT;
            if(outputSize < inputSize) {
                JT.resize(inputSize,outputSize);
                JT = J.transposed();
                JinvT = yarp::math::pinv(JT,0.0);
                Jinv = JinvT.transposed();
            } else {       
                Jinv = yarp::math::pinv(J,0.0); 
            }
            return Jinv;
        }  
        

        /** 
         * Getter for the transpose of the jacobian
         * \return the Jacobian Transpose
         **/
        yarp::sig::Matrix getJacobianTranspose() {            
            yarp::sig::Matrix JT = J.transposed();
            return JT;
        }  

        /** 
         * is the potential function running?
         * \return running
         **/
        bool isJacobianRunning() { return running; }
        
        /** 
         * Getter for the potential space
         * \return the space
         **/
        std::string getInputSpace() { return inputSpace; }  
        
        
        /** 
         * Getter for the input size
         * \return the input size
         **/
        int getInputSize() { return inputSize; }  
        
        
        /** 
         * Getter for the output size
         * \return the output size
         **/
        int getOutputSize() { return outputSize; }  
        
        /**
         * Setter for updated delay
         * \param delay (in seconds)
         **/
        void setUpdateDelay(double t) { updateDelay = t; }
        
        /**
         * connect to inputs
         **/
        virtual bool connectToInputs() {
            std::cout << "cb jacobian connect" << std::endl;
            return false;            
        }
        
        /** 
         * virtual update function
         **/
        virtual bool updateJacobian() {
            std::cout << "cb jacobian update" << std::endl;
            return false;            
        }
        
        /**
         * virtual start function
         **/
        virtual void startJacobian() {
            std::cout << "cb jacobian start" << std::endl;
        }
        
        /**
         * virtual stop function
         **/
        virtual void stopJacobian() {
            std::cout << "cb jacobian start" << std::endl;
        }
        
        /**
         * virtual post data function to be filled in by abstract interface
         **/
        virtual void postData() {
            
            yarp::os::Bottle &b = outputPort.prepare();
            b.clear();
            
            b.addString(jName.c_str());
            b.addInt(inputSize);
            b.addInt(outputSize);
            
            for(int i = 0; i < outputSize; i++) {
                for(int j = 0; j < inputSize; j++){
                    b.addDouble(J[i][j]);
                }
            }
            outputPort.write();
        }
        
        /**
         * main run function for pf (instatiates for thread)
         **/
        void run() {
            
            int randomID = (int)(yarp::os::Random::uniform()*1000.0);
            char *c = (char *)malloc(16);
            sprintf(c,"%d", randomID);
            std::string randomIDstr(c);

            // set up port name
            jName = "/cb/jacobian/" + randomIDstr + "/" + inputSpace + ":" + outputSpace + deviceName;
            
            std::cout << "ControlBasisJacobian::run() name=" << jName.c_str() << std::endl;        
            
            outputPortName = jName + ":o";
            bool ok = outputPort.open(outputPortName.c_str());
            if(!ok) {
                std::cout << "ControlBasisJacobian::run() -- couldnt open output port!!" << std::endl;
                return;
            }            

            while(!isStopping()) {
                if(!updateJacobian()) {
                    std::cout << "Problem updating jacobian: (" << jName.c_str() << ")!!" << std::endl;
                    break;
                }
                postData();
                yarp::os::Time::delay(updateDelay);
            }
            std::cout << "ControlBasisJacobian::run() -- setting running flag to false and closing ports." << std::endl;
            running = false;
            outputPort.close(); 

            for(int i=0; i<inputPorts.size(); i++) {
                inputPorts[i]->close();
            }
            inputPorts.clear();
        }
        
        /**
         * Constructor
         **/
        ControlBasisJacobian() :
            updateDelay(0.1)
        {  }
        
    };
    
}

#endif
