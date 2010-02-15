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
         * The Yarp output ports
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> outputPort;
        
        /**
         * The Yarp output port names
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
        std::string inputName;
        
        /**
         * the input ports
         **/
        yarp::os::BufferedPort<yarp::os::Bottle> inputPort;
        
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
         * Getter for the output space
         * \return the space
         **/
        std::string getOutputSpace() { return outputSpace; }  
                
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
         * virtual connect to inputs
         **/
        virtual bool connectToInputs()=0;
        
        /** 
         * virtual update function
         **/
        virtual bool updateJacobian()=0;
        
        /**
         * virtual start function
         **/
        virtual void startJacobian()=0;
        
        /**
         * virtual stop function
         **/
        virtual void stopJacobian()=0;
        
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
            
            bool ok = true;            
            outputPortName = jName + ":o";
            outputPort.close();
            ok = outputPort.open(outputPortName.c_str());
            if(!ok) {
                std::cout << "ControlBasisJacobian::run() -- couldnt open output port!!" << std::endl;
                return;
            }            

            running = true;
            while(!isStopping()) {
                if(!updateJacobian()) {
                    std::cout << "Problem updating jacobian: (" << jName.c_str() << ")!!" << std::endl;
                    break;
                }
                postData();
                yarp::os::Time::delay(updateDelay);
            }

            running = false;

        }
        
        /**
         * sets the device this Jacobian refers to
         **/
        void setDevice(std::string dev) {
            
            int randomID = (int)(yarp::os::Random::uniform()*1000.0);
            char *c = (char *)malloc(16);
            sprintf(c,"%d", randomID);
            std::string randomIDstr(c);

            deviceName=dev;

            // set up port name
            jName = "/cb/jacobian/" + randomIDstr + "/" + inputSpace + ":" + outputSpace + deviceName;
            
            std::cout << "ControlBasisJacobian::setDevice() name=" << jName.c_str() << std::endl;        
            
        }

        /**
         * Constructor
         **/
        ControlBasisJacobian(std::string inSpace, std::string outSpace, int inSize=0, int outSize=0) :
            inputSpace(inSpace),
            outputSpace(outSpace),
            inputSize(inSize),
            outputSize(outSize),
            updateDelay(0.1),
            running(false),
            connectedToInputs(false)
        {              
            if( (inputSize==0)|| (outputSize==0) ) {
                J.resize(1,1);
            } else {
                J.resize(outputSize,inputSize);
            }
        }

        /**
         * Destructor
         **/
        ~ControlBasisJacobian() {
            inputPort.close();
            outputPort.close(); 
        }
        
    };
    
}

#endif
