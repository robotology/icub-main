// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONFIGURATION_VARIABLES__H_
#define _CONFIGURATION_VARIABLES__H_

#include "ControlBasisResource.h"

#include <yarp/sig/Matrix.h>
#include <math.h>

namespace CB {
    
    /**
     * This class instantiates the abstract ControlBasisResource class for a ConfigurationVariable type resource.
     * This type of resource can be used to read and write joint values to a robot manipulator of the position of a mobile robot.
     * It is still abstract, in that it doesn't implement a runnable resource. This class must be extended for
     * a specific device that implements the start, update, and stop functions.
     **/
    class ConfigurationVariables : public ControlBasisResource {
        
    protected:
        
        /**
         * The number of DOFs of the robot
         **/
        int numDOFs;
        
        /**
         * The number of links of the robot
         **/
        int numLinks;
    
        /**
         * whether this set of variables can accept inputs
         **/
        bool moveable;
        
        /**
         * The desired values if the variables are writeable
         **/
        yarp::sig::Vector desiredValues;

        /**
         * The max increment
         **/
        double maxSetVal;
        
        /**
         * The min range limits of the variables
         **/
        yarp::sig::Vector minLimits;
        
        /**
         * The max range limits of the variables
         **/
        yarp::sig::Vector maxLimits;
        
        /**
         * The DH Parameters for this robot
         **/
        yarp::sig::Matrix DHParameters;
        
        /**
         * The link type for each link (prismatic, revolute, constant, non-interfering)
         **/
        yarp::sig::Vector LinkTypes;
        
        /**
         * Setter for the number of DOFs
         * \param the number of DOFs
         **/
        void setNumDOF(int n) { numDOFs = n; }
        
        /**
         * Setter for the number of Linkls
         * \param the number of Links
         **/        
        void setNumLinks(int l) { numLinks = l; }

        /**
         * function to make sure input date is of
         * safe values
         **/
        yarp::sig::Vector trimInputData(const yarp::sig::Vector &Vdes) {

            yarp::sig::Vector Vout(Vdes.size());
            int maxID = -1;
            double maxVal = -100000;
            
            // copy desired delta vector
            Vout = Vdes;
            
            // get biggest component
            for (int i = 0; i < numDOFs; i++) {
                if (fabs(Vout[i]) > maxSetVal) {
                    maxID = i;
                    maxVal = fabs(Vout[i]);
                    std::cout << "found value[" << maxID << "] too big: " << maxVal << std::endl;
                }
            }
            
            // see if the biggest component is bigger than max allowable
            if (maxVal > maxSetVal) {
                // normalize all the joints to the max val
                for (int i = 0; i < numDOFs; i++) {
                    Vout[i] = (Vout[i] / maxVal) * maxSetVal;
                }
            }
            
            // filter small values
            for (int i = 0; i < numDOFs; i++) {
                if (fabs(Vout[i]) < 1e-10) Vout[i] = 0;
            }

            return Vout;
        }
 
    public:
        
        /**
         * gets the number of DOFs of the device.
         **/
        int getNumDOF() { return numDOFs; }

        /**
         * gets the number of links of the device.
         **/
        int getNumLinks() { return numLinks; }

        /**
         * Gets the value of a particular DOF
         **/
        double getVal(int n) { return values[n]; }

        /**
         * gets the min joint limit for a specified DOF.
         **/
        double getMinLimit(int n) { return minLimits[n]; }

        /**
         * gets the max joint limit for a specified DOF.
         **/
        double getMaxLimit(int n) { return maxLimits[n]; }

        /**
         * flag specifying if the resource accepts input commands that move the device.
         **/
        bool isMoveable() { return moveable; } 

        /**
         * constructor.  sets type and port names.
         **/
        ConfigurationVariables() :
            ControlBasisResource("configuration", 2, 3) 
        {        
            std::cout << "setting type of ConfigurationVariables to " << type.c_str() << std::endl;

            running=false;

            inputName.push_back("data");
            inputName.push_back("lock");        

            outputName.push_back("data");
            outputName.push_back("limits");
            outputName.push_back("params");

        }    

        /** 
         * Destructor
         **/
        ~ConfigurationVariables() { }
        
        /**
         * This is the function that posts the resource data to the output port.
         * it is type specific, so it is defined here.  it is automatically called 
         * after the update() function in the main thread loop.
         **/
        void postData() {
            
            // prepare the output bottles to post information           
            yarp::os::Bottle &b0 = outputPort[0]->prepare();
            yarp::os::Bottle &b1 = outputPort[1]->prepare();
            yarp::os::Bottle &b2 = outputPort[2]->prepare();
 
            b0.clear();
            b1.clear();
            b2.clear();

            b0.addInt(numDOFs);
            b1.addInt(numDOFs);
            
            for(int i = 0; i < numDOFs; i++) {
                
                // add position to output port
                b0.addDouble(values[i]);
                
                // add limit information to output port
                b1.addDouble(minLimits[i]);
                b1.addDouble(maxLimits[i]);
                    
            }
            
            // write the information to the ports
            outputPort[0]->write();      
            outputPort[1]->write();      
            
            b2.addInt(numLinks);
            for(int i=0; i < numLinks; i++) {
                for(int j = 0; j < 4; j++) {
                    b2.addDouble(DHParameters[j][i]);
                }
                b2.addInt(LinkTypes[i]);
            }
            outputPort[2]->write();
            
        }


        /**
         * This is the function that posts the input data to the hardware
         * It is automatically called after the update() function in the main 
         * thread loop.
         **/
        virtual void getInputData() {

            // If the resource is not moveable, there shouldn't be an input port.
            // In other words, no other service should be sending inputs to this resource.
            if(!moveable) {
                numInputs = 0;
                for(int i=0; i<inputPort.size(); i++) {
                    inputPort[i]->close();
                    }
                inputPort.clear();
                return;
            }
            
            yarp::os::Bottle *b[2];
            yarp::sig::Vector inData(numDOFs);

            b[0] = inputPort[0]->read(false);      
            b[1] = inputPort[1]->read(false);      

            if(b[0]!=NULL) {

                // parse joint values
                for(int i = 0; i < numDOFs; i++)  {
                    inData[i] = b[0]->get(i).asDouble();
                }

                // trim input delta by max allowable magnitude
                yarp::sig::Vector trimmed = trimInputData(inData);
                setDesiredIncrement(trimmed);

            }

            // see if there has been a lock change
            if(b[1]!=NULL) {
                lock = (bool)(b[1]->get(0).asInt());
            }

        }

        /**
         * this function addes the input increment to the current joint position
         * to get the desired position.
         * \param Vinc the desired increment to be added
         **/
        void setDesiredIncrement(const yarp::sig::Vector &Vinc) {
            if(Vinc.size() != numDOFs) {
                std::cout << "ConfigurationVariables::setDesiredIncrement() -- MISMATCH -- size=" << Vinc.size() << std::endl;
                return;
            }
            for(int i=0; i<Vinc.size(); i++) {
                desiredValues[i] = values[i]+Vinc[i];
            }
        }

        
    };
    
}

#endif

