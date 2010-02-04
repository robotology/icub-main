// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _CONFIGURATION_VARIABLES_MODULE__H_
#define _CONFIGURATION_VARIABLES_MODULE___H_

#include "ControlBasisResourceModule.h"

namespace CB {
    
    /**
     * This class instantiates the abstract ControlBasisResourceModule class for a ConfigurationVAriable type resource.
     * This type of resource can be used to read and write joint values to a robot manipulator of the position of a mobile robot.
     **/
    class ConfigurationVariablesModule : public ControlBasisResourceModule {
         
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
        Vector desiredValues;

        /**
         * The maximum amount each variable can be set
         **/
        double maxSetVal;
        
        /**
         * The min range limits of the variables
         **/
        Vector minLimits;
        
        /**
         * The max range limits of the variables
         **/
        Vector maxLimits;
        
        /**
         * The DH Parameters for this robot
         **/
        Matrix DHParameters;
        
        /**
         * The link type for each link (prismatic, revolute, constant, non-interfering)
         **/
        Vector LinkTypes;
        
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
        Vector trimInputData(const Vector &Vdes) {

            Vector Vout(Vdes.size());
            Vector Vdiff(Vdes.size());
            int maxID = -1;
            double maxVal = -100000;

            // get delta difference
            Vdiff = Vdes;//-values; 

            // get biggest component
            for (int i = 0; i < numDOFs; i++)
                {
                    if (fabs(Vdiff[i]) > maxSetVal)
                        {
                            maxID = i;
                            maxVal = fabs(Vdiff[i]);
                            //                            printf("found value[%d] too big: %f\n", maxID, maxVal);
                        }
                }
            
            // see if the biggest component is bigger than max allowable
            if (maxVal > maxSetVal)
                {
                    // normalize all the joints to the max val
                    for (int i = 0; i < numDOFs; i++)
                        {
                            Vdiff[i] = (Vdiff[i] / maxVal) * maxSetVal;
                        }
                }

            for (int i = 0; i < numDOFs; i++)
                {
                    if (fabs(Vdiff[i]) < 1e-10) Vdiff[i] = 0;
                }

            // take the delta, and add to current to get new desired
            Vout = values+Vdiff;

            return Vout;
        }
 
    public:
        
        int getNumDOF() { return numDOFs; }
        int getNumLinks() { return numLinks; }
        double getVal(int n) { return values[n]; }
        double getMinLimit(int n) { return minLimits[n]; }
        double getMaxLimit(int n) { return maxLimits[n]; }
        bool isMoveable() { return moveable; } 

        /**
         * constructor.  sets type name.
         **/
        ConfigurationVariablesModule() {
        
            running = false;

            type = "configuration";
            printf("setting type of ConfigurationVariablesModule to %s\n", type.c_str());
            
            numOutputs = 3;      
            outputName.push_back("data");
            outputName.push_back("limits");
            outputName.push_back("params");

            numInputs = 2;
            inputName.push_back("data");
            inputName.push_back("lock");
        
        }    

        ~ConfigurationVariablesModule() { 
            printf("ConfigurationVariablesModule() destructor...\n");
        }
        
        /**
         * This is the function that posts the resource data to the output port.
         * it is type specific, so it is defined here.  it is automatically called 
         * after the update() function in the main thread loop.
         **/
        void postData() {
            
            //ACE_OS::printf ("Configuration %s posting data\n", resourceName.c_str());
                       
            Bottle &b0 = outputPort[0]->prepare();
            Bottle &b1 = outputPort[1]->prepare();
            Bottle &b2 = outputPort[2]->prepare();
 
            b0.clear();
            b1.clear();
            b2.clear();

            b0.addInt(numDOFs);
            b1.addInt(numDOFs);
            
            //ACE_OS::printf ("pos: (");
            for(int i = 0; i < numDOFs; i++)
                {
                    //      ACE_OS::printf ("%.4f ", values[i]);
                    
                    // add position to output port
                    b0.addDouble(values[i]);
                    
                    // add limit information to output port
                    b1.addDouble(minLimits[i]);
                    b1.addDouble(maxLimits[i]);
                    
                }
            
            //ACE_OS::printf (") -- %s\n", deviceName.c_str());
            //printf("bottle 0: %s\n", b0.toString().c_str());
            //printf("bottle 1: %s\n", b[1].toString().c_str());

            //outputPort[0]->write(b[0]);      
            //outputPort[1]->write(b[1]);      
            outputPort[0]->write();      
            outputPort[1]->write();      
            
            b2.addInt(numLinks);
            for(int i=0; i < numLinks; i++) 
                {
                    for(int j = 0; j < 4; j++)
                        {
                            b2.addDouble(DHParameters[j][i]);
                        }
                    b2.addInt(LinkTypes[i]);
                }
            //outputPort[2]->write(b[2]);
            outputPort[2]->write();

        }


        /**
         * This is the function that posts the input data to the hardware
         * It is automatically called after the update() function in the main 
         * thread loop.
         **/
        void getInputData() {

            Bottle *b[2];
            Vector inData(numDOFs);

            //ACE_OS::printf ("\nConfiguration %s setting data\n", resourceName.c_str());
            
            b[0] = inputPort[0]->read(false);      
            b[1] = inputPort[1]->read(false);      

            if(b[0]!=NULL) {
                //lock = false;
                //                ACE_OS::printf ("config set: (");
                for(int i = 0; i < numDOFs; i++) 
                    {
                        inData[i] = b[0]->get(i).asDouble();
                        //      ACE_OS::printf ("%.3f ", inData[i]);                    
                    }
                //ACE_OS::printf (")\n");            

                desiredValues = trimInputData(inData);

                // ACE_OS::printf ("trimmed to: (");
                //               for(int i = 0; i < numDOFs; i++)
                //  {
                //      ACE_OS::printf ("%.4f ", desiredValues[i]);                    
                //  }
                //                ACE_OS::printf (")\n");            
            }

            if(b[1]!=NULL) {
                lock = (bool)(b[1]->get(0).asInt());
                ACE_OS::printf("setting lock to: %d\n", lock);           
            }

        }


        void setDesiredIncrement(const Vector &Vinc) {
            if(Vinc.size() != numDOFs) {
                printf("ConfigurationVariablesModule::setDesiredIncrement() -- size=%d mismatch!!\n", Vinc.size());
                return;
            }
            //printf("ConfigurationVariablesModule::settingDesiredIncrement:\n", Vinc.size());
            for(int i=0; i<Vinc.size(); i++) {
                desiredValues[i] = values[i]+Vinc[i];
                //printf("%.3f    %.3f    %.3f\n",values[i], Vinc[i], desiredValues[i]);
            }
        }

        
    };
    
}

#endif


