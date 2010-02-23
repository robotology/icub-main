// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-      
#ifndef _YARP_ATTENTION_MECHANISM_HEADING__H_
#define _YARP_ATTENTION_MECHANISM_HEADING__H_

#include "Heading.h"
#include <vector>
#include <deque>

namespace CB {
    
    /**
     * Implements the Heading abstract interface to return the 
     * coordinates of an image feature evaluated through the 
     * YARP Attention Mechanism modules
     **/
    class YARPAttentionMechanismHeading : public Heading {
        
    protected:
        
        /**
         * flag for whether the resource is connected to a configuration resource.
         **/
        bool connectedToAttentionMechanism;
        
        /*
         * the image coordinates
         **/
        yarp::sig::Vector imageCoordinates;

        /**
         * a vector to store the estimate
         **/
        yarp::sig::Vector estimate;


        /** 
         * the filter parameter
         **/
        double alpha;

    public:
        
        /**
         * Constructor
         */
        YARPAttentionMechanismHeading(std::string name) 
            : connectedToAttentionMechanism(false),
              imageCoordinates(2),
              estimate(2)
        {

            deviceName=name;
            numInputs = 1;
            inputName.push_back(deviceName + "/attentionMechanism");
            updateDelay=0.01;

            // filiter initialization params
            alpha=0;
            estimate.zero();

            // mandatory inherit function
            initPorts();
                       
        }
        
        /** 
         * Destructor
         **/
        ~YARPAttentionMechanismHeading() { 
        }
        
        /**
         * Inherited update function.
         * \returns success on update
         **/
        bool updateResource();

        /**
         * Inherited start function.
         **/
        void startResource();

        /**
         * Inherited stop function.
         **/
        void stopResource();
        
        /**
         * A functon that connects the EndEffector resource to 
         * the CB configuration device resource device it represents.
         **/
        bool connectToAttentionMechanism();
        
    };
    
}

#endif
