/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * \defgroup neuralNetworks neuralNetworks 
 *  
 * @ingroup ctrlLib
 *
 * Classes for neural networking
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __NEURALNETWORKS_H__
#define __NEURALNETWORKS_H__

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <iCub/ctrl/math.h>
#include <deque>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup neuralNetworks
*
* Feed-forward 2 layers Neural Network. 
* Useful to implement the networks trained via MATLAB (e.g. 
* through nftool). 
*/
class ff2LayNN
{
protected:
    struct minmax
    {
       double min;
       double max;
    };
    
    // 1st layer data
    std::deque<yarp::sig::Vector> IW;
    yarp::sig::Vector b1;
    
    // 2nd layer data
    std::deque<yarp::sig::Vector> LW;
    yarp::sig::Vector b2;
    
    // process input
    std::deque<minmax> inMinMaxX;
    std::deque<minmax> inMinMaxY;
    
    // process output
    std::deque<minmax> outMinMaxX;
    std::deque<minmax> outMinMaxY;

    yarp::sig::Vector inMinX;
    yarp::sig::Vector inMinY;
    yarp::sig::Vector inRatio;

    yarp::sig::Vector outMinX;
    yarp::sig::Vector outMinY;
    yarp::sig::Vector outRatio;

    bool configured;

    bool getItem(yarp::os::Property &options, const char *tag, yarp::sig::Vector &item);

    virtual yarp::sig::Vector hiddenLayerFcn(const yarp::sig::Vector &x)=0;
    virtual yarp::sig::Vector outputLayerFcn(const yarp::sig::Vector &x)=0;

public:
    /**
    * Create an empty network.
    */ 
    ff2LayNN();

    /**
    * Create and configure the network.
    * @param options contains the parameters to configure the 
    *               network.
    * @see configure() 
    */ 
    ff2LayNN(yarp::os::Property &options);

    /**
    * Configure/reconfigure the network.
    * @param options contains the parameters to configure the 
    *                network according to the convention derived
    *                from MATLAB.
    * @return true/false on success/fail. 
    *  
    * @note Available options are: 
    *  
    * @b numInputNodes <int> : input dimension. 
    *  
    * @b numHiddenNodes <int> : number of nodes of the first layer 
    *    (hidden).
    *  
    * @b numOutputNodes <int> : number of nodes of the second layer 
    *    (output).
    *  
    * @b IW_<j> (<double> <double> ...) : jth weights vector of the 
    *    hidden layer.
    *  
    * @b LW_<j> (<double> <double> ...) : jth weights vector of the 
    *    output layer.
    *  
    * @b b1 (<double> <double> ...) : bias vector of the hidden 
    *    layer.
    *  
    * @b b2 (<double> <double> ...) : bias vector of the output 
    *    layer.
    *  
    * @b inMinMaxX_<j> (<min> <max>) : range of the jth input 
    *    element.
    *  
    * @b inMinMaxY_<j> (<min> <max>) : range where to map the jth 
    *    input element.
    *  
    * @b outMinMaxY_<j> (<min> <max>) : range of the jth output 
    *    element.
    *  
    * @b outMinMaxX_<j> (<min> <max>) : range where to remap back 
    *    the jth output element.
    *  
    * @note all indexes are 0-based. 
    */ 
    virtual bool configure(yarp::os::Property &options);

    /**
    * Return the internal status after a configuration.
    * @return true/false on success/fail.
    */ 
    virtual bool isValid();

    /**
    * Predict the output given a certain input to the network.
    * @param x is the actual input to the network.
    * @return the predicted output.
    */ 
    virtual yarp::sig::Vector predict(const yarp::sig::Vector &x);

    /**
    * Dump the network structure on the screen.
    */ 
    virtual void printStructure();
};


/**
* \ingroup neuralNetworks
*
* Feed-forward 2 layers Neural Network with a tansig function 
* for the hidden nodes and a purelin for the output nodes. 
*/
class ff2LayNN_tansig_purelin : public ff2LayNN
{
protected:
    // implement the tansig
    virtual yarp::sig::Vector hiddenLayerFcn(const yarp::sig::Vector &x);
    // implement the purelin
    virtual yarp::sig::Vector outputLayerFcn(const yarp::sig::Vector &x);

public:
    /**
    * Create an empty network.
    */ 
    ff2LayNN_tansig_purelin();

    /**
    * Create and configure the network.
    * @param options contains the parameters to configure the 
    *               network.
    * @see configure() 
    */ 
    ff2LayNN_tansig_purelin(yarp::os::Property &options);
};

}

}


#endif



