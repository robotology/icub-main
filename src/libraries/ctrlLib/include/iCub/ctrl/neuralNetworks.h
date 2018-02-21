/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

/**
 * \defgroup neuralNetworks Neural Networks
 *  
 * @ingroup ctrlLib
 *
 * Classes for neural networking.
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __NEURALNETWORKS_H__
#define __NEURALNETWORKS_H__

#include <iostream>
#include <string>
#include <deque>

#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>
#include <iCub/ctrl/math.h>


namespace iCub
{

namespace ctrl
{

/**
* \ingroup neuralNetworks
*
* Feed-Forward 2 layers Neural Network. 
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

    void prepare();
    void setItem(yarp::os::Property &options, const std::string &tag, const yarp::sig::Vector &item) const;
    bool getItem(const yarp::os::Property &options, const std::string &tag, yarp::sig::Vector &item) const;

public:
    /**
    * Create an empty network.
    */ 
    ff2LayNN();

    /**
    * Create and configure the network.
    * @param options contains the parameters to configure the 
    *               network.
    * @see configure
    */ 
    ff2LayNN(const yarp::os::Property &options);

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
    virtual bool configure(const yarp::os::Property &options);    

    /**
    * Return the internal status after a configuration.
    * @return true/false on success/fail.
    */ 
    virtual bool isValid() const;

    /**
    * Predict the output given a certain input to the network.
    * @param x is the actual input to the network.
    * @return the predicted output.
    */ 
    virtual yarp::sig::Vector predict(const yarp::sig::Vector &x) const;

    /**
    * Retrieve the network structure as a Property object.
    * @param options is the output stream. 
    * @return true/false on success/fail. 
    * @see configure 
    */ 
    virtual bool getStructure(yarp::os::Property &options) const;

    /**
    * Dump tadily the network structure on the stream.
    * @param stream is the output stream. 
    * @return true/false on success/fail.  
    */ 
    virtual bool printStructure(std::ostream &stream=std::cout) const;

    /**
    * Retrieve first layer weights.
    * @return the list of first layer weights.
    */ 
    std::deque<yarp::sig::Vector> &get_IW() { return IW; }

    /**
    * Retrieve second layer weights.
    * @return the list of second layer weights.
    */ 
    std::deque<yarp::sig::Vector> &get_LW() { return LW; }

    /**
    * Retrieve first layer bias.
    * @return the list of first layer bias.
    */ 
    yarp::sig::Vector &get_b1() { return b1; }

    /**
    * Retrieve second layer bias.
    * @return the list of second layer bias.
    */ 
    yarp::sig::Vector &get_b2() { return b2; }

    /**
    * Scale input to be used with the network.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector scaleInputToNetFormat(const yarp::sig::Vector &x) const;

    /**
    * Scale back input from the network's format.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector scaleInputFromNetFormat(const yarp::sig::Vector &x) const;

    /**
    * Scale output to be used with the network.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector scaleOutputToNetFormat(const yarp::sig::Vector &x) const;

    /**
    * Scale back output from the network's format.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector scaleOutputFromNetFormat(const yarp::sig::Vector &x) const;

    /**
    * Hidden Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector hiddenLayerFcn(const yarp::sig::Vector &x) const=0;

    /**
    * Output Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector outputLayerFcn(const yarp::sig::Vector &x) const=0;

    /**
    * Gradient of the Hidden Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector hiddenLayerGrad(const yarp::sig::Vector &x) const=0;

    /**
    * Gradient of the Output Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector outputLayerGrad(const yarp::sig::Vector &x) const=0;
};


/**
* \ingroup neuralNetworks
*
* Feed-Forward 2 layers Neural Network with a tansig function 
* for the hidden nodes and a purelin for the output nodes. 
*/
class ff2LayNN_tansig_purelin : virtual public ff2LayNN
{
public:
    /**
    * Create an empty network.
    */ 
    ff2LayNN_tansig_purelin();

    /**
    * Create and configure the network.
    * @param options contains the parameters to configure the 
    *               network.
    * @see configure
    */ 
    ff2LayNN_tansig_purelin(const yarp::os::Property &options);

    /**
    * Hidden Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector hiddenLayerFcn(const yarp::sig::Vector &x) const;

    /**
    * Output Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector outputLayerFcn(const yarp::sig::Vector &x) const;

    /**
    * Gradient of the Hidden Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector hiddenLayerGrad(const yarp::sig::Vector &x) const;

    /**
    * Gradient of the Output Layer Function.
    * @param x is the input vector.
    * @return the output vector.
    */ 
    virtual yarp::sig::Vector outputLayerGrad(const yarp::sig::Vector &x) const;
};

}

}


#endif



