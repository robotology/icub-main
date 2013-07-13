/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
 * @defgroup nnTraining Training of Neural Networks
 * @ingroup optimization
 *
 * Training of neural networks using optimization methods. 
 * (<b>in progress</b>) 
 *  
 * @author Ugo Pattacini 
 */ 

#ifndef __ICUB_OPT_NNTRAINING_H__
#define __ICUB_OPT_NNTRAINING_H__

#include <deque>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/ctrl/neuralNetworks.h>

#ifdef WIN32
    #pragma warning(disable:4250)
#endif

namespace iCub
{

namespace optimization
{

/**
* @ingroup nnTraining
*
* Class to deal with training of Feed-Forward 2 layers Neural 
* Network using IpOpt. 
*/
class ff2LayNNTrain: virtual public iCub::ctrl::ff2LayNN
{
protected:
    yarp::os::Property bounds;
    void* App;

public:
    /**
    * Default constructor.
    */
    ff2LayNNTrain();

    /**
    * Allow specifying the bounds for training network's parameters.
    * @param bounds a property-like object containing bounds in the 
    *               form: ("tag" (<min> <max>)), where tag is a
    *               string referring to network parameters: e.g. IW,
    *               LW, b1, b2 ...
    */
    void setBounds(const yarp::os::Property &bounds);

    /**
    * Train the network through optimization. 
    * @param numHiddenNodes is the number of hidden nodes. 
    * @param in the list of input vector. 
    * @param out the list of output vector. 
    * @param pred the returned list of predicted output. 
    * @param error returns the prediction error.
    * @return true/false on success/fail. 
    */
    virtual bool train(const unsigned int numHiddenNodes, const std::deque<yarp::sig::Vector> &in,
                       const std::deque<yarp::sig::Vector> &out, std::deque<yarp::sig::Vector> &pred,
                       double &error);

    /**
    * Retrain the network through optimization. 
    * @param numHiddenNodes is the number of hidden nodes. 
    * @param in the list of input vector. 
    * @param out the list of output vector. 
    * @param pred the returned list of predicted output. 
    * @param error returns the prediction error.
    * @return true/false on success/fail. 
    */
    virtual bool retrain(const std::deque<yarp::sig::Vector> &in, const std::deque<yarp::sig::Vector> &out,
                         std::deque<yarp::sig::Vector> &pred, double &error);

    /**
    * Default destructor.
    */
    virtual ~ff2LayNNTrain();
};


/**
* @ingroup nnTraining
*
* Class to deal with training of Feed-Forward 2 layers Neural 
* Network with a tansig function for the hidden nodes and a 
* purelin for the output nodes. 
*/
class ff2LayNNTrain_tansig_purelin: public ff2LayNNTrain,
                                    public iCub::ctrl::ff2LayNN_tansig_purelin                                    
{
};

}
 
}

#endif



