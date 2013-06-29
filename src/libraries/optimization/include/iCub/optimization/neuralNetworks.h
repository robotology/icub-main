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
 *  
 * @author Ugo Pattacini 
 */ 

#ifndef __ICUB_OPT_NNTRAINING_H__
#define __ICUB_OPT_NNTRAINING_H__

#include <deque>
#include <yarp/sig/all.h>

#include <iCub/ctrl/neuralNetworks.h>


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
class ff2LayNNTrain: public iCub::ctrl::ff2LayNN
{
public:
    /**
    * Train the network through optimization. 
    * @param numHiddenNodes is the number of hidden nodes. 
    * @param in the list of input vector. 
    * @param out the list of output vector. 
    * @param outn the returned list of predicted output. 
    * @param error returns the prediction error.
    * @return true/false on success/fail. 
    */
    bool train(const unsigned int numHiddenNodes, const std::deque<yarp::sig::Vector> &in,
               const std::deque<yarp::sig::Vector> &out, std::deque<yarp::sig::Vector> &pred,
               double &error);
};

}
 
}

#endif



