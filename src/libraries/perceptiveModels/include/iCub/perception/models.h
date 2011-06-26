/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @defgroup percmod_Models Models
 * @ingroup percmod_Interfaces 
 *  
 * Classes for distributing perceptive frameworks.
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section intro_sec Description
 *
 * A Model is meant to be a container that encapsulates a 
 * collection of nodes representing the perceptive architecture 
 * and provides the higher layers suitable routines to perform 
 * data acquisition. 
 */ 

#ifndef __PERCEPTIVEMODELS_MODELS_H__
#define __PERCEPTIVEMODELS_MODELS_H__

#include <string>
#include <map>
#include <ostream>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <iCub/perception/nodes.h>


namespace iCub
{

namespace perception
{

/**
* @ingroup percmod_Models
*  
* An abstract class that provides basic methods for interfacing 
* with the data acquisition. 
*/
class Model
{
protected:
    std::string name;
    std::map<std::string,Node*> nodes;

public:
    /**
    * Constructor. 
    */
    Model();

    /**
    * Retrieve the model name. 
    * @return a string containing the model name. 
    */
    std::string getName() const
    {
        return name;
    }

    /**
    * Attach a node object to the model. 
    * @param node the node object to be attached, so that the mode 
    *             can employ it.
    */
    void attachNode(Node &node);

    /**
    * Retrieve an attached node by its name.
    * @param name the name of the attached node. 
    * @return the pointer to the node. 
    */
    Node* getNode(const std::string &name) const;

    /**
    * Configure the model taking its parameters from a Property 
    * object. 
    * @param options a Property containing the configuration 
    *                parameters.
    * @return true/false on success/failure.
    */
    virtual bool fromProperty(const yarp::os::Property &options) = 0;

    /**
    * Return a Property representation of all the model parameters.
    * @param options a Property filled with the configuration 
    *                parameters.
    */
    virtual void toProperty(yarp::os::Property &options) const = 0;

    /**
    * Similar to the @see toProperty method but it operates on 
    * output streams (e.g. string, ofstream, ...). It allows to 
    * better manage the storing of the configuration over files. 
    * @param str the reference to the output stream. 
    * @return true/false on success/failure. 
    */
    virtual bool toStream(std::ostream &str) const = 0;

    /**
    * Some kinds of models need to be calibrated to properly 
    * operate. This method executes the calibration phase. 
    * @param options a Property containing the calibration 
    *                parameters.
    * @return true/false on success/failure. 
    * @note Usually this method should call the corresponding 
    *       calibrate method of all the nodes attached to the model.
    * @note For models that do not envisage any calibration, this 
    *       method should always return true.
    */
    virtual bool calibrate(const yarp::os::Property &options) = 0;

    /**
    * Return the internal status of the calibration.
    * @return true/false on calibrated/uncalibrated-failure.
    * @note For models that do not envisage any calibration, this 
    *       method should always return true.
    */
    virtual bool isCalibrated() const = 0;

    /**
    * Provide the higher layers with the model output computed over 
    * the attached sensors. 
    * @param out a Value containing the model output.
    * @return true/false on success/failure. 
    */
    virtual bool getOutput(yarp::os::Value &out) const = 0;

    /**
    * Destructor. 
    */
    virtual ~Model() { }
};


}

}

#endif


