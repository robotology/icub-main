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
 * @defgroup Models Models
 *  
 * @ingroup PerceptiveModels 
 *  
 * Abstract layers for dealing with perceptive models framework.
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section intro_sec Description
 *
 * ... 
 *  
 */ 

#ifndef __PERCEPTIVEMODELS_MODELS_H__
#define __PERCEPTIVEMODELS_MODELS_H__

#include <string>
#include <map>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <iCub/perception/nodes.h>


namespace iCub
{

namespace perception
{

/**
* @ingroup Models
*
*/
class Model
{
protected:
    std::string name;

    std::map<std::string,Node*> nodes;

public:
    Model();
    std::string getName() const { return name; }

    void attachNode(Node &node);
    Node* getNode(const std::string &name) const;

    virtual bool fromProperty(const yarp::os::Property &options) = 0;
    virtual void toProperty(yarp::os::Property &options) const = 0;
    virtual bool calibrate(const yarp::os::Property &options) = 0;
    virtual bool getOutput(yarp::os::Value &out) const = 0;

    virtual ~Model() { }
};


}

}

#endif


