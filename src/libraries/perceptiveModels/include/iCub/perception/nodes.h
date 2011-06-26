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
 * @defgroup percmod_Nodes Nodes
 * @ingroup percmod_Interfaces 
 *  
 * Abstract classes for data processing.
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section intro_sec Description
 *
 * A Node is the simplest object that allows to process data 
 * acquired from the sensor attached to it. \n 
 * User can build up a complete architecture of nodes linked 
 * each others. 
 */ 

#ifndef __PERCEPTIVEMODELS_NODES_H__
#define __PERCEPTIVEMODELS_NODES_H__

#include <map>
#include <ostream>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>

#include <iCub/perception/sensors.h>


namespace iCub
{

namespace perception
{

/**
* @ingroup percmod_Nodes 
*  
* An abstract class that provides basic events handling.
*/
class EventCallback
{
protected:
    std::string name;

public:
    /**
    * Constructor. 
    */
    EventCallback();

    /**
    * Retrieve the node name. 
    * @return a string containing the node name. 
    */
    std::string getName() const
    {
        return name;
    }

    /**
    * The callback routine. 
    * @param ptr void pointer at user disposal.
    */
    virtual void execute(void* ptr) = 0;
};


/**
* @ingroup percmod_Nodes
*  
* An abstract class that exposes the basic methods for the 
* handling of data acquired through the attached sensors. 
*/
class Node
{
protected:
    std::string name;

    std::map<std::string,Sensor*>        sensors;
    std::map<std::string,EventCallback*> callbacks;
    std::map<std::string,Node*>          neighbors;

public:
    /**
    * Constructor. 
    */
    Node();

    /**
    * Retrieve the node name. 
    * @return a string containing the node name. 
    */
    std::string getName() const
    {
        return name;
    }

    /**
    * Attach a sensor object to the node. 
    * @param sensor the sensor object to be attached, so that the 
    *               node can use it.
    */
    void attachSensor(Sensor &sensor);

    /**
    * Attach an event callback to the node. 
    * @param callback the callback to be attached and that can be 
    *                 raised upon a specific event.
    */
    void attachCallback(EventCallback &callback);

    /**
    * Add a node as a neighbor for the process of building the 
    * architecture. 
    * @param node a node lying in the neighborhood. 
    */
    void addNeighbor(Node &node);

    /**
    * Remove a node previously added as neighbor.
    * @param name the node name uniquely identified. 
    * @return true/false on success/failure. 
    */
    bool removeNeighbor(const std::string &name);

    /**
    * Retrieve a neighbor node by its name.
    * @param name the name of the neighbor node. 
    * @return the pointer to the neighbor node. 
    */
    Node* getNeighbor(const std::string &name) const;

    /**
    * Configure the node taking its parameters from a Property 
    * object. 
    * @param options a Property containing the configuration 
    *                parameters.
    * @return true/false on success/failure.
    */
    virtual bool fromProperty(const yarp::os::Property &options) = 0;

    /**
    * Return a Property representation of all the node parameters.
    * @param options a Property filled with the configuration 
    *                parameters.
    */
    virtual void toProperty(yarp::os::Property &options) const = 0;

    /**
    * Similar to the toProperty() method but it operates on output
    * streams (e.g. string, ofstream, ...). It allows to better 
    * manage the storing of the configuration over files. 
    * @param str the reference to the output stream. 
    * @return true/false on success/failure. 
    */
    virtual bool toStream(std::ostream &str) const = 0;

    /**
    * Some kinds of nodes need to be calibrated to properly operate. 
    * This method executes the calibration phase. 
    * @param options a Property containing the calibration 
    *                parameters.
    * @return true/false on success/failure. 
    * @note For nodes that do not envisage any calibration, this 
    *       method should always return true.
    */
    virtual bool calibrate(const yarp::os::Property &options) = 0;

    /**
    * Return the internal status of the calibration.
    * @return true/false on calibrated/uncalibrated-failure.
    * @note For nodes that do not envisage any calibration, this 
    *       method should always return true.
    */
    virtual bool isCalibrated() const = 0;

    /**
    * Retrieve data from the whole set of attached sensors, giving 
    * back a standard representation of it.
    * @param data a Value containing the representation of the data. 
    *             It can be a double, a string, a collection of
    *             doubles and so on.
    * @return true/false on success/failure. 
    */
    virtual bool getSensorsData(yarp::os::Value &data) const = 0;

    /**
    * Retrieve the node output computed over the sensors data.
    * @param out a Value containing the node output.
    * @return true/false on success/failure. 
    */
    virtual bool getOutput(yarp::os::Value &out) const = 0;

    /**
    * Destructor. 
    */
    virtual ~Node() { }
};


}

}

#endif


