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
 * @defgroup PerceptiveModels perceptiveModels
 * @ingroup icub_libraries 
 *  
 * Abstract layers for dealing with perceptive models framework. 
 *  
 * @section framework_intro_sec Description 
 *  
 * This library provides the user with a collection of objects 
 * in order to organically handle the problem of acquiring data
 * from the robot, execute some first grade processing and then
 * pass them to the higher software layers.
 *  
 * The final objective is to give the raw data a kind of common 
 * structure together with the set of operations that are
 * normally carried out at a preliminary stage such as the 
 * normalization of the values, the data formatting, the 
 * gathering of the data from different sources so that it turns
 * to be convenient to model this structure to then apply 
 * alternative algorithms/techniques decoupling the higher 
 * routines from the acquisition layer. 
 *  
 * As practical example, one may think to address the problem of 
 * sensing external contacts with the fingers of the robot 
 * either by relying on a elastic model of the distal links or 
 * by directly reading the output of the tactile sensors. This 
 * is only a mere implementative aspect (yet quite significant 
 * indeed) from the standpoint of a software designer, thus it 
 * can be relevant to build a sort of wrapper over the 
 * acquisition phase that in turn returns an homogeneous measure 
 * of the amount of contact regardless the source of the raw 
 * data. This is exactly what this library attempts to do. 
 *  
 * Central to the \ref PerceptiveModels platform are the 
 * following concepts: 
 *  
 * - <b>Sensor</b>: a sensor is an object that simply allows to 
 *   retrieve raw data from a source, e.g. a port, a motor
 *   interface (example: the patches of a finger tip are
 *   considered to be sensors).
 *  
 * - <b>Node</b>: a node is an object that performs some 
 *   operations on the data that it can read through the sensors
 *   attached to the node itself (example: a finger can be
 *   treated as a node to which a number of patches are
 *   attached).
 *  
 * - <b>Model</b>: a model is a kind of super-object that 
 *   encapsulates nodes and executes operations on these nodes
 *   when asked by the user; it also provides the results in a
 *   unified format, so that the user might instantiate
 *   different models to easily change the implementation
 *   without affecting the format of the outcome.
 *  
 * @defgroup Sensors Sensors 
 * @ingroup PerceptiveModels
 *  
 * Classes for data acquisition. 
 *  
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section sensors_intro_sec Description
 *
 * A Sensor is an object that allows to retrieve data from 
 * different kinds of sources such as yarp ports and yarp motor 
 * interfaces. A Sensor is normally attached to a Node.
 */ 

#ifndef __PERCEPTIVEMODELS_SENSORS_H__
#define __PERCEPTIVEMODELS_SENSORS_H__

#include <string>

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>


namespace iCub
{

namespace perception
{

/** 
* @ingroup Sensors 
*  
* An abstract class that exposes the basic methods for sensors 
* handling. 
*/
class Sensor
{
protected:
    std::string name;
    bool configured;
    void *source;

public:
    /**
    * Constructor. 
    */
    Sensor();

    /**
    * Retrieve the sensor name. 
    * @return a string containing the sensor name. 
    */
    std::string getName() const { return name; }

    /**
    * Configure the sensor. 
    * @param source a pointer to the underlying structure to which 
    *               the sensor is attached.
    * @param options a Property containing the configuration 
    *                parameters.
    * @return true/false on success/failure.
    */
    virtual bool configure(void *source, const yarp::os::Property &options) = 0;

    /**
    * Retrieve the sensor raw output.
    * @param in a value containing the sensor output.
    * @return true/false on success/failure.
    */
    virtual bool getOutput(yarp::os::Value &in) const = 0;

    /**
    * Destructor. 
    */
    virtual ~Sensor() { }
};


/**
* @ingroup Sensors 
*  
* This class implements the reading of motor joints encoders. 
*/
class SensorInterface : public Sensor
{
protected:
    int size;
    int index;

public:
    /**
    * Configure the sensor. 
    * @param source a pointer to the yarp::dev::IEncoders interface.
    * @param options a Property containing the configuration 
    *                parameters. Available options are:
    * <b>name</b>: the name of the sensor. 
    * <b>size</b>: the size of the whole sensor data vector. 
    * <b>index</b>: the index corresponding to the joint that needs 
    * to be sensed. 
    * @return true/false on success/failure.
    */
    bool configure(void *source, const yarp::os::Property &options);

    /**
    * Retrieve the sensor joint value.
    * @param in a value containing the current joint position.
    * @return true/false on success/failure.
    */
    bool getOutput(yarp::os::Value &in) const;
};


/**
* @ingroup Sensors 
*  
* This class implements the reading of a value from a port. 
*/
class SensorPort : public Sensor
{
protected:
    int index;

public:
    /**
    * Configure the sensor. 
    * @param source a pointer to the yarp::os::Port object.
    * @param options a Property containing the configuration 
    *                parameters. Available options are:
    * <b>name</b>: the name of the sensor. 
    * <b>index</b>: the index corresponding to the double that needs
    * to be retrieved. 
    * @return true/false on success/failure.
    */
    bool configure(void *source, const yarp::os::Property &options);

    /**
    * Retrieve the sensor output.
    * @param in is filled with the current output value.
    * @return true/false on success/failure.
    */
    bool getOutput(yarp::os::Value &in) const;
};


}

}

#endif


