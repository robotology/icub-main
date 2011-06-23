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
 * ... 
 *  
 *  
 * @defgroup Sensors Sensors 
 * @ingroup PerceptiveModels
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section sensors_intro_sec Description
 *
 * ... 
 *  
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


