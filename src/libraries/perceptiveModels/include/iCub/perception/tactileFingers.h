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
 * @defgroup TactileFingers tactileFingers
 * @ingroup percmod_Implementations 
 *  
 * A very simple wrapper that replace the \ref SpringyFingers 
 * model with a perception based on the tactile sensors.
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section intro_sec Description
 *
 * This framework cannot be called properly a model because it 
 * does not perform any particular computation but rather 
 * collects data from the fingers tips and provide them to the 
 * user for contact detection. 
 */ 

#ifndef __PERCEPTIVEMODELS_TACTILEFINGERS_H__
#define __PERCEPTIVEMODELS_TACTILEFINGERS_H__

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <iCub/perception/sensors.h>
#include <iCub/perception/nodes.h>
#include <iCub/perception/models.h>


namespace iCub
{

namespace perception
{

/**
* @ingroup TactileFingers
*  
* An implementation of the Node class that represents the 
* tactile finger. 
*/
class TactileFinger : public Node
{
protected:
    bool directLogic;
    double outputGain;

    bool extractSensorsData(yarp::sig::Vector &in) const;

public:
    /**
    * Configure the finger taking its parameters from a Property 
    * object. 
    * @param options a Property containing the configuration 
    *                parameters. Available options are:\n
    * <b>name</b>: the name of the finger; it can be "thumb", 
    * "index", "middle", "ring" or "little".\n 
    * <b>logic</b>: it can be "direct" or "inverse". A direct logic 
    * indicates that the attached sensors represent the floating 
    * condition (no external forces excerted on the tactile patch) 
    * with the maximum value (e.g. 255 for a byte). 
    * <b>output_gain</b>: a double that is used to multiply the 
    * final output for normalization purpose.\n  
    * @return true/false on success/failure.
    */
    bool fromProperty(const yarp::os::Property &options);

    /**
    * Return a Property representation of all the node parameters.
    * @param options a Property filled with the configuration 
    *                parameters.
    */
    void toProperty(yarp::os::Property &options) const;

    /**
    * Similar to the toProperty() method but it operates on output
    * streams (e.g. string, ofstream, ...). It allows to better 
    * manage the storing of the configuration over files. 
    * @param str the reference to the output stream. 
    * @return true/false on success/failure. 
    * @see toProperty 
    */
    bool toStream(std::ostream &str) const;

    /**
    * Not available.
    * @return true.
    */
    bool calibrate(const yarp::os::Property &options);

    /**
    * Retrieve tactile data from the ports.
    * @param data a Value containing the representation of the data 
    *             in the format: ((in (1.0 2.0 3.0 4.0 ...))).
    * @return true/false on success/failure. 
    */
    bool getSensorsData(yarp::os::Value &data) const;

    /**
    * Retrieve the finger output.
    * @param out a Value containing the finger output in the form: 
    *            output_gain*max(f(sensors_data)), where f() can be
    *            either 255-sensors_data or sensors_data itself
    *            depending on the logic, direct or inverse
    *            respectively.
    * @return true/false on success/failure. 
    */
    bool getOutput(yarp::os::Value &out) const;    

    /**
    * Not available.
    * @return true.
    */
    bool isCalibrated() const;

    /**
    * Retrieve the status of internal logic.
    * @return true if the internal logic is direct.
    */
    bool isDirectLogic() const
    {
        return directLogic;
    }
};


/**
* @ingroup TactileFingers
*  
* A class that provides a mesaure of contact detection for each 
* finger relying on tactile sensors. 
*/
class TactileFingersModel : public virtual Model
{
private:
    std::string type;
    std::string robot;
    int verbosity;

    SensorPort    sensPort[60];
    TactileFinger fingers[5];
    bool configured;
    bool compensation;

    yarp::os::BufferedPort<yarp::os::Bottle> *port;

    int printMessage(const int level, const char *format, ...) const;
    void close();

public:
    /**
    * Constructor. 
    */
    TactileFingersModel();

    /**
    * Configure the model taking its parameters from a Property 
    * object. 
    * @param options a Property containing the configuration 
    *                parameters. Available options are:\n
    * <b>name</b>: the name of the model.\n 
    * <b>type</b>: the handedness type; it can be either "left" or 
    * "right".\n 
    * <b>robot</b>: the name of the robot to connect to; e.g. "icub" 
    * or "icubSim".\n 
    * <b>compensation</b>: it can be "true" or "false" and indicates 
    * which ports to connect to for data acquisition. When false it 
    * collects the raw data, when true it connects to the 
    * compensated ports. 
    * <b>verbosity</b>: an integer that accounts for the verbosity 
    * level of model print-outs. 
    * @return true/false on success/failure.
    */
    bool fromProperty(const yarp::os::Property &options);

    /**
    * Return a Property representation of all the model parameters.
    * @param options a Property filled with the configuration 
    *                parameters.
    */
    void toProperty(yarp::os::Property &options) const;

    /**
    * Similar to the toProperty() method but it operates on output
    * streams (e.g. string, ofstream, ...). It allows to better 
    * manage the storing of the configuration over files. 
    * @param str the reference to the output stream. 
    * @return true/false on success/failure. 
    * @see toProperty 
    */
    bool toStream(std::ostream &str) const;

    /**
    * Not available.
    * @return true.
    */
    bool calibrate(const yarp::os::Property &options);

    /**
    * Not available.
    * @return true.
    */
    bool isCalibrated() const;

    /**
    * Retrieve the complete output of the model.
    * @param out a Value containing the model output in the form: 
    *            (thumb_out index_out ... little_out), where the
    *            finger_out is the output of the corresponding
    *            finger.
    * @return true/false on success/failure. 
    */
    bool getOutput(yarp::os::Value &out) const;    

    /**
    * Destructor. 
    */
    virtual ~TactileFingersModel();
};


}

}

#endif


