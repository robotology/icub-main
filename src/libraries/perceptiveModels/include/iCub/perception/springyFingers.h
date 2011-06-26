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
 * @defgroup Implementations Implementations 
 * @ingroup PerceptiveModels 
 * 
 * @defgroup SpringyFingers springyFingers
 * @ingroup Implementations 
 *  
 * An elastic model of the distal joints of the robot's fingers 
 * that enables to perceive contacts with external forces.
 *
 * @author Ugo Pattacini 
 *  
 * Copyright (C) 2011 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *
 * @section intro_sec Description
 *
 * Essentially, this framework attemps to model the elastic 
 * properties of the distal joints of the fingers by carrying 
 * out a training stage where the fingers are left free to move 
 * in the space while acquiring sensors data. As result, the 
 * "springy" relations between the data acquired from motor 
 * joint encoders and the distal joints encoders are learnt 
 * relying on a Least-Squares SVM machine from the \ref 
 * icub_libLM_learning_machines "Learning Machine" library. 
 * Finally, the model output consists of the metric distance 
 * between the vector of distal joints readings as currently 
 * acquired and the vector of quantities predicted by the 
 * machine. 
 *  
 * This method represents a natural extension of the linear 
 * technique proposed in the paper <a 
 * href="http://people.liralab.it/iron/Papers/conference/schmitzEtAlHumanoids10.pdf">pdf</a>. 
 */ 

#ifndef __PERCEPTIVEMODELS_SPRINGYFINGERS_H__
#define __PERCEPTIVEMODELS_SPRINGYFINGERS_H__

#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include <iCub/learningMachine/FixedRangeScaler.h>
#include <iCub/learningMachine/LSSVMLearner.h>
#include <iCub/perception/sensors.h>
#include <iCub/perception/nodes.h>
#include <iCub/perception/models.h>


namespace iCub
{

namespace perception
{

/**
* @ingroup SpringyFingers
*  
* An implementation of the Node class that represents the 
* springy finger. 
*/
class SpringyFinger : public Node
{
protected:
    mutable iCub::learningmachine::FixedRangeScaler scaler;
    mutable iCub::learningmachine::LSSVMLearner     lssvm;
    double  calibratingVelocity;
    double  outputGain;
    bool    calibrated;

    bool extractSensorsData(yarp::sig::Vector &in, yarp::sig::Vector &out) const;

public:
    /**
    * Configure the finger taking its parameters from a Property 
    * object. 
    * @param options a Property containing the configuration 
    *                parameters. Available options are:\n
    * <b>name</b>: the name of the finger; it can be "thumb", 
    * "index", "middle", "ring" or "little".\n 
    * <b>calib_vel</b>: a double that specifies the velocity [deg/s]
    * with which the finger is actuated during calibration.\n 
    * <b>output_gain</b>: a double that is used to multiply the 
    * final output for normalization purpose.\n 
    * <b>calibrated</b>: it can be "true" or "false" and indicates 
    * whether the configured finger has to be considered calibrated 
    * or not.\n 
    * <b>scaler</b>: the string that configures the internal scaler 
    * used by the learning machine. \see 
    * iCub::learningmachine::FixedRangeScaler.\n 
    * <b>lssvm</b>: the string that configures the Least-Squares SVM 
    * machine. \see iCub::learningmachine::LSSVMLearner.
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
    * Similar to the @see toProperty method but it operates on 
    * output streams (e.g. string, ofstream, ...). It allows to 
    * better manage the storing of the configuration over files. 
    * @param str the reference to the output stream. 
    * @return true/false on success/failure. 
    */
    bool toStream(std::ostream &str) const;

    /**
    * Allow to send calibration commands to the finger.
    * @param options a Property containing the calibration 
    *                parameters. Available options are:\n
    * <b>reset</b>: resets the internal machine.\n
    * <b>feed</b>: feeds the internal machine with the current 
    * couple of input-output data for calibration purpose.\n 
    * <b>train</b>: issues the final training stage after having 
    * collected a sufficient number of input-output data couples 
    * through the "feed" command. 
    * @return true/false on success/failure. 
    */
    bool calibrate(const yarp::os::Property &options);    

    /**
    * Retrieve data finger from the joints used both for calibration 
    * and normal operation. 
    * @param data a Value containing the representation of the data 
    *             in the format: ((in (1.0)) (out (2.0 3.0))).
    * @return true/false on success/failure. 
    */
    bool getSensorsData(yarp::os::Value &data) const;

    /**
    * Retrieve the finger output.
    * @param out a Value containing the finger output in the form: 
    *            output_gain*norm(out-pred).
    * @return true/false on success/failure. 
    */
    bool getOutput(yarp::os::Value &out) const;

    /**
    * Return the internal status of the calibration.
    * @return true/false on calibrated/uncalibrated-failure.
    */
    bool isCalibrated() const
    {
        return calibrated;
    }

    /**
    * Set the finger actuation velocity used while calibrating. 
    * @param vel the velocity [deg/s].
    */
    void setCalibVel(const double vel)
    {
        calibratingVelocity=vel;
    }

    /**
    * Return the finger actuation velocity used while calibrating.
    * @return the velocity [deg/s].
    */
    double getCalibVel() const
    {
        return calibratingVelocity;
    }    
};


/**
* @ingroup SpringyFingers
*  
* A class that provides the user with a suitable framework to 
* deal with the elastic approach for the problem of detecting 
* contacts of fingers with external forces. 
*/
class SpringyFingersModel : public virtual Model
{
private:
    std::string type;
    std::string robot;
    int verbosity;

    SensorInterface sensIF[5];
    SensorPort      sensPort[12];
    SpringyFinger   fingers[5];
    bool configured;

    yarp::os::BufferedPort<yarp::os::Bottle> *port;
    yarp::dev::PolyDriver                     driver;

    yarp::os::Semaphore mutex;

    class CalibThread : public yarp::os::Thread
    {
        SpringyFingersModel *model;
        SpringyFinger       *finger;
        double               min;
        double               max;
        int                  joint;
        bool                 done;

    public:
        CalibThread() : model(NULL), done(false) { }

        void setInfo(SpringyFingersModel *model, SpringyFinger &finger,
                     const int joint, const double min, const double max)
        {
            this->model=model;
            this->finger=&finger;
            this->joint=joint;
            this->min=min;
            this->max=max;
        }

        void run()
        {
            if (!done && (model!=NULL))
            {
                model->calibrateFinger(*finger,joint,min,max);
                done=true;
            }

            Time::delay(0.1);
        }

        bool isDone() const { return done; }
    };
    friend class CalibThread;

    int printMessage(const int level, const char *format, ...) const;
    void calibrateFinger(SpringyFinger &finger, const int joint,
                         const double min, const double max);
    void close();

public:
    /**
    * Constructor. 
    */
    SpringyFingersModel();

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
    * Similar to the @see toProperty method but it operates on 
    * output streams (e.g. string, ofstream, ...). It allows to 
    * better manage the storing of the configuration over files. 
    * @param str the reference to the output stream. 
    * @return true/false on success/failure. 
    */
    bool toStream(std::ostream &str) const;

    /**
    * Execute the fingers calibration.
    * @param options a Property containing the calibration 
    *                parameters. Available options are:\n
    * <b>finger</b>: a string that encodes the finger to calibrate; 
    * it can be "thumb", "index", "middle", "ring", "little", "all", 
    * "all_serial", "all_parallel". The special tags "all" and 
    * "all_serial" serve to calibrate all the fingers consecutively, 
    * whereas the tag "all_parallel" allows calibrating all the 
    * fingers at the same time.
    * @return true/false on success/failure. 
    */
    bool calibrate(const yarp::os::Property &options);

    /**
    * Return the internal status of the calibration.
    * @return true/false on calibrated/uncalibrated-failure.
    */
    bool isCalibrated() const;

    /**
    * Retrieve the complete output of the model.
    * @param out a Value containing the model output in the form: 
    *            (thumb_out index_out ... little_out), where the
    *            finger_out is the output double of the
    *            corresponding finger.
    * @return true/false on success/failure. 
    */
    bool getOutput(yarp::os::Value &out) const;    

    /**
    * Destructor. 
    */
    virtual ~SpringyFingersModel();
};


}

}

#endif


