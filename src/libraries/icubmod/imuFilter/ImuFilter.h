/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini, Nicolo' Genesio
 * email:  ugo.pattacini@iit.it, nicolo.genesio@iit.it
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

#ifndef IMU_FILTER_H
#define IMU_FILTER_H


#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>

#include "PassThroughInertial.h"

#include <yarp/math/Math.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/pids.h>

#include <vector>
#include <mutex>

#include <iostream>

namespace yarp {
    namespace dev {
        class ImuFilter;
    }
}

/**
* @ingroup icub_mod_library
* @brief imuFilter.h device driver for apply a filter to remove gyro bias
*/

/**
*
\defgroup icub_imuFilter imuFilter

This device driver applies filtering to remove gyro bias. \n
At startup it tries to attach directly to the IMU devcice of the specified robot
or through a MultipleAnalogSensorClient(using the subdevice mechanism).

\section parameters_sec Parameters

--name("imuFilter")          // module's name; all the open ports will be tagged with the prefix /name.

--robot("icub")              // name of the robot to connect to.

--gyro-order(5)              // Order of the median filter for the gyro data

--mag-order(51)              // Order of the median filter for the magnetometer data.

--mag-vel-thres-up(0.04)     // Magnetic field up-threshold for stopping bias adaption.

--mag-vel-thres-down(0.02)   // Magnetic field down-threshold for starting bias adaption.

--bias-gain(0.001)           // Gain to integrate gyro bias.

--verbose(false)             // If specified enable verbosity.

--proxy-remote               // Required only if run with multipleanalgosensorsclient as subdevice, port prefix of the multipleanalogsensorsserver publishing the imu.

--proxy-local                // Required only if run with multipleanalgosensorsclient as subdevice, port prefix of the multipleanalogsensorsclinet to be created.

\subsection deployment how to run the imuFilter

The `imuFilter` can be instantiated using the yarprobotinterface with the following xml files:

\code{.xml}
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


    <device xmlns:xi="http://www.w3.org/2001/XInclude" name="head-imuFilter" type="imuFilter">
        <param name="period">       10                  </param>
        <param name="name">     /imuFilter       </param>


        <action phase="startup" level="5" type="attach">
            <paramlist name="networks">
                <!-- The name of the element can be any string (we use SetOfIMUs to better express its nature).
                     Its value must match the device name in the corresponding body_part-jx_y-inertials.xml file
                     or in body_part-ebX-inertials.xml -->
                <elem name="SetOfIMUs"> head-inertial</elem>
            </paramlist>
        </action>

        <action phase="shutdown" level="5" type="detach" />
    </device
\endcode

\code{.xml}
<?xml version="1.0" encoding="UTF-8" ?>
<!DOCTYPE devices PUBLIC "-//YARP//DTD yarprobotinterface 3.0//EN" "http://www.yarp.it/DTD/yarprobotinterfaceV3.0.dtd">


    <device xmlns:xi="http://www.w3.org/2001/XInclude" name="head-imuFilter_wrapper" type="multipleanalogsensorsserver">
        <param name="period">       10                  </param>
        <param name="name">     /imuFilter       </param>


        <action phase="startup" level="6" type="attach">
            <paramlist name="networks">
                <!-- The name of the element can be any string (we use SetOfIMUs to better express its nature).
                     Its value must match the device name in the corresponding body_part-jx_y-inertials.xml file
                     or in body_part-ebX-inertials.xml -->
                <elem name="SetOfIMUs"> head-imuFilter </elem>
            </paramlist>
        </action>

        <action phase="shutdown" level="6" type="detach" />
    </device>
\endcode

Alternatively it can be run using yarpdev through this command line:

yarpdev --device imuFilter --subdevice multipleanalogsensorsclient --proxy-remote MASServerName --proxy-local MASClientName

\section usage_sec  Usage

The canonical way to use the imuFilter is to instantiate in the client
code a MultipleAnalogSensorsClient specifying as <remote> argument the <name> used
for the imuFilter.

Another way is reading through the streaming port <name>/measures:o.

\section portsc_sec Ports Created

The imuFilter device is executed combined with the network wrapper multipleanalogsensorsserver. It opens the followig ports:

Output ports:

- <name>/bias:o         streams out a yarp::sig::Vector representing the current estimate of the gyro bias.
- <name>/measures:o     streams out the imu data filtered serialized through the yarp::os::Bottle.
- <name>/rpc:o          rpc to retrieve the MultipleAnalogSensorsMetadata.

\author Ugo Pattacini, Nicolo' Genesio
**/


class yarp::dev::ImuFilter :  public yarp::os::PeriodicThread,
                              public yarp::dev::PassThroughInertial
{

    yarp::os::BufferedPort<yarp::sig::Vector> bPort;

    iCub::ctrl::MedianFilter   gyroFilt;
    iCub::ctrl::MedianFilter   magFilt;
    iCub::ctrl::AWLinEstimator velEst;
    iCub::ctrl::Integrator     biasInt;

    yarp::dev::IThreeAxisGyroscopes *iGyro{nullptr};

    yarp::sig::Vector gyroBias, gyro;
    double mag_vel_thres_up{0.0};
    double mag_vel_thres_down{0.0};
    double bias_gain{0.0};
    bool verbose{false};
    bool adaptGyroBias{false};
    double m_period{0.01};
    double gyroTs{0.0};
    mutable std::mutex m_mutex;
    yarp::os::Stamp stampBias;
    double prevTs{0.0};

public:
    ImuFilter();
    virtual ~ImuFilter() = default;

    /* PeriodicThread methods */
    bool threadInit() override;
    void run() override;
    void threadRelease() override;

    /* Device Driver methods */
    bool open(yarp::os::Searchable& config) override;
    bool close() override;

    /* IMultipleWrapper methods */
    bool attachAll(const yarp::dev::PolyDriverList &p) override;
    bool detachAll() override;


    bool getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

};

#endif // IMU_FILTER_H
