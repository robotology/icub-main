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
 * bla bla bla
*/


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
