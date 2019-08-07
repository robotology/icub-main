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

#include "ImuFilter.h"
#include <yarp/os/LogStream.h>
#include <cstring>
#include <cmath>

#include <yarp/os/Time.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

namespace yarp {
namespace dev {

ImuFilter::ImuFilter(): gyroFilt(1,Vector(3,0.0)), magFilt(1,Vector(3,0.0)),
                        velEst(40,0.05), biasInt(0.0,Vector(3,0.0)),
                        PassThroughInertial(), PeriodicThread(0.0)
{
}

bool ImuFilter::threadInit() {
    return true;
}
void ImuFilter::run() {

    bool ok{true};
    double ts{0.0};
    Vector gyroFiltered, magn;

    if (! PassThroughInertial::proxyIGyro ||
        ! PassThroughInertial::proxyIMagn) {
        yError()<<"imuFilter: the imu is not attached";
        this->askToStop();
        return;
    }
    ok &= PassThroughInertial::proxyIGyro->getThreeAxisGyroscopeStatus(0) == MAS_OK;
    ok &= PassThroughInertial::proxyIGyro->getThreeAxisGyroscopeMeasure(0, gyro, gyroTs);

    if (!ok){
        yError()<<"imuFilter: unable to get gyro";
        return;
    }

    if (std::fabs(gyroTs - prevTs) < 1e-6 ){
        return;
    }

    ok &= PassThroughInertial::proxyIMagn->getThreeAxisMagnetometerStatus(0) == MAS_OK;
    ok &= PassThroughInertial::proxyIMagn->getThreeAxisMagnetometerMeasure(0, magn, ts);

    if (!ok){
        yError()<<"imuFilter: unable to get magnetometer measures";
        return;
    }

    stampBias.update(ts);
    double t0=Time::now();

    m_mutex.lock();
    gyroFiltered = gyroFilt.filt(gyro);

    gyro -= gyroBias;
    gyroFiltered -= gyroBias;
    m_mutex.unlock();

    Vector mag_filt=magFilt.filt(magn);
    double magVel=yarp::math::norm(velEst.estimate(iCub::ctrl::AWPolyElement(mag_filt,stampBias.getTime())));

    adaptGyroBias=adaptGyroBias?(magVel<mag_vel_thres_up):(magVel<mag_vel_thres_down);
    gyroBias=biasInt.integrate(adaptGyroBias?gyroFiltered:Vector(3,0.0));
    double dt=Time::now()-t0;

    if (bPort.getOutputCount()>0)
    {
        bPort.prepare()=gyroBias;
        bPort.setEnvelope(stampBias);
        bPort.write();
    }

    if (verbose)
    {
        yInfo("imuFilter: magVel   = %g => [%s]",magVel,adaptGyroBias?"adapt-gyroBias":"no-adaption");
        yInfo("imuFilter: gyro     = %s",gyro.toString(3,3).c_str());
        yInfo("imuFilter: gyroBias = %s",gyroBias.toString(3,3).c_str());
        yInfo("imuFilter: dt       = %.0f [us]",dt*1e6);
        yInfo("\n");
    }
    prevTs = gyroTs;
}
void ImuFilter::threadRelease() {}

//DEVICE DRIVER 
bool ImuFilter::open(yarp::os::Searchable& config)
{
    bool ok = true;

    // opening comunication ports
    ok &= PassThroughInertial::open(config);

    if (!ok) {
        yError()<<"ImuFilter: failed to open proxy device";
        return false;
    }

    string name=config.check("name",Value("imuFilter")).asString();
    string robot=config.check("robot",Value("icub")).asString();
    size_t gyro_order=(size_t)config.check("gyro-order",Value(5)).asInt();
    size_t mag_order=(size_t)config.check("mag-order",Value(51)).asInt();

    mag_vel_thres_up=config.check("mag-vel-thres-up",Value(0.04)).asDouble();
    mag_vel_thres_down=config.check("mag-vel-thres-down",Value(0.02)).asDouble();
    bias_gain=config.check("bias-gain",Value(0.001)).asDouble();
    verbose=config.check("verbose");

    gyroFilt.setOrder(gyro_order);
    magFilt.setOrder(mag_order);
    biasInt.setTs(bias_gain);
    gyroBias.resize(3,0.0);
    adaptGyroBias=false;

    m_period=config.check("period",Value(0.01)).asDouble()/1000.0;

    if (name.at(0) != '/')
        name = "/" +name;
    ok &= bPort.open(name+"/bias:o");

    this->setPeriod(m_period);

    if (!ok) {
        yError()<<"imuFilter: failed to open the bias port";
        return false;
    }
    return true;

}

bool ImuFilter::close()
{
    bool ok =  detachAll();
    ok &= PassThroughInertial::close();
    return ok;
}

bool ImuFilter::attachAll(const yarp::dev::PolyDriverList &p) {

    if(PassThroughInertial::attachAll(p)) {
        return this->start();
    }
    else{
        yError()<<"imuFilter: attach failed";
        return false;
    }
}
bool ImuFilter::detachAll() {
    this->PeriodicThread::stop();
    return PassThroughInertial::detachAll();
}

bool ImuFilter::getThreeAxisGyroscopeMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const
{
    if (sens_index != 0)
    {
        yError() << "imuFilter: sens_index must be equal to 0, since there is  only one sensor in consideration";
        return false;
    }

    m_mutex.lock();
    out = gyro;
    timestamp = gyroTs;
    m_mutex.unlock();
    return true;
}

} // namespace dev
} // namespace yarp
