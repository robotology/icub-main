/* 
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <string>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <iCub/ctrl/pids.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/**********************************************************/
class FilterModule: public RFModule
{
    BufferedPort<Vector> iPort;
    BufferedPort<Vector> oPort;
    BufferedPort<Vector> bPort;

    MedianFilter gyroFilt;
    MedianFilter magFilt;
    AWLinEstimator velEst;
    Integrator biasInt;

    Vector gyroBias;
    double mag_vel_thres_up;
    double mag_vel_thres_down;
    double bias_gain;
    bool verbose;
    bool adaptGyroBias;

public:
    /**********************************************************/
    FilterModule() : gyroFilt(1,Vector(3,0.0)), magFilt(1,Vector(3,0.0)),
                     velEst(40,0.05), biasInt(0.0,Vector(3,0.0)) { }

    /**********************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.check("name",Value("imuFilter")).asString();
        string robot=rf.check("robot",Value("icub")).asString();
        size_t gyro_order=(size_t)rf.check("gyro-order",Value(5)).asInt();
        size_t mag_order=(size_t)rf.check("mag-order",Value(51)).asInt();
        mag_vel_thres_up=rf.check("mag-vel-thres-up",Value(0.04)).asDouble();
        mag_vel_thres_down=rf.check("mag-vel-thres-down",Value(0.02)).asDouble();
        bias_gain=rf.check("bias-gain",Value(0.001)).asDouble();
        verbose=rf.check("verbose");
        
        gyroFilt.setOrder(gyro_order);
        magFilt.setOrder(mag_order);
        biasInt.setTs(bias_gain);
        gyroBias.resize(3,0.0);
        adaptGyroBias=false;

        iPort.open("/"+name+"/inertial:i");
        oPort.open("/"+name+"/inertial:o");
        bPort.open("/"+name+"/bias:o");

        string imuPortName=("/"+robot+"/inertial");
        if (!Network::connect(imuPortName,iPort.getName()))
            yWarning("Unable to connect to %s",imuPortName.c_str());

        Time::turboBoost();
        return true;
    }

    /**********************************************************/
    double getPeriod()
    {
        return 0.0; // sync on incoming data
    }

    /**********************************************************/
    bool updateModule()
    {
        Vector *imuData=iPort.read();
        if (imuData==NULL)
            return false;

        Stamp stamp;
        iPort.getEnvelope(stamp);

        double t0=Time::now();
        Vector gyro=imuData->subVector(6,8);
        Vector gyro_filt=gyroFilt.filt(gyro);

        gyro-=gyroBias;
        gyro_filt-=gyroBias;

        Vector mag_filt=magFilt.filt(imuData->subVector(9,11));
        double magVel=norm(velEst.estimate(AWPolyElement(mag_filt,stamp.getTime())));

        adaptGyroBias=adaptGyroBias?(magVel<mag_vel_thres_up):(magVel<mag_vel_thres_down);
        gyroBias=biasInt.integrate(adaptGyroBias?gyro_filt:Vector(3,0.0));
        double dt=Time::now()-t0;

        if (oPort.getOutputCount()>0)
        {
            Vector &outData=oPort.prepare();
            outData=*imuData;
            outData.setSubvector(6,gyro);
            oPort.setEnvelope(stamp);
            oPort.write();
        }

        if (bPort.getOutputCount()>0)
        {
            bPort.prepare()=gyroBias;
            bPort.setEnvelope(stamp);
            bPort.write();
        }

        if (verbose)
        {
            yInfo("magVel   = %g => [%s]",magVel,adaptGyroBias?"adapt-gyroBias":"no-adaption");
            yInfo("gyro     = %s",gyro.toString(3,3).c_str());
            yInfo("gyroBias = %s",gyroBias.toString(3,3).c_str());
            yInfo("dt       = %.0f [us]",dt*1e6);
            yInfo("\n");
        }

        return true;
    }

    /**********************************************************/
    bool interruptModule()
    {
        iPort.interrupt();
        oPort.interrupt();
        bPort.interrupt();
        return true; 
    }

    /**********************************************************/
    bool close()
    {
        iPort.close();
        oPort.close();
        bPort.close();
        return true;
    }
};


/**********************************************************/
int main(int argc, char *argv[])
{   
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    ResourceFinder rf;
    rf.configure(argc,argv);

    FilterModule filter;
    return filter.runModule(rf);
}


