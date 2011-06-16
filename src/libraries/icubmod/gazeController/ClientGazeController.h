/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * \defgroup clientgazecontroller clientgazecontroller
 * @ingroup icub_hardware_modules 
 *  
 * Implements the client part of the <a 
 * href="http://eris.liralab.it/yarpdoc/d2/df5/classyarp_1_1dev_1_1IGazeControl.html">Gaze 
 * Control Interface</a>. 
 *  
 * Copyright (C) 2010 RobotCub Consortium.
 *
 * Author: Ugo Pattacini
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at 
 * src/modules/gazeController/ClientGazeController.h 
 */

#ifndef __CLIENTGAZECONTROLLER_H__
#define __CLIENTGAZECONTROLLER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

#include <string>
#include <set>


class ClientGazeController : public yarp::dev::DeviceDriver,
                             public yarp::dev::IGazeControl
{
protected:
    bool connected;
    bool closed;

    double timeout;
    double lastFpMsgArrivalTime;
    double lastAngMsgArrivalTime;

    yarp::sig::Vector fixationPoint;
    yarp::sig::Vector angles;

    yarp::os::BufferedPort<yarp::sig::Vector> portStateFp;
    yarp::os::BufferedPort<yarp::sig::Vector> portStateAng;
    yarp::os::BufferedPort<yarp::sig::Vector> portStateHead;

    yarp::os::Port portCmdFp;
    yarp::os::Port portCmdAng;
    yarp::os::Port portCmdMono;
    yarp::os::Port portCmdStereo;

    yarp::os::Port portRpc;

    std::set<int> contextIdList;

    virtual bool deleteContexts();
    virtual bool getPose(const std::string &poseSel, yarp::sig::Vector &x, yarp::sig::Vector &o);
    virtual bool blockNeckJoint(const std::string &joint, const double min, const double max);
    virtual bool blockNeckJoint(const std::string &joint, const int j);
    virtual bool getNeckJointRange(const std::string &joint, double *min, double *max);
    virtual bool clearNeckJoint(const std::string &joint);

public:
    ClientGazeController();
    ClientGazeController(yarp::os::Searchable &config);

    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();

    virtual bool setTrackingMode(const bool f);
    virtual bool getTrackingMode(bool *f);
    virtual bool getFixationPoint(yarp::sig::Vector &fp);
    virtual bool getAngles(yarp::sig::Vector &ang);
    virtual bool lookAtFixationPoint(const yarp::sig::Vector &fp);
    virtual bool lookAtAbsAngles(const yarp::sig::Vector &ang);
    virtual bool lookAtRelAngles(const yarp::sig::Vector &ang);
    virtual bool lookAtMonoPixel(const int camSel, const yarp::sig::Vector &px, const double z=1.0);
    virtual bool lookAtStereoPixels(const yarp::sig::Vector &pxl, const yarp::sig::Vector &pxr);
    virtual bool getNeckTrajTime(double *t);
    virtual bool getEyesTrajTime(double *t);
    virtual bool getLeftEyePose(yarp::sig::Vector &x, yarp::sig::Vector &o);
    virtual bool getRightEyePose(yarp::sig::Vector &x, yarp::sig::Vector &o);
    virtual bool getHeadPose(yarp::sig::Vector &x, yarp::sig::Vector &o);
    virtual bool getJointsDesired(yarp::sig::Vector &qdes);
    virtual bool getJointsVelocities(yarp::sig::Vector &qdot);
    virtual bool getStereoOptions(yarp::os::Bottle &options);
    virtual bool setNeckTrajTime(const double t);
    virtual bool setEyesTrajTime(const double t);
    virtual bool setStereoOptions(const yarp::os::Bottle &options);
    virtual bool bindNeckPitch(const double min, const double max);
    virtual bool blockNeckPitch(const double val);
    virtual bool blockNeckPitch();
    virtual bool bindNeckRoll(const double min, const double max);
    virtual bool blockNeckRoll(const double val);
    virtual bool blockNeckRoll();
    virtual bool bindNeckYaw(const double min, const double max);
    virtual bool blockNeckYaw(const double val);
    virtual bool blockNeckYaw();
    virtual bool getNeckPitchRange(double *min, double *max);
    virtual bool getNeckRollRange(double *min, double *max);
    virtual bool getNeckYawRange(double *min, double *max);
    virtual bool clearNeckPitch();
    virtual bool clearNeckRoll();
    virtual bool clearNeckYaw();
    virtual bool checkMotionDone(bool *f);
    virtual bool waitMotionDone(const double period=0.1, const double timeout=0.0);
    virtual bool stopControl();
    virtual bool storeContext(int *id);
    virtual bool restoreContext(const int id);

    virtual ~ClientGazeController();
};


#endif


