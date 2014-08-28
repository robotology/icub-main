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
 * href="http://wiki.icub.org/yarpdoc/d2/df5/classyarp_1_1dev_1_1IGazeControl.html">Gaze
 * Control Interface</a>. 
 *  
 * @note Please read carefully the \ref icub_gaze_interface
 *       "Gaze Interface" documentation.
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

#include <string>
#include <set>
#include <map>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>


// forward declaration
class ClientGazeController;


/************************************************************************/
class GazeEventHandler : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    ClientGazeController *interface;
    void onRead(yarp::os::Bottle &event);

public:
    GazeEventHandler() : interface(NULL) { }
    void setInterface(ClientGazeController *interface);
};


/************************************************************************/
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

    yarp::os::Stamp   fpStamp;
    yarp::os::Stamp   anglesStamp;

    yarp::os::BufferedPort<yarp::sig::Vector> portStateFp;
    yarp::os::BufferedPort<yarp::sig::Vector> portStateAng;
    yarp::os::BufferedPort<yarp::sig::Vector> portStateHead;

    yarp::os::BufferedPort<yarp::os::Bottle>  portCmdFp;
    yarp::os::BufferedPort<yarp::os::Bottle>  portCmdAng;
    yarp::os::BufferedPort<yarp::os::Bottle>  portCmdMono;
    yarp::os::BufferedPort<yarp::os::Bottle>  portCmdStereo;
    yarp::os::RpcClient                       portRpc;

    std::set<int> contextIdList;
    std::map<std::string,yarp::dev::GazeEvent*> eventsMap;
    GazeEventHandler portEvents;
    friend class GazeEventHandler;

    void init();
    bool deleteContexts();
    bool getPose(const std::string &poseSel, yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp=NULL);
    bool blockNeckJoint(const std::string &joint, const double min, const double max);
    bool blockNeckJoint(const std::string &joint, const int j);
    bool getNeckJointRange(const std::string &joint, double *min, double *max);
    bool clearJoint(const std::string &joint);
    void eventHandling(yarp::os::Bottle &event);
    bool getInfoHelper(yarp::os::Bottle &info);

public:
    ClientGazeController();
    ClientGazeController(yarp::os::Searchable &config);

    bool open(yarp::os::Searchable &config);
    bool close();

    bool setTrackingMode(const bool f);
    bool getTrackingMode(bool *f);
    bool getFixationPoint(yarp::sig::Vector &fp, yarp::os::Stamp *stamp=NULL);
    bool getAngles(yarp::sig::Vector &ang, yarp::os::Stamp *stamp=NULL);
    bool lookAtFixationPoint(const yarp::sig::Vector &fp);
    bool lookAtAbsAngles(const yarp::sig::Vector &ang);
    bool lookAtRelAngles(const yarp::sig::Vector &ang);
    bool lookAtMonoPixel(const int camSel, const yarp::sig::Vector &px, const double z=1.0);
    bool lookAtMonoPixelWithVergence(const int camSel, const yarp::sig::Vector &px, const double ver);
    bool lookAtStereoPixels(const yarp::sig::Vector &pxl, const yarp::sig::Vector &pxr);
    bool getNeckTrajTime(double *t);
    bool getEyesTrajTime(double *t);
    bool getVORGain(double *gain);
    bool getOCRGain(double *gain);
    bool getSaccadesStatus(bool *f);
    bool getSaccadesInhibitionPeriod(double *period);
    bool getSaccadesActivationAngle(double *angle);
    bool getLeftEyePose(yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp=NULL);
    bool getRightEyePose(yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp=NULL);
    bool getHeadPose(yarp::sig::Vector &x, yarp::sig::Vector &o, yarp::os::Stamp *stamp=NULL);
    bool get2DPixel(const int camSel, const yarp::sig::Vector &x, yarp::sig::Vector &px);
    bool get3DPoint(const int camSel, const yarp::sig::Vector &px, const double z, yarp::sig::Vector &x);    
    bool get3DPointOnPlane(const int camSel, const yarp::sig::Vector &px, const yarp::sig::Vector &plane, yarp::sig::Vector &x);
    bool get3DPointFromAngles(const int mode, const yarp::sig::Vector &ang, yarp::sig::Vector &x);
    bool getAnglesFrom3DPoint(const yarp::sig::Vector &x, yarp::sig::Vector &ang);
    bool triangulate3DPoint(const yarp::sig::Vector &pxl, const yarp::sig::Vector &pxr, yarp::sig::Vector &x);
    bool getJointsDesired(yarp::sig::Vector &qdes);
    bool getJointsVelocities(yarp::sig::Vector &qdot);
    bool getStereoOptions(yarp::os::Bottle &options);
    bool setNeckTrajTime(const double t);
    bool setEyesTrajTime(const double t);
    bool setVORGain(const double gain);
    bool setOCRGain(const double gain);
    bool setSaccadesStatus(const bool f);
    bool setSaccadesInhibitionPeriod(const double period);
    bool setSaccadesActivationAngle(const double angle);
    bool setStereoOptions(const yarp::os::Bottle &options);
    bool bindNeckPitch(const double min, const double max);
    bool blockNeckPitch(const double val);
    bool blockNeckPitch();
    bool bindNeckRoll(const double min, const double max);
    bool blockNeckRoll(const double val);
    bool blockNeckRoll();
    bool bindNeckYaw(const double min, const double max);
    bool blockNeckYaw(const double val);
    bool blockNeckYaw();
    bool blockEyes(const double ver);
    bool blockEyes();
    bool getNeckPitchRange(double *min, double *max);
    bool getNeckRollRange(double *min, double *max);
    bool getNeckYawRange(double *min, double *max);
    bool getBlockedVergence(double *ver);
    bool clearNeckPitch();
    bool clearNeckRoll();
    bool clearNeckYaw();
    bool clearEyes();
    bool getNeckAngleUserTolerance(double *angle);
    bool setNeckAngleUserTolerance(const double angle);
    bool checkMotionDone(bool *f);
    bool waitMotionDone(const double period=0.1, const double timeout=0.0);
    bool checkSaccadeDone(bool *f);
    bool waitSaccadeDone(const double period=0.1, const double timeout=0.0);
    bool stopControl();
    bool storeContext(int *id);
    bool restoreContext(const int id);
    bool deleteContext(const int id);
    bool getInfo(yarp::os::Bottle &info);
    bool registerEvent(yarp::dev::GazeEvent &event);
    bool unregisterEvent(yarp::dev::GazeEvent &event);
    bool tweakSet(const yarp::os::Bottle &options);
    bool tweakGet(yarp::os::Bottle &options);

    virtual ~ClientGazeController();
};


#endif


