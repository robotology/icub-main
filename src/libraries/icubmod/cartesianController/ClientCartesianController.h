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
 * \defgroup clientcartesiancontroller clientcartesiancontroller
 * @ingroup icub_hardware_modules 
 *  
 * Implements the client part of the <a 
 * href="http://eris.liralab.it/yarpdoc/dd/de6/classyarp_1_1dev_1_1ICartesianControl.html">Cartesian 
 * Interface</a>.
 *
 * Copyright (C) 2010 RobotCub Consortium.
 *
 * Author: Ugo Pattacini
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 * This file can be edited at 
 * src/modules/cartesianController/ClientCartesianController.h 
 */

#ifndef __CLIENTCARTESIANCONTROLLER_H__
#define __CLIENTCARTESIANCONTROLLER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/iKin/iKinHlp.h>


class ClientCartesianController : public yarp::dev::DeviceDriver,
                                  public yarp::dev::IPreciselyTimed,
                                  public yarp::dev::ICartesianControl,
                                  public iCub::iKin::CartesianHelper
{
protected:
    bool connected;
    bool closed;

    double timeout;
    double lastPoseMsgArrivalTime;

    yarp::sig::Vector   pose;
    yarp::os::Stamp     rxInfo;
    yarp::os::Semaphore mutex;

    yarp::os::BufferedPort<yarp::os::Bottle>  *portCmd;
    yarp::os::BufferedPort<yarp::sig::Vector> *portState;
    yarp::os::Port                            *portRpc;

public:
    ClientCartesianController();
    ClientCartesianController(yarp::os::Searchable &config);

    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();

    virtual bool setTrackingMode(const bool f);
    virtual bool getTrackingMode(bool *f);
    virtual bool getPose(yarp::sig::Vector &x, yarp::sig::Vector &o);
    virtual bool getPose(const int axis, yarp::sig::Vector &x, yarp::sig::Vector &o);
    virtual bool goToPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t=0.0);
    virtual bool goToPosition(const yarp::sig::Vector &xd, const double t=0.0);
    virtual bool goToPoseSync(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t=0.0);
    virtual bool goToPositionSync(const yarp::sig::Vector &xd, const double t=0.0);
    virtual bool getDesired(yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,yarp::sig::Vector &qdhat);
    virtual bool askForPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, yarp::sig::Vector &xdhat,
                            yarp::sig::Vector &odhat, yarp::sig::Vector &qdhat);
    virtual bool askForPose(const yarp::sig::Vector &q0, const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                            yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat, yarp::sig::Vector &qdhat);
    virtual bool askForPosition(const yarp::sig::Vector &xd, yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                                yarp::sig::Vector &qdhat);
    virtual bool askForPosition(const yarp::sig::Vector &q0, const yarp::sig::Vector &xd, yarp::sig::Vector &xdhat,
                                yarp::sig::Vector &odhat, yarp::sig::Vector &qdhat);
    virtual bool getDOF(yarp::sig::Vector &curDof);
    virtual bool setDOF(const yarp::sig::Vector &newDof, yarp::sig::Vector &curDof);
    virtual bool getRestPos(yarp::sig::Vector &curRestPos);
    virtual bool setRestPos(const yarp::sig::Vector &newRestPos, yarp::sig::Vector &curRestPos);
    virtual bool getRestWeights(yarp::sig::Vector &curRestWeights);
    virtual bool setRestWeights(const yarp::sig::Vector &newRestWeights, yarp::sig::Vector &curRestWeights);
    virtual bool getLimits(const int axis, double *min, double *max);
    virtual bool setLimits(const int axis, const double min, const double max);
    virtual bool getTrajTime(double *t);
    virtual bool setTrajTime(const double t);
    virtual bool getInTargetTol(double *tol);
    virtual bool setInTargetTol(const double tol);
    virtual bool getJointsVelocities(yarp::sig::Vector &qdot);
    virtual bool getTaskVelocities(yarp::sig::Vector &xdot, yarp::sig::Vector &odot);
    virtual bool checkMotionDone(bool *f);
    virtual bool stopControl();

    virtual yarp::os::Stamp getLastInputStamp();

    virtual ~ClientCartesianController();
};


#endif


