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
 * @note Please read carefully the \ref icub_cartesian_interface
 *       "Cartesian Interface" documentation.
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
#include <yarp/sig/Vector.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/dev/CartesianControl.h>

#include <iCub/iKin/iKinHlp.h>

#include <set>


class ClientCartesianController : public    yarp::dev::DeviceDriver,
                                  public    yarp::dev::IPreciselyTimed,
                                  public    yarp::dev::ICartesianControl,
                                  protected iCub::iKin::CartesianHelper
{
protected:
    bool connected;
    bool closed;

    double timeout;
    double lastPoseMsgArrivalTime;

    yarp::sig::Vector pose;
    yarp::os::Stamp   rxInfo;

    yarp::os::BufferedPort<yarp::sig::Vector> portState;
    yarp::os::Port                            portCmd;
    yarp::os::Port                            portRpc;

    std::set<int> contextIdList;

    virtual bool deleteContexts();

public:
    ClientCartesianController();
    ClientCartesianController(yarp::os::Searchable &config);

    bool open(yarp::os::Searchable &config);
    bool close();

    bool setTrackingMode(const bool f);
    bool getTrackingMode(bool *f);
    bool getPose(yarp::sig::Vector &x, yarp::sig::Vector &o);
    bool getPose(const int axis, yarp::sig::Vector &x, yarp::sig::Vector &o);
    bool goToPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t=0.0);
    bool goToPosition(const yarp::sig::Vector &xd, const double t=0.0);
    bool goToPoseSync(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t=0.0);
    bool goToPositionSync(const yarp::sig::Vector &xd, const double t=0.0);
    bool getDesired(yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,yarp::sig::Vector &qdhat);
    bool askForPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, yarp::sig::Vector &xdhat,
                    yarp::sig::Vector &odhat, yarp::sig::Vector &qdhat);
    bool askForPose(const yarp::sig::Vector &q0, const yarp::sig::Vector &xd, const yarp::sig::Vector &od,
                    yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat, yarp::sig::Vector &qdhat);
    bool askForPosition(const yarp::sig::Vector &xd, yarp::sig::Vector &xdhat, yarp::sig::Vector &odhat,
                        yarp::sig::Vector &qdhat);
    bool askForPosition(const yarp::sig::Vector &q0, const yarp::sig::Vector &xd, yarp::sig::Vector &xdhat,
                        yarp::sig::Vector &odhat, yarp::sig::Vector &qdhat);
    bool getDOF(yarp::sig::Vector &curDof);
    bool setDOF(const yarp::sig::Vector &newDof, yarp::sig::Vector &curDof);
    bool getRestPos(yarp::sig::Vector &curRestPos);
    bool setRestPos(const yarp::sig::Vector &newRestPos, yarp::sig::Vector &curRestPos);
    bool getRestWeights(yarp::sig::Vector &curRestWeights);
    bool setRestWeights(const yarp::sig::Vector &newRestWeights, yarp::sig::Vector &curRestWeights);
    bool getLimits(const int axis, double *min, double *max);
    bool setLimits(const int axis, const double min, const double max);
    bool getTrajTime(double *t);
    bool setTrajTime(const double t);
    bool getInTargetTol(double *tol);
    bool setInTargetTol(const double tol);
    bool getJointsVelocities(yarp::sig::Vector &qdot);
    bool getTaskVelocities(yarp::sig::Vector &xdot, yarp::sig::Vector &odot);
    bool setTaskVelocities(const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot);
    bool checkMotionDone(bool *f);
    bool waitMotionDone(const double period=0.1, const double timeout=0.0);
    bool stopControl();
    bool storeContext(int *id);
    bool restoreContext(const int id);

    yarp::os::Stamp getLastInputStamp();

    virtual ~ClientCartesianController();
};


#endif


