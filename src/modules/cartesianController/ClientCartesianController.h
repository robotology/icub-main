// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Developed by Ugo Pattacini

#ifndef __CLIENTCARTESIANCONTROLLER_H__
#define __CLIENTCARTESIANCONTROLLER_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>


class ClientCartesianController : public yarp::dev::DeviceDriver,
                                  public yarp::dev::ICartesianControl
{
protected:
    bool closed;
    bool gotPose;

    yarp::sig::Vector pose;

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
    virtual bool goToPose(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t=0.0);
    virtual bool goToPosition(const yarp::sig::Vector &xd, const double t=0.0);
    virtual bool goToPoseSync(const yarp::sig::Vector &xd, const yarp::sig::Vector &od, const double t=0.0);
    virtual bool goToPositionSync(const yarp::sig::Vector &xd, const double t=0.0);
    virtual bool getDOF(yarp::sig::Vector &curDof);
    virtual bool setDOF(const yarp::sig::Vector &newDof, yarp::sig::Vector &curDof);
    virtual bool getLimits(int axis, double *min, double *max);
    virtual bool setLimits(int axis, const double min, const double max);
    virtual bool getTrajTime(double *t);
    virtual bool setTrajTime(const double t);
    virtual bool checkMotionDone(bool *f);
    virtual bool stopControl();

    virtual ~ClientCartesianController();
};


#endif


