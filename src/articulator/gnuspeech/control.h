// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef ARTICULATOR_CONTROL
#define ARTICULATOR_CONTROL

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>

class ArticulatorMotor : public yarp::dev::DeviceDriver, 
                         public yarp::dev::IPositionControl,
                         public yarp::dev::IControlLimits,
                         public yarp::dev::IAxisInfo,
                         public yarp::dev::IEncoders {
private:
    int nj;
public:

    ArticulatorMotor() {
        nj = 0;
    }

    virtual bool open(yarp::os::Searchable& config);

    virtual bool close();

    virtual bool getAxes(int *ax) {
        *ax = nj;
        return true;
    }

    virtual bool setPositionMode() {
        return true;
    }

    virtual bool positionMove(int j, double ref);

    virtual bool positionMove(const double *refs);

    virtual bool relativeMove(int j, double delta) {
        return false;
    }

    virtual bool relativeMove(const double *deltas) {
        return false;
    }

    virtual bool checkMotionDone(int j, bool *flag) {
        for (int i=0; i<nj; i++) {
            flag[i] = true;
        }
        return false;
    }

    virtual bool checkMotionDone(bool *flag) {
        return false;
    }

    virtual bool setRefSpeed(int j, double sp) {
        return false;
    }

    virtual bool setRefSpeeds(const double *spds) {
        return false;
    }

    virtual bool setRefAcceleration(int j, double acc) {
        return false;
    }

    virtual bool setRefAccelerations(const double *accs) {
        return true;
    }

    virtual bool getRefSpeed(int j, double *ref) {
        *ref = 1;
        return false;
    }

    virtual bool getRefSpeeds(double *spds) {
        for (int i=0; i<nj; i++) {
            spds[i] = 1;
        }
        return false;
    }

    virtual bool getRefAcceleration(int j, double *acc) {
        *acc = 1;
        return false;
    }

    virtual bool getRefAccelerations(double *accs) {
        for (int i=0; i<nj; i++) {
            accs[i] = 1;
        }
        return false;
    }

    virtual bool stop(int j) {
        return false;
    }

    virtual bool stop() {
        return false;
    }

    virtual bool getAxisName(int axis, yarp::os::ConstString& name);

    virtual bool setLimits(int axis, double min, double max) {
        return false;
    }

    virtual bool getLimits(int axis, double *min, double *max);



    virtual bool resetEncoder(int j) {
        return false;
    }

    virtual bool resetEncoders() {
        return false;
    }

    virtual bool setEncoder(int j, double val) {
        return false;
    }

    virtual bool setEncoders(const double *vals) {
        return false;
    }
 
    virtual bool getEncoder(int j, double *v);

    virtual bool getEncoders(double *encs);
 
    virtual bool getEncoderSpeed(int j, double *sp) {
        return false;
    }

    virtual bool getEncoderSpeeds(double *spds) {
        return false;
    }
    
    virtual bool getEncoderAcceleration(int j, double *spds) {
        return false;
    }

    virtual bool getEncoderAccelerations(double *accs) {
        return false;
    }
};


#endif



