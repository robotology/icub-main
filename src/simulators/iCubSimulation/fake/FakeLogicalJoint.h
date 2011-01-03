// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_FAKELOGICALJOINT_INC
#define ICUBSIMULATION_FAKELOGICALJOINT_INC

#include "LogicalJoint.h"

#include <yarp/os/Time.h>

class FakeLogicalJoint : public LogicalJoint {
private:
    double angle;
    double vel;
    double last;
public:
    FakeLogicalJoint() { 
        last = yarp::os::Time::now(); 
        angle = vel = 0;
    }

    virtual double getAngle() { 
        update();
        return angle; 
    }
    
    virtual double getVelocity() { 
        update();
        return vel; 
    }
    
    virtual void setControlParameters(double vel, double acc) {
        this->vel = vel;
    }
    
    virtual void setPosition(double target) {
        update();
        vel = (target-angle)*5;
    }
    
    virtual void setVelocity(double target) {
        update();
        vel = target;
    }

    virtual bool isValid() {
        return true;
    }

    void update() {
        double now = yarp::os::Time::now();
        angle += vel*(now-last);
        last = now;
    }
};


#endif
