// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_SIMCONTROL_INC
#define ICUBSIMULATION_SIMCONTROL_INC

#include <ode/ode.h>

#include <string>

#include "pidfilter.h"

/////////////////////////////////////////////////////////////////////
// establish a mapping from the model to the external axes of control

/**
 *
 * Convenience class for mapping from physical ODE joints in the model to
 * control joints in the ICUB specification.
 *
 */
class SimControl {
public:
    /**
     * Constructor.
     */
    SimControl();

    /**
     * Destructor.
     */
    virtual ~SimControl();

    /**
     * Create an array of nested control units.  The array will be 
     * destroyed when this object is destroyed.
     */
    SimControl *nest(int len);

    /**
     * Access a nested control unit.
     */
    SimControl *at(int index);

    /**
     * Initialize a regular control unit.
     */
    void init(const char *unit, const char *type, int index, int sign);

    /**
     * Initialize a differential pair of control units.
     */
    void init(SimControl& left, SimControl& right, SimControl& peer, int sgn);

    /**
     * Get the angle of an associated joint, in ICUB units and sign.
     */
    double getAngle() {
        return getAngleRaw()*sign;
    }

    /**
     * Get the angular velocity of an associated joint, in ICUB units and sign.
     */
    double getVelocity() {
        return getVelocityRaw()*sign;
    }

    /**
     * Get the angle of an associated joint in unconverted units and sign.
     */
    double getAngleRaw();

    /**
     * Get the velocity of an associated joint in unconverted units and sign.
     */
    double getVelocityRaw();

    /**
     * Get the current target velocity.
     */
    double getSpeedSetpoint() {
        return speedSetpoint;
    }

    /**
     * Set velocity and acceleration control parameters.
     */
    void setControlParameters(double vel, double acc);

    /**
     * Drive towards an angle setpoint (in ICUB units/sign).
     */
    void setPosition(double target);

    /**
     * Set velocity of joint (in ICUB units/sign).
     */
    void setVelocity(double target);

    /**
     * Set raw velocity of joint (in ODE units/sign).
     */
    void setVelocityRaw(double target);

private:
    int number;
	std::string unit;
    dJointID *joint;
    dReal *speed;
    double speedSetpoint;
    bool hinged;
    int universal;
    int sign;
    SimControl *sub;
    int subLength;
    bool active;
    double vel;
    double acc;
    PidFilter filter;

    SimControl *left, *right, *peer;
    int verge;
};


#endif
