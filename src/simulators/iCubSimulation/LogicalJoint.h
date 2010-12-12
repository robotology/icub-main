// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef ICUBSIMULATION_LOGICALJOINT_INC
#define ICUBSIMULATION_LOGICALJOINT_INC


/////////////////////////////////////////////////////////////////////
// establish a mapping from the model to the external axes of control

/**
 *
 * Abstract class for mapping from "physical" (simulated) joints in the model 
 * to control joints in the ICUB specification.
 *
 */
class LogicalJoint {
public:
    /**
     * Destructor.
     */
    virtual ~LogicalJoint() {}

    /**
     * Get the angle of an associated joint, in ICUB units and sign.
     */
    virtual double getAngle() = 0;

    /**
     * Get the angular velocity of an associated joint, in ICUB units and sign.
     */
    virtual double getVelocity() = 0;

    /**
     * Set velocity and acceleration control parameters.
     */
    virtual void setControlParameters(double vel, double acc) = 0;

    /**
     * Drive towards an angle setpoint (in ICUB units/sign).
     */
    virtual void setPosition(double target) = 0;

    /**
     * Set velocity of joint (in ICUB units/sign).
     */
    virtual void setVelocity(double target) = 0;
};


#endif
