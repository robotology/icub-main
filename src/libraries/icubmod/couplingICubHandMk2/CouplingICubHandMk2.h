/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef COUPLINGXCUBHANDMK5_H
#define COUPLINGXCUBHANDMK5_H

#include <yarp/os/LogComponent.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ImplementJointCoupling.h>
#include "CouplingICubHandMk2_ParamsParser.h"

#include <unordered_map>
#include <vector>
#include <string>


YARP_DECLARE_LOG_COMPONENT(COUPLINGXCUBHANDMK5)

/**
 * Parameters from https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling
 */
struct FingerParameters
{
    double L0x;
    double L0y;
    double q2bias;
    double q1off;
    double k;
    double d;
    double l;
    double b;
};

/** TBD
 */
class CouplingICubHandMk2 : public yarp::dev::DeviceDriver,
                            public yarp::dev::ImplementJointCoupling,
                            public CouplingICubHandMk2_ParamsParser {
public:
    CouplingICubHandMk2() = default;
    virtual ~CouplingICubHandMk2() override = default;
    bool convertFromPhysicalJointsToActuatedAxesPos(const yarp::sig::Vector& physJointsPos, yarp::sig::Vector& actAxesPos) override;
    bool convertFromPhysicalJointsToActuatedAxesVel(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, yarp::sig::Vector& actAxesVel) override;
    bool convertFromPhysicalJointsToActuatedAxesAcc(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, const yarp::sig::Vector& physJointsAcc, yarp::sig::Vector& actAxesAcc) override;
    bool convertFromPhysicalJointsToActuatedAxesTrq(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsTrq, yarp::sig::Vector& actAxesTrq) override;
    bool convertFromActuatedAxesToPhysicalJointsPos(const yarp::sig::Vector& actAxesPos, yarp::sig::Vector& physJointsPos) override;
    bool convertFromActuatedAxesToPhysicalJointsVel(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, yarp::sig::Vector& physJointsVel) override;
    bool convertFromActuatedAxesToPhysicalJointsAcc(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, const yarp::sig::Vector& actAxesAcc, yarp::sig::Vector& physJointsAcc) override;
    bool convertFromActuatedAxesToPhysicalJointsTrq(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesTrq, yarp::sig::Vector& physJointsTrq) override;

    // //DeviceDriver
    /**
        * Configure with a set of options.
        * @param config The options to use
        * @return true iff the object could be configured.
        */
    bool open(yarp::os::Searchable& config) override;
private:

    std::unordered_map<std::string, FingerParameters> mFingerParameters;


    /*
     * This method implements the law q2 = q2(q1) from
     * https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling,
     * i.e., the absolute angle of the distal joint q2 with respect to the palm.
     *
     * The inputs q1 and the return value of the function are in degrees.
     */
    double evaluateCoupledJoint(const double& q1, const std::string& finger_name);

    /*
     * This method implements the law \frac{\partial{q2}}{\partial{q1}} from
     * https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling,
     * i.e., the jacobian of the absolute angle of the distal joint q2 measured from the palm,
     * with respect to the proximal joint q1.
     *
     * The input q1 is in degrees.
     */
    double evaluateCoupledJointJacobian(const double& q1, const std::string& finger_name);

    bool populateFingerParameters();
    bool populateCouplingParameters();
};

#endif // COUPLINGXCUBHANDMK5_H