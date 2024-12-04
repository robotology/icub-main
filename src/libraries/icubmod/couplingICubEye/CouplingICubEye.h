/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef COUPLINGICUBEYE_H
#define COUPLINGICUBEYE_H

#include <yarp/os/LogComponent.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ImplementJointCoupling.h>
#include "CouplingICubEye_ParamsParser.h"

#include <unordered_map>
#include <vector>
#include <string>


YARP_DECLARE_LOG_COMPONENT(COUPLINGICUBEYE)

/**
 * Coupling law from https://icub-tech-iit.github.io/documentation/icub_kinematics/icub-vergence-version/icub-vergence-version/
 */
/** TBD
 */
class CouplingICubEye :  public yarp::dev::DeviceDriver,
                            public yarp::dev::ImplementJointCoupling,
                            public CouplingICubEye_ParamsParser {
public:
    CouplingICubEye() = default;
    virtual ~CouplingICubEye() override = default;
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

    bool populateCouplingParameters();
};

#endif // COUPLINGICUBEYE_H