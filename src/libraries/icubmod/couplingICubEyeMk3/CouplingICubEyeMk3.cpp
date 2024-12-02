/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "CouplingICubEyeMk3.h"
#include <yarp/os/LogStream.h>
#include <cmath>
#include <array>
#include <numeric>


YARP_LOG_COMPONENT(COUPLINGICUBEYEMK3, "yarp.device.couplingICubEyeMk3")


bool CouplingICubEyeMk3::populateCouplingParameters() {
    yarp::sig::VectorOf<size_t> coupled_physical_joints{1, 2};
    yarp::sig::VectorOf<size_t> coupled_actuated_axes{1, 2};
    std::vector<std::pair<double, double>> physical_joint_limits;

    physical_joint_limits.resize(m_jointNames.size());
    for (int i = 0; i< m_jointNames.size(); i++)
    {
        physical_joint_limits[i] = std::make_pair(m_LIMITS_jntPosMin[i], m_LIMITS_jntPosMax[i]);
    }
    initialise(coupled_physical_joints, coupled_actuated_axes, m_jointNames, m_COUPLING_actuatedAxesNames, physical_joint_limits);
    return true;
}

bool CouplingICubEyeMk3::open(yarp::os::Searchable& config) {

    if(!parseParams(config)) {
        yCError(COUPLINGICUBEYEMK3) << "Error parsing parameters";
        return false;
    }

    yCDebug(COUPLINGICUBEYEMK3) << "Opening couplingICubEyeMk3" << config.toString();
    return true;
}

bool CouplingICubEyeMk3::convertFromPhysicalJointsToActuatedAxesPos(const yarp::sig::Vector& physJointsPos, yarp::sig::Vector& actAxesPos) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsPos.size() != nrOfPhysicalJoints || actAxesPos.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYEMK3) << "convertFromPhysicalJointsToActuatedAxesPos: input or output vectors have wrong size";
        return false;
    }

    //version
    actAxesPos[1] = (physJointsPos[1] + physJointsPos[2])/2;
    //vergence
    actAxesPos[2] = (physJointsPos[1] - physJointsPos[2]);

    return true;
}

bool CouplingICubEyeMk3::convertFromPhysicalJointsToActuatedAxesVel(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, yarp::sig::Vector& actAxesVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYEMK3) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }

    //version
    actAxesPos[1] = (physJointsPos[1] + physJointsPos[2])/2;
    //vergence
    actAxesPos[2] = (physJointsPos[1] - physJointsPos[2]);

    return true;
}

bool CouplingICubEyeMk3::convertFromPhysicalJointsToActuatedAxesAcc(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, const yarp::sig::Vector& physJointsAcc, yarp::sig::Vector& actAxesAcc) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsAcc.size() != nrOfPhysicalJoints || actAxesAcc.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYEMK3) << "convertFromPhysicalJointsToActuatedAxesAcc: input or output vectors have wrong size";
        return false;
    }

    //version
    actAxesPos[1] = (physJointsPos[1] + physJointsPos[2])/2;
    //vergence
    actAxesPos[2] = (physJointsPos[1] - physJointsPos[2]);

    return true;
}

bool CouplingICubEyeMk3::convertFromPhysicalJointsToActuatedAxesTrq(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsTrq, yarp::sig::Vector& actAxesTrq) {
    return false;
}

bool CouplingICubEyeMk3::convertFromActuatedAxesToPhysicalJointsPos(const yarp::sig::Vector& actAxesPos, yarp::sig::Vector& physJointsPos) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsPos.size() != nrOfPhysicalJoints || actAxesPos.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYEMK3) << "convertFromActuatedAxesToPhysicalJointsPos: input or output vectors have wrong size";
        return false;
    }

    // l_eye_pan_joint
    physJointsPos[1] = actAxesPos[1] + actAxesPos[2]/2;
    // r_eye_pan_joint
    physJointsPos[2] = actAxesPos[1] - actAxesPos[2]/2;

    return true;
}

bool CouplingICubEyeMk3::convertFromActuatedAxesToPhysicalJointsVel(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, yarp::sig::Vector& physJointsVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || actAxesPos.size() != nrOfActuatedAxes || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYEMK3) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }

    // l_eye_pan_joint
    physJointsPos[1] = actAxesPos[1] + actAxesPos[2]/2;
    // r_eye_pan_joint
    physJointsPos[2] = actAxesPos[1] - actAxesPos[2]/2;
    
    return true;
}

bool CouplingICubEyeMk3::convertFromActuatedAxesToPhysicalJointsAcc(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, const yarp::sig::Vector& actAxesAcc, yarp::sig::Vector& physJointsAcc) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || actAxesPos.size() != nrOfActuatedAxes || physJointsAcc.size() != nrOfPhysicalJoints || actAxesAcc.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYEMK3) << "convertFromPhysicalJointsToActuatedAxesAcc: input or output vectors have wrong size";
        return false;
    }

    // l_eye_pan_joint
    physJointsPos[1] = actAxesPos[1] + actAxesPos[2]/2;
    // r_eye_pan_joint
    physJointsPos[2] = actAxesPos[1] - actAxesPos[2]/2;

    return true;
}

bool CouplingICubEyeMk3::convertFromActuatedAxesToPhysicalJointsTrq(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesTrq, yarp::sig::Vector& physJointsTrq) {
    return false;
}