/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "CouplingICubEye.h"
#include <yarp/os/LogStream.h>
#include <cmath>
#include <array>
#include <numeric>


YARP_LOG_COMPONENT(COUPLINGICUBEYE, "yarp.device.couplingICubEye")


bool CouplingICubEye::populateCouplingParameters() {
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

bool CouplingICubEye::open(yarp::os::Searchable& config) {

    if(!parseParams(config)) {
        yCError(COUPLINGICUBEYE) << "Error parsing parameters";
        return false;
    }

    if(!populateCouplingParameters) {
        yCError(COUPLINGICUBEYE) << "Error populating coupling parameters";
        return false;
    }

    yCDebug(COUPLINGICUBEYE) << "Opening couplingICubEye" << config.toString();
    return true;
}

bool CouplingICubEye::convertFromPhysicalJointsToActuatedAxesPos(const yarp::sig::Vector& physJointsPos, yarp::sig::Vector& actAxesPos) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsPos.size() != nrOfPhysicalJoints || actAxesPos.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYE) << "convertFromPhysicalJointsToActuatedAxesPos: input or output vectors have wrong size";
        return false;
    }

    // eyes_tilt
    actAxesPos[0] = physJointsPos[0];
    // eyes_version
    actAxesPos[1] = (physJointsPos[1] + physJointsPos[2])/2;
    // eyes_vergence
    actAxesPos[2] = (physJointsPos[1] - physJointsPos[2]);

    return true;
}

bool CouplingICubEye::convertFromPhysicalJointsToActuatedAxesVel(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, yarp::sig::Vector& actAxesVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYE) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }
    // eyes_tilt
    actAxesVel[0] = physJointsVel[0];
    // eyes_version
    actAxesVel[1] = (physJointsVel[1] + physJointsVel[2])/2;
    // eyes_vergence
    actAxesVel[2] = (physJointsVel[1] - physJointsVel[2]);

    return true;
}

bool CouplingICubEye::convertFromPhysicalJointsToActuatedAxesAcc(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, const yarp::sig::Vector& physJointsAcc, yarp::sig::Vector& actAxesAcc) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsAcc.size() != nrOfPhysicalJoints || actAxesAcc.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYE) << "convertFromPhysicalJointsToActuatedAxesAcc: input or output vectors have wrong size";
        return false;
    }

    // eyes_tilt
    actAxesAcc[0] = physJointsAcc[0];
    // eyes_version
    actAxesAcc[1] = (physJointsAcc[1] + physJointsAcc[2])/2;
    // eyes_vergence
    actAxesAcc[2] = (physJointsAcc[1] - physJointsAcc[2]);

    return true;
}

bool CouplingICubEye::convertFromPhysicalJointsToActuatedAxesTrq(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsTrq, yarp::sig::Vector& actAxesTrq) {
    return false;
}

bool CouplingICubEye::convertFromActuatedAxesToPhysicalJointsPos(const yarp::sig::Vector& actAxesPos, yarp::sig::Vector& physJointsPos) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsPos.size() != nrOfPhysicalJoints || actAxesPos.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYE) << "convertFromActuatedAxesToPhysicalJointsPos: input or output vectors have wrong size";
        return false;
    }

    // eyes_tilt
    physJointsPos[0] = actAxesPos[0];
    // l_eye_pan_joint
    physJointsPos[1] = actAxesPos[1] + actAxesPos[2]/2;
    // r_eye_pan_joint
    physJointsPos[2] = actAxesPos[1] - actAxesPos[2]/2;

    return true;
}

bool CouplingICubEye::convertFromActuatedAxesToPhysicalJointsVel(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, yarp::sig::Vector& physJointsVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || actAxesPos.size() != nrOfActuatedAxes || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYE) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }

    // eyes_tilt
    physJointsVel[0] = actAxesVel[0];
    // l_eye_pan_joint
    physJointsVel[1] = actAxesVel[1] + actAxesVel[2]/2;
    // r_eye_pan_joint
    physJointsVel[2] = actAxesVel[1] - actAxesVel[2]/2;
    
    return true;
}

bool CouplingICubEye::convertFromActuatedAxesToPhysicalJointsAcc(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, const yarp::sig::Vector& actAxesAcc, yarp::sig::Vector& physJointsAcc) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || actAxesPos.size() != nrOfActuatedAxes || physJointsAcc.size() != nrOfPhysicalJoints || actAxesAcc.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBEYE) << "convertFromPhysicalJointsToActuatedAxesAcc: input or output vectors have wrong size";
        return false;
    }

    // eyes_tilt
    physJointsAcc[0] = actAxesAcc[0];
    // l_eye_pan_joint
    physJointsAcc[1] = actAxesAcc[1] + actAxesAcc[2]/2;
    // r_eye_pan_joint
    physJointsAcc[2] = actAxesAcc[1] - actAxesAcc[2]/2;

    return true;
}

bool CouplingICubEye::convertFromActuatedAxesToPhysicalJointsTrq(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesTrq, yarp::sig::Vector& physJointsTrq) {
    return false;
}