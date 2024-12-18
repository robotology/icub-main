/*
 * Copyright (C) 2006-2024 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "CouplingICubHandMk2.h"
#include <yarp/os/LogStream.h>
#include <cmath>
#include <array>
#include <numeric>


YARP_LOG_COMPONENT(COUPLINGICUBHANDMK2, "yarp.device.couplingICubHandMk2")


bool CouplingICubHandMk2::populateCouplingParameters() {
    yarp::sig::VectorOf<size_t> coupled_physical_joints, coupled_actuated_axes;
    coupled_physical_joints.resize(m_jointNames.size());
    coupled_actuated_axes.resize(m_COUPLING_actuatedAxesNames.size());
    std::iota(coupled_physical_joints.begin(), coupled_physical_joints.end(), 0);
    std::iota(coupled_actuated_axes.begin(), coupled_actuated_axes.end(), 0);
    std::vector<std::pair<double, double>> physical_joint_limits;

    physical_joint_limits.resize(m_jointNames.size());
    for (int i = 0; i< m_jointNames.size(); i++)
    {
        physical_joint_limits[i] = std::make_pair(m_LIMITS_jntPosMin[i], m_LIMITS_jntPosMax[i]);
    }
    initialise(coupled_physical_joints, coupled_actuated_axes, m_jointNames, m_COUPLING_actuatedAxesNames, physical_joint_limits);
    return true;
}

bool CouplingICubHandMk2::open(yarp::os::Searchable& config) {

    if(!parseParams(config)) {
        yCError(COUPLINGICUBHANDMK2) << "Error parsing parameters";
        return false;
    }

    if(!populateCouplingParameters()) {
        yCError(COUPLINGICUBHANDMK2) << "Error populating coupling parameters";
        return false;
    }

    yCDebug(COUPLINGICUBHANDMK2) << "Opening CouplingICubHandMk2"<<config.toString();
    return true;
}

bool CouplingICubHandMk2::convertFromPhysicalJointsToActuatedAxesPos(const yarp::sig::Vector& physJointsPos, yarp::sig::Vector& actAxesPos) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsPos.size() != nrOfPhysicalJoints || actAxesPos.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBHANDMK2) << "convertFromPhysicalJointsToActuatedAxesPos: input or output vectors have wrong size";
        return false;
    }

    // l/r_hand_finger
    actAxesPos[0] = (20.0 - physJointsPos[12])*3;
    // l/r_thumb_oppose
    actAxesPos[1] = physJointsPos[0];
    // l/r_thumb_proximal
    actAxesPos[2] = physJointsPos[1];
    // l/r_thumb_distal
    actAxesPos[3] = physJointsPos[2] + physJointsPos[3];
    // l/r_index_proximal
    actAxesPos[4] = physJointsPos[5];
    // l/r_index_distal
    actAxesPos[5] = physJointsPos[6] + physJointsPos[7];
    // l/r_middle_proximal
    actAxesPos[6] = physJointsPos[9];
    // l/r_middle_distal
    actAxesPos[7] = physJointsPos[10] + physJointsPos[11];
    // l/r_pinky
    actAxesPos[8] = physJointsPos[13] + physJointsPos[14] + physJointsPos[15];
    return true;
}

bool CouplingICubHandMk2::convertFromPhysicalJointsToActuatedAxesVel(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, yarp::sig::Vector& actAxesVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBHANDMK2) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }
    // l/r_hand_finger
    actAxesVel[0] = -physJointsVel[12]*3;
    // l/r_thumb_oppose
    actAxesVel[1] = physJointsVel[0];
    // l/r_thumb_proximal
    actAxesVel[2] = physJointsVel[1];
    // l/r_thumb_distal
    actAxesVel[3] = physJointsVel[2] + physJointsVel[3];
    // l/r_index_proximal
    actAxesVel[4] = physJointsVel[5];
    // l/r_index_distal
    actAxesVel[5] = physJointsVel[6] + physJointsVel[7];
    // l/r_middle_proximal
    actAxesVel[6] = physJointsVel[9];
    // l/r_middle_distal
    actAxesVel[7] = physJointsVel[10] + physJointsVel[11];
    // l/r_pinky
    actAxesVel[8] = physJointsVel[13] + physJointsVel[14] + physJointsVel[15];
    return true;
}


bool CouplingICubHandMk2::convertFromPhysicalJointsToActuatedAxesAcc(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, const yarp::sig::Vector& physJointsAcc, yarp::sig::Vector& actAxesAcc) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsAcc.size() != nrOfPhysicalJoints || actAxesAcc.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBHANDMK2) << "convertFromPhysicalJointsToActuatedAxesAcc: input or output vectors have wrong size";
        return false;
    }
    // l/r_hand_finger
    actAxesAcc[0] = -physJointsAcc[12]*3;
    // l/r_thumb_oppose
    actAxesAcc[1] = physJointsAcc[0];
    // l/r_thumb_proximal
    actAxesAcc[2] = physJointsAcc[1];
    // l/r_thumb_distal
    actAxesAcc[3] = physJointsAcc[2] + physJointsAcc[3];
    // l/r_index_proximal
    actAxesAcc[4] = physJointsAcc[5];
    // l/r_index_distal
    actAxesAcc[5] = physJointsAcc[6] + physJointsAcc[7];
    // l/r_middle_proximal
    actAxesAcc[6] = physJointsAcc[9];
    // l/r_middle_distal
    actAxesAcc[7] = physJointsAcc[10] + physJointsAcc[11];
    // l/r_pinky
    actAxesAcc[8] = physJointsAcc[13] + physJointsAcc[14] + physJointsAcc[15];
    return false;
}

bool CouplingICubHandMk2::convertFromPhysicalJointsToActuatedAxesTrq(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsTrq, yarp::sig::Vector& actAxesTrq) {
    return false;
}


bool CouplingICubHandMk2::convertFromActuatedAxesToPhysicalJointsPos(const yarp::sig::Vector& actAxesPos, yarp::sig::Vector& physJointsPos) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsPos.size() != nrOfPhysicalJoints || actAxesPos.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBHANDMK2) << "convertFromActuatedAxesToPhysicalJointsPos: input or output vectors have wrong size";
        return false;
    }

    // l/r_hand_thumb_0
    physJointsPos[0] = actAxesPos[1];
    // l/r_hand_thumb_1
    physJointsPos[1] = actAxesPos[2];
    // l/r_hand_thumb_2
    physJointsPos[2] = actAxesPos[3]/2;
    // l/r_hand_thumb_3
    physJointsPos[3] = actAxesPos[3]/2;
    // l/r_hand_index_0
    physJointsPos[4] = -(20.0 - actAxesPos[0]/3);
    // l/r_hand_index_1
    physJointsPos[5] = actAxesPos[4];
    // l/r_hand_index_2
    physJointsPos[6] = actAxesPos[5]/2;
    // l/r_hand_index_3
    physJointsPos[7] = actAxesPos[5]/2;
    // l/r_hand_middle_0
    physJointsPos[8] = 0.0;
    // l/r_hand_middle_1
    physJointsPos[9] = actAxesPos[6];
    // l/r_hand_middle_2
    physJointsPos[10] = actAxesPos[7]/2;
    // l/r_hand_middle_3
    physJointsPos[11] = actAxesPos[7]/2;
    // l/r_hand_ring_0
    physJointsPos[12] = 20.0 - actAxesPos[0]/3;
    // l/r_hand_ring_1
    physJointsPos[13] = actAxesPos[8]/3;
    // l/r_hand_ring_2
    physJointsPos[14] = actAxesPos[8]/3;
    // l/r_hand_ring_3
    physJointsPos[15] = actAxesPos[8]/3;
    // l/r_hand_little_0
    physJointsPos[16] = 20.0 - actAxesPos[0]/3;
    // l/r_hand_little_1
    physJointsPos[17] = actAxesPos[8]/3;
    // l/r_hand_little_2
    physJointsPos[18] = actAxesPos[8]/3;
    // l/r_hand_little_3
    physJointsPos[19] = actAxesPos[8]/3;
    
    return true;
}


bool CouplingICubHandMk2::convertFromActuatedAxesToPhysicalJointsVel(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, yarp::sig::Vector& physJointsVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || actAxesPos.size() != nrOfActuatedAxes || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBHANDMK2) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }

    // l/r_hand_thumb_0
    physJointsVel[0] = actAxesVel[1];
    // l/r_hand_thumb_1
    physJointsVel[1] = actAxesVel[2];
    // l/r_hand_thumb_2
    physJointsVel[2] = actAxesVel[3]/2;
    // l/r_hand_thumb_3
    physJointsVel[3] = actAxesVel[3]/2;
    // l/r_hand_index_0
    physJointsVel[4] = actAxesVel[0]/3;
    // l/r_hand_index_1
    physJointsVel[5] = actAxesVel[4];
    // l/r_hand_index_2
    physJointsVel[6] = actAxesVel[5]/2;
    // l/r_hand_index_3
    physJointsVel[7] = actAxesVel[5]/2;
    // l/r_hand_middle_0
    physJointsVel[8] = 0.0;
    // l/r_hand_middle_1
    physJointsVel[9] = actAxesVel[6];
    // l/r_hand_middle_2
    physJointsVel[10] = actAxesVel[7]/2;
    // l/r_hand_middle_3
    physJointsVel[11] = actAxesVel[7]/2;
    // l/r_hand_ring_0
    physJointsVel[12] = -actAxesVel[0]/3;
    // l/r_hand_ring_1
    physJointsVel[13] = actAxesVel[8]/3;
    // l/r_hand_ring_2
    physJointsVel[14] = actAxesVel[8]/3;
    // l/r_hand_ring_3
    physJointsVel[15] = actAxesVel[8]/3;
    // l/r_hand_little_0
    physJointsVel[16] = - actAxesVel[0]/3;
    // l/r_hand_little_1
    physJointsVel[17] = actAxesVel[8]/3;
    // l/r_hand_little_2
    physJointsVel[18] = actAxesVel[8]/3;
    // l/r_hand_little_3
    physJointsVel[19] = actAxesVel[8]/3;

    return true;
}

bool CouplingICubHandMk2::convertFromActuatedAxesToPhysicalJointsAcc(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, const yarp::sig::Vector& actAxesAcc, yarp::sig::Vector& physJointsAcc) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || actAxesPos.size() != nrOfActuatedAxes || physJointsAcc.size() != nrOfPhysicalJoints || actAxesAcc.size() != nrOfActuatedAxes) {
        yCError(COUPLINGICUBHANDMK2) << "convertFromPhysicalJointsToActuatedAxesAcc: input or output vectors have wrong size";
        return false;
    }

    // l/r_hand_thumb_0
    physJointsAcc[0] = actAxesAcc[1];
    // l/r_hand_thumb_1
    physJointsAcc[1] = actAxesAcc[2];
    // l/r_hand_thumb_2
    physJointsAcc[2] = actAxesAcc[3]/2;
    // l/r_hand_thumb_3
    physJointsAcc[3] = actAxesAcc[3]/2;
    // l/r_hand_index_0
    physJointsAcc[4] = actAxesAcc[0]/3;
    // l/r_hand_index_1
    physJointsAcc[5] = actAxesAcc[4];
    // l/r_hand_index_2
    physJointsAcc[6] = actAxesAcc[5]/2;
    // l/r_hand_index_3
    physJointsAcc[7] = actAxesAcc[5]/2;
    // l/r_hand_middle_0
    physJointsAcc[8] = 0.0;
    // l/r_hand_middle_1
    physJointsAcc[9] = actAxesAcc[6];
    // l/r_hand_middle_2
    physJointsAcc[10] = actAxesAcc[7]/2;
    // l/r_hand_middle_3
    physJointsAcc[11] = actAxesAcc[7]/2;
    // l/r_hand_ring_0
    physJointsAcc[12] = - actAxesAcc[0]/3;
    // l/r_hand_ring_1
    physJointsAcc[13] = actAxesAcc[8]/3;
    // l/r_hand_ring_2
    physJointsAcc[14] = actAxesAcc[8]/3;
    // l/r_hand_ring_3
    physJointsAcc[15] = actAxesAcc[8]/3;
    // l/r_hand_little_0
    physJointsAcc[16] = - actAxesAcc[0]/3;
    // l/r_hand_little_1
    physJointsAcc[17] = actAxesAcc[8]/3;
    // l/r_hand_little_2
    physJointsAcc[18] = actAxesAcc[8]/3;
    // l/r_hand_little_3
    physJointsAcc[19] = actAxesAcc[8]/3;
    
    return false;
}
bool CouplingICubHandMk2::convertFromActuatedAxesToPhysicalJointsTrq(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesTrq, yarp::sig::Vector& physJointsTrq) {
    return false;
}
