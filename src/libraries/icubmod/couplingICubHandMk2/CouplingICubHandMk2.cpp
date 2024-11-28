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


YARP_LOG_COMPONENT(COUPLINGICUBHANDMK2, "yarp.device.couplingICubHandMk2")


double CouplingICubHandMk2::evaluateCoupledJoint(const double& q1, const std::string& finger_name)
{
    /**
     * Coupling law taken from from https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling
     */
    auto params = mFingerParameters.at(finger_name);
    double q1_rad = q1 * M_PI / 180.0;
    double q1off_rad = params.q1off * M_PI / 180.0;
    double q2bias_rad = params.q2bias * M_PI / 180.0;

    double P1x_q1 = params.d * cos(q1_rad + q1off_rad);
    double P1y_q1 = params.d * sin(q1_rad + q1off_rad);

    double h_sq = std::pow(P1x_q1 - params.L0x, 2) + std::pow(P1y_q1 - params.L0y, 2);
    double h = std::sqrt(h_sq);
    double l_sq = std::pow(params.l, 2);
    double k_sq = std::pow(params.k, 2);

    double q2 = atan2(P1y_q1 - params.L0y, P1x_q1 - params.L0x) + \
        acos((h_sq + l_sq - k_sq) / (2.0 * params.l * h)) + \
        -q2bias_rad - M_PI;

    // The value of q1 is subtracted from the result as the formula provides
    // the absolute angle of the coupled distal joint with respect to the palm.
    return q2 * 180.0 / M_PI - q1;
}


double CouplingICubHandMk2::evaluateCoupledJointJacobian(const double& q1, const std::string& finger_name)
{
    /**
     * Coupling law jacobian taken from from https://icub-tech-iit.github.io/documentation/hands/hands_mk5_coupling
     */
    auto params = mFingerParameters.at(finger_name);
    double q1_rad = q1 * M_PI / 180.0;
    double q1off_rad = params.q1off * M_PI / 180.0;

    double P1x_q1 = params.d * cos(q1_rad + q1off_rad);
    double P1y_q1 = params.d * sin(q1_rad + q1off_rad);

    double h_sq = std::pow(P1x_q1 - params.L0x, 2) + std::pow(P1y_q1 - params.L0y, 2);
    double h = std::sqrt(h_sq);
    double l_sq = std::pow(params.l, 2);
    double k_sq = std::pow(params.k, 2);

    double dq2_dq1_11 = 1;
    double dq2_dq1_21 = 2 - (std::pow(params.d, 2) - std::pow(params.b, 2)) / (std::pow(params.d, 2) - (params.L0x * P1x_q1 + params.L0y * P1y_q1));
    double dq2_dq1_12 = (params.L0x * P1y_q1 - params.L0y * P1x_q1) * (l_sq - k_sq - h_sq);
    double dq2_dq1_22 = 2 * params.l * h * h_sq * std::sqrt(1 - std::pow((l_sq - k_sq + h_sq) / (2 * params.l * h), 2));
    double dq2_dq1 = dq2_dq1_11 / dq2_dq1_21 + dq2_dq1_12 / dq2_dq1_22;

    // The value of 1 is subtracted from the result as evaluateCoupledJointJacobian provides
    // the jacobian of the absolute angle of the coupled distal joint.
    return dq2_dq1 - 1;
}


bool CouplingICubHandMk2::populateFingerParameters()
{
    constexpr int nFingers = 5;
    // All the +1 is because the first element of the bottle is the name of the group
    if(m_COUPLING_PARAMS_L0x.size()!=nFingers   || m_COUPLING_PARAMS_L0y.size()!=nFingers || m_COUPLING_PARAMS_q2bias.size()!=nFingers ||
       m_COUPLING_PARAMS_q1off.size()!=nFingers || m_COUPLING_PARAMS_k.size()!=nFingers   || m_COUPLING_PARAMS_d.size()!=nFingers      ||
       m_COUPLING_PARAMS_l.size()!=nFingers     || m_COUPLING_PARAMS_b.size()!=nFingers )
    {
        yCError(COUPLINGXCUBHANDMK5)<<"Invalid hand parameters, check your configuration file";
        return false;
    }


    const std::array<std::string,5> names = {"thumb", "index", "middle", "ring", "pinky"};
    for (std::size_t i = 0; i < names.size(); i++)
    {
        mFingerParameters.insert({names.at(i), {m_COUPLING_PARAMS_L0x[i], m_COUPLING_PARAMS_L0y[i], m_COUPLING_PARAMS_q2bias[i],
                                  m_COUPLING_PARAMS_q1off[i], m_COUPLING_PARAMS_k[i], m_COUPLING_PARAMS_d[i],
                                  m_COUPLING_PARAMS_l[i], m_COUPLING_PARAMS_b[i]}});
    }

    return true;
}

bool CouplingICubHandMk2::populateCouplingParameters() {
    yarp::sig::VectorOf<size_t> coupled_physical_joints{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    yarp::sig::VectorOf<size_t> coupled_actuated_axes{0, 1, 2, 3, 4, 5};
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
        yCError(COUPLINGXCUBHANDMK5) << "Error parsing parameters";
        return false;
    }

    yCDebug(COUPLINGXCUBHANDMK5) << "Opening CouplingICubHandMk2"<<config.toString();

    bool ok = populateCouplingParameters();
    if (!ok) {
        yCError(COUPLINGXCUBHANDMK5) << "Error parsing coupling parameters";
        return false;
    }

    ok = ok && populateFingerParameters();
    if (!ok) {
        yCError(COUPLINGXCUBHANDMK5) << "Error parsing finger parameters";
        return false;
    }

    return ok;
}

bool CouplingICubHandMk2::convertFromPhysicalJointsToActuatedAxesPos(const yarp::sig::Vector& physJointsPos, yarp::sig::Vector& actAxesPos) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsPos.size() != nrOfPhysicalJoints || actAxesPos.size() != nrOfActuatedAxes) {
        yCError(COUPLINGXCUBHANDMK5) << "convertFromPhysicalJointsToActuatedAxesPos: input or output vectors have wrong size";
        return false;
    }

    /* thumb_add <-- thumb_add */
    actAxesPos[0] = physJointsPos[0];
    /* thumb_oc <-- thumb_prox */
    actAxesPos[1] = physJointsPos[1];
    /* index_add <-- index_add */
    actAxesPos[2] = physJointsPos[3];
    /* index_oc <-- index_prox */
    actAxesPos[3] = physJointsPos[4];
    /* middle_oc <-- middle_prox */
    actAxesPos[4] = physJointsPos[6];
    /**
     * ring_pinky_oc <-- pinkie_prox
     * as, on the real robot, the coupled group composed of ring_prox, ring_dist, pinkie_prox and pinkie_dist
     * is controlled using the encoder on the pinkie_prox as feedback
     */
    actAxesPos[5] = physJointsPos[10];
    return true;
}

bool CouplingICubHandMk2::convertFromPhysicalJointsToActuatedAxesVel(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, yarp::sig::Vector& actAxesVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGXCUBHANDMK5) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }
    /* thumb_add <-- thumb_add */
    actAxesVel[0] = physJointsVel[0];
    /* thumb_oc <-- thumb_prox */
    actAxesVel[1] = physJointsVel[1];
    /* index_add <-- index_add */
    actAxesVel[2] = physJointsVel[3];
    /* index_oc <-- index_prox */
    actAxesVel[3] = physJointsVel[4];
    /* middle_oc <-- middle_prox */
    actAxesVel[4] = physJointsVel[6];
    /**
     * ring_pinky_oc <-- pinkie_prox
     * as, on the real robot, the coupled group composed of ring_prox, ring_dist, pinkie_prox and pinkie_dist
     * is controlled using the encoder on the pinkie_prox as feedback
     */
    actAxesVel[5] = physJointsVel[10];
    return true;
}


bool CouplingICubHandMk2::convertFromPhysicalJointsToActuatedAxesAcc(const yarp::sig::Vector& physJointsPos, const yarp::sig::Vector& physJointsVel, const yarp::sig::Vector& physJointsAcc, yarp::sig::Vector& actAxesAcc) {
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
        yCError(COUPLINGXCUBHANDMK5) << "convertFromActuatedAxesToPhysicalJointsPos: input or output vectors have wrong size";
        return false;
    }
    physJointsPos[0] = actAxesPos[0];
    /* thumb_prox <-- thumb_oc */
    physJointsPos[1] = actAxesPos[1];
    /* thumb_dist <-- coupling_law(thumb_prox) */
    physJointsPos[2] = evaluateCoupledJoint(physJointsPos[1], "thumb");
    /* index_add <-- index_add */
    physJointsPos[3] = actAxesPos[2];
    /* index_prox <-- index_oc */
    physJointsPos[4] = actAxesPos[3];
    /* index_dist <-- coupling_law(index_prox) */
    physJointsPos[5] = evaluateCoupledJoint(physJointsPos[4], "index");
    /* middle_prox <-- middle_oc */
    physJointsPos[6] = actAxesPos[4];
    /* middle_dist <-- coupling_law(middle_prox) */
    physJointsPos[7] = evaluateCoupledJoint(physJointsPos[6], "middle");
    /* ring_prox <-- ring_pinky_oc */
    physJointsPos[8] = actAxesPos[5];
    /* ring_dist <-- coupling_law(ring_prox) */
    physJointsPos[9] = evaluateCoupledJoint(physJointsPos[8], "ring");
    /* pinky_prox <-- ring_pinky_oc */
    physJointsPos[10] = actAxesPos[5];
    /* pinky_dist <-- coupling_law(pinky_prox) */
    physJointsPos[11] = evaluateCoupledJoint(physJointsPos[10], "pinky");

    return true;
}


bool CouplingICubHandMk2::convertFromActuatedAxesToPhysicalJointsVel(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, yarp::sig::Vector& physJointsVel) {
    size_t nrOfPhysicalJoints;
    size_t nrOfActuatedAxes;
    auto ok = getNrOfPhysicalJoints(nrOfPhysicalJoints);
    ok = ok && getNrOfActuatedAxes(nrOfActuatedAxes);
    if (!ok || actAxesPos.size() != nrOfActuatedAxes || physJointsVel.size() != nrOfPhysicalJoints || actAxesVel.size() != nrOfActuatedAxes) {
        yCError(COUPLINGXCUBHANDMK5) << "convertFromPhysicalJointsToActuatedAxesVel: input or output vectors have wrong size";
        return false;
    }

    /**
     * Extract the current position of proximal joints from pos_feedback.
     */
    double lastThumbProx  = actAxesPos[1];
    double lastIndexProx  = actAxesPos[3];
    double lastMiddleProx = actAxesPos[4];
    double lastRingProx   = actAxesPos[5];
    double lastPinkyProx  = actAxesPos[5];

    /**
     * In the following, we use the fact that:
     * /dot{distal_joint} = \partial{distal_joint}{proximal_joint} \dot{proximal_joint}.
     */


    /* thumb_add <-- thumb_add */
    physJointsVel[0] = actAxesVel[0];
    /* thumb_prox <-- thumb_oc */
    physJointsVel[1] = actAxesVel[1];
    /* thumb_dist <-- coupling_law_jacobian(thumb_prox_position) * thumb_prox */
    physJointsVel[2] = evaluateCoupledJointJacobian(lastThumbProx, "thumb") * physJointsVel[1];
    /* index_add <-- index_add */
    physJointsVel[3] = actAxesVel[2];
    /* index_prox <-- index_oc */
    physJointsVel[4] = actAxesVel[3];
    /* index_dist <-- coupling_law_jacobian(index_prox_position) * index_prox */
    physJointsVel[5] = evaluateCoupledJointJacobian(lastIndexProx, "index") * physJointsVel[4];
    /* middle_prox <-- middle_oc */
    physJointsVel[6] = actAxesVel[4];
    /* middle_dist <-- coupling_law_jacobian(middle_prox_position) * middle_prox */
    physJointsVel[7] = evaluateCoupledJointJacobian(lastMiddleProx, "middle") * physJointsVel[6];
    /* ring_prox <-- ring_pinky_oc */
    physJointsVel[8] = actAxesVel[5];
    /* ring_dist <-- coupling_law_jacobian(ring_prox_position) * ring_prox */
    physJointsVel[9] = evaluateCoupledJointJacobian(lastRingProx, "ring") * physJointsVel[8];
    /* pinky_prox <-- ring_pinky_oc */
    physJointsVel[10] = actAxesVel[5];
    /* pinky_dist <-- coupling_law(pinky_prox) */
    physJointsVel[11] = evaluateCoupledJointJacobian(lastPinkyProx, "pinky") * physJointsVel[10];

    return true;
}

bool CouplingICubHandMk2::convertFromActuatedAxesToPhysicalJointsAcc(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesVel, const yarp::sig::Vector& actAxesAcc, yarp::sig::Vector& physJointsAcc) {
    return false;
}
bool CouplingICubHandMk2::convertFromActuatedAxesToPhysicalJointsTrq(const yarp::sig::Vector& actAxesPos, const yarp::sig::Vector& actAxesTrq, yarp::sig::Vector& physJointsTrq) {
    return false;
}
