#!/usr/bin/python

# Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Phuong D.H. Nguyen (phuong.nguyen@iit.it)
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.
from math import pi

import yarp

import icub


def rad_to_deg(V):
    V_deg = yarp.Vector(V.size(), 0.0)
    for i in range(V.size()):
        V_deg[i] = V[i]*180./pi

    return V_deg


yarp.Network.init()
icub.init()

name = 'test'
robot = 'icubSim'
arm_name = 'right_arm'

# To use yarp log function
log = yarp.Log()

props = yarp.Property()
props.put("device", "remote_controlboard")
props.put("local", "/" + name + "/" + arm_name)
props.put("remote", "/" + robot + "/" + arm_name)

# create remote driver
_armDriver = yarp.PolyDriver(props)

# query motor control interfaces
_iPos = _armDriver.viewIPositionControl()
iVel = _armDriver.viewIVelocityControl()
_iEnc_arm = _armDriver.viewIEncoders()

# retrieve number of joints
_jnts_arm = _iPos.getAxes()
log.info('[{:s}] Controlling {:d} joints'.format(name, _jnts_arm))

# Cartesian Controller
props_cart = yarp.Property("(device cartesiancontrollerclient)")
props_cart.put("remote", ("/" + robot + "/cartesianController/" + arm_name))
props_cart.put("local", ("/" + name + "/cart_ctrl/" + arm_name))

_iCartDev = yarp.PolyDriver(props_cart)
_iCartCtrl = _iCartDev.viewICartesianControl()

encs = yarp.Vector(_jnts_arm)
_iEnc_arm.getEncoders(encs.data())
log.debug('[{:s}] Encoder values with direct motor control interface  : {:s}'.format(name, encs.subVector(0,6).toString(3,3)))

# End-effector: measured value
arm_cart_pos = yarp.Vector(3, 0.0)
arm_cart_rot = yarp.Vector(4, 0.0)
_iCartCtrl.getPose(arm_cart_pos, arm_cart_rot)
log.debug('[{:s}] End-effector pose obtained with Cartesian Controller: {:s}'.format(name, arm_cart_pos.toString(3,3)))

# iKinChain to obtain virtual End-effector pose
if arm_name == 'right_arm':
    _iArm = icub.iCubArm('right')
elif arm_name == 'left_arm':
    _iArm = icub.iCubArm('left')
else:
    raise Exception('arm_name should be right_arm or left_arm!!!')

_iArm.setAllConstraints(False)

joint_angles = yarp.Vector(_iArm.getAng())

log.debug('[{:s}] joint angles (rad) obtained with iKin: {:s}'.format(name, rad_to_deg(joint_angles).toString(3, 3)))

# End-effector: estimated with iKin
arm_real = _iArm.EndEffPose(False)
log.debug('[{:s}] End-effector pose obtained with iKin : {:s}'.format(name, arm_real.toString(3,3)))

