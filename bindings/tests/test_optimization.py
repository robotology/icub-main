#!/usr/bin/python

# Copyright (C) Tobias Fischer
# All rights reserved.
# Author: Tobias Fischer (t.fischer@imperial.ac.uk)
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

from __future__ import (absolute_import, division,
                        print_function)
from future import standard_library

standard_library.install_aliases()

import yarp
import icub
import math
import numpy as np
import time

def np_to_yarp(v):
    if len(v.shape) == 1: # Vector
        yarp_vec = yarp.Vector(v.shape[0])
        for i in range(v.shape[0]):
            yarp_vec[i] = v[i]
        return yarp_vec
    elif len(v.shape) == 2: # Matrix
        yarp_mat = yarp.Matrix(v.shape[0], v.shape[1])
        for i in range(v.shape[0]):
            for j in range(v.shape[1]):
                yarp_mat[i, j] = v[i, j]
        return yarp_mat
    else:
        raise Exception("Only 1D or 2D numpy arrays are supported")

# no bindings for yarp.math, so let's implement euler2dcm ourselves
# following http://www.yarp.it/math_8cpp_source.html
def euler2dcm(v):
    assert(v.shape[0] >= 3)

    Rza, Ryb, Rzg = np.eye(4), np.eye(4), np.eye(4)

    alpha=v[0]
    ca=math.cos(alpha)
    sa=math.sin(alpha)

    beta=v[1]
    cb=math.cos(beta)
    sb=math.sin(beta)

    gamma=v[2]
    cg=math.cos(gamma)
    sg=math.sin(gamma)

    Rza[0,0]=ca
    Rza[1,1]=ca
    Rza[1,0]=sa
    Rza[0,1]=-sa
    Rzg[0,0]=cg
    Rzg[1,1]=cg
    Rzg[1,0]=sg
    Rzg[0,1]=-sg
    Ryb[0,0]=cb
    Ryb[2,2]=cb
    Ryb[2,0]=-sb
    Ryb[0,2]=sb;

    return np.matmul(np.matmul(Rza,Ryb),Rzg)

yarp.Network.init()
icub.init()

# define the "unknown" transformation:
# translation
p = np.empty(3)
p[0]=0.3
p[1]=-2.0
p[2]=3.1
# rotation
euler = np.empty(3)
euler[0] = math.pi / 4.0
euler[1] = math.pi / 6.0
euler[2] = math.pi / 3.0

H=euler2dcm(euler)
H[0, 3]=p[0]
H[1, 3]=p[1]
H[2, 3]=p[2]

# ansisotropic scale
scale = np.empty(3)
scale[0]=0.9
scale[1]=0.8
scale[2]=1.1

h_scale = np.append(scale, 1.0)
print('h_scale', h_scale)
calibrator = icub.CalibReferenceWithMatchedPoints()

#generate randomly the two clouds of matching 3D points
for i in range(10):
    p0 = np.random.uniform(-5.0, 5.0, 3)
    p0 = np.append(p0, 1.0)

    #make data a bit dirty (add up noise in the range (-1,1) [cm])
    eps = np.random.uniform(-0.01, 0.01, 3)
    eps = np.append(eps, 0.0)

    p1=h_scale*np.matmul(H,p0).T+eps
    #feed the calibrator with the matching pair
    calibrator.addPoints(np_to_yarp(p0),np_to_yarp(p1))

# set the working bounding box where the solution is seeked
bb_min = yarp.Vector(3, -5.0)
bb_max = yarp.Vector(3, 5.0)

calibrator.setBounds(bb_min,bb_max)

# carry out the calibration
t0 = time.time()
x = calibrator.calibrate()
dt = time.time()-t0

success, Hcap, scalecap, error=x

# final report
print("success", success)
print("H", H)
print("scale", scale)

print("Hcap", Hcap.toString(3, 3))
print("scalecap", scalecap.toString(3, 3))

print("residual error", error, "[m]")
print("calibration performed in ", dt, "[s]")


