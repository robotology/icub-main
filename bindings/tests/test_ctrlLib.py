#!/usr/bin/python

# Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Phuong D.H. Nguyen (phuong.nguyen@iit.it)
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.


# Example: Using Kalman filter to estimate a scalar random constant, such as a "voltage reading" from a source.
# So let's assume that it has a constant value of aV (volts), but of course we some noisy readings above and below a volts.
# And we assume that the standard deviation of the measurement noise is 0.1 V.
# Measurement: z = [0.39,	0.50,	0.48,	0.29,	0.25,	0.32,	0.34,	0.48,	0.41,	0.45]
# A=1, B=0, H=1, Q=0, R=0.1

from __future__ import (absolute_import, division,
                        print_function)
from future import standard_library

standard_library.install_aliases()

import yarp

import icub

log = yarp.Log()

A = yarp.Matrix(1, 1)
A[0, 0] = 1.
H = yarp.Matrix(1, 1)
H[0, 0] = 1.
Q = yarp.Matrix(1, 1)
Q[0, 0] = 0.
R = yarp.Matrix(1, 1)
R[0, 0] = 0.1

x0 = yarp.Vector(1, 0.)
P0 = H
kal = icub.Kalman(A, H, Q, R)
kal.init(x0, P0)

z = [0.39, 0.50, 0.48, 0.29, 0.25, 0.32, 0.34, 0.48, 0.41, 0.45]

z_vec = yarp.Vector(len(z))
x = yarp.Vector(len(z))

for i in range(len(z)):
    z_vec[i] = z[i]
    Z = yarp.Vector(1, z[i])
    X = kal.filt(Z)
    x[i] = X[0]

log.info('Measured value                       : {:s}'.format(z_vec.toString(3, 3)))
log.info('Corrected value with 1D Kalman filter: {:s}'.format(x.toString(3, 3)))
