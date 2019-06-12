#!/usr/bin/python

# Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
# All rights reserved.
# Author: Phuong D.H. Nguyen (phuong.nguyen@iit.it)
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

# Requirement: icubsim with following setting iCub_parts_activation.ini
# [COLLISIONS]
# self_collisions on
# covers_collisions on
# [SENSORS]
# pressure off
# whole_body_skin_emul on

# Working condition: there are physical contacts between skin parts with objects, otherwise, skinContactList is empty

from __future__ import (absolute_import, division,
                        print_function)
from future import standard_library

standard_library.install_aliases()

import yarp
import icub

log = yarp.Log()
yarp.Network.init()
icub.init()

skinEventsPortIn = icub.BufferedPortSkinContactList()
skinEventsPortIn.open("/test/skin_events:i")
connected = yarp.Network.connect("/icubSim/skinManager/skin_events:o", "/test/skin_events:i")

if connected:

    scl = icub.skinContactList()

    scl = skinEventsPortIn.read(True)

    if scl is not None:
        print(scl.toString())

    dcl = scl.toDynContactList()
    print(dcl.toString())

else:
    log.error('no connection')

skinEventsPortIn.close()
