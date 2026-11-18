// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Valentina Gaggero
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

#include "measuresConverter.h"



measureConvFactors::measureConvFactors(int numofjoints)
    : angleToEncoder(numofjoints)
    , dutycycleToPWM(numofjoints)
    , ampsToSensor(numofjoints)
    , newtonsToSensor(numofjoints)
    , bemf2raw(numofjoints)
    , ktau2raw(numofjoints)
{
}

