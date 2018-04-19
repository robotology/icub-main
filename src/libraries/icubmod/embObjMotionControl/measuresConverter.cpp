// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Valentina Gaggero
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

#include "measuresConverter.h"



measureConvFactors::measureConvFactors(int numofjoints)
{
    angleToEncoder   = new double [numofjoints];
    dutycycleToPWM   = new double [numofjoints];
    ampsToSensor     = new double [numofjoints];
    newtonsToSensor  = new double [numofjoints];
    bemf2raw         = new double [numofjoints];
    ktau2raw         = new double [numofjoints];
}

measureConvFactors::~measureConvFactors()
{
    if (angleToEncoder)   delete [] angleToEncoder;
    if (dutycycleToPWM)   delete [] dutycycleToPWM;
    if (ampsToSensor)     delete [] ampsToSensor;
    if (newtonsToSensor)  delete [] newtonsToSensor;
    if (bemf2raw)         delete [] bemf2raw;
    if (ktau2raw)         delete [] ktau2raw;

    angleToEncoder = nullptr;
    dutycycleToPWM = nullptr;
    ampsToSensor = nullptr;
    newtonsToSensor  = nullptr;
    ktau2raw = nullptr;
    bemf2raw = nullptr;
}

