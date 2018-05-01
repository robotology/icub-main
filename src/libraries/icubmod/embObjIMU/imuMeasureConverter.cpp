/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * 
 * Author Valentina Gaggero
 * 
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#include "imuMeasureConverter.h"

//     double accFactor; //raw to metric measure
//     double gyrFactor; //raw to metric measure
//     double magFactor; //raw to metric measure

ImuMeasureConverter::ImuMeasureConverter()
{
    double accFactor = 1.0;
    double gyrFactor = 1.0;
    double magFactor = 1.0;
}

ImuMeasureConverter::ImuMeasureConverter(double accConvFactor, double gyrConvFactor, double magConvFactor)
{
    Initialize(accConvFactor, gyrConvFactor, magConvFactor);
}

void ImuMeasureConverter::Initialize(double accConvFactor, double gyrConvFactor, double magConvFactor)
{
    accFactor = accConvFactor;
    gyrFactor = gyrConvFactor;
    magFactor = magConvFactor;
}
    
double ImuMeasureConverter::convertAcc_raw2metric(double accRaw) const
{
    return accRaw / accFactor;
}
double ImuMeasureConverter::convertGyr_raw2metric(double gyrRaw) const
{
    return gyrRaw / gyrFactor;
}
double ImuMeasureConverter::convertMag_raw2metric(double magRaw) const
{
    return magRaw / magFactor;
}


