/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * 
 * Author Valentina Gaggero
 * 
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef __ImuMeasureConverter_h__
#define __ImuMeasureConverter_h__

// namespace yarp {
//     namespace dev {
//         class ImuMeasureConverter;
//     }
// }
class ImuMeasureConverter
{
private:
    double accFactor; //raw to metric measure
    double gyrFactor; //raw to metric measure
    double magFactor; //raw to metric measure
    double eulFactor; //raw to metric measure
public:
    ImuMeasureConverter();
    ImuMeasureConverter(double accConvFactor, double gyrConvFactor, double magConvFactor, double eulConvFactor);
    void Initialize(double accConvFactor, double gyrConvFactor, double magConvFactor, double eulConvFactor);
    
    double convertAcc_raw2metric(double) const;
    double convertGyr_raw2metric(double) const;
    double convertMag_raw2metric(double) const;
    double convertEul_raw2metric(double) const;
};

#endif