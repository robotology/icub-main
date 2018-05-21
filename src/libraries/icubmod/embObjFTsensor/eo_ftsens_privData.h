/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Valentina Gaggero
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef __eo_ftsens_privData_h__
#define __eo_ftsens_privData_h__

#include "embObjGeneralDevPrivData.h"


#include "serviceParser.h"

namespace yarp {
    namespace dev {
        class eo_ftsens_privData;
    }
}

class yarp::dev::eo_ftsens_privData : public yarp::dev::embObjDevPrivData
{
public:
    
    enum { strain_Channels = 6, strain_FormatData = 16 };
    
    yarp::os::Semaphore mutex;
    std::string devicename;
    std::vector<double> analogdata;
    std::vector<double> offset;
    std::vector<double> scaleFactor;
    
    bool scaleFactorIsFilled;
    bool useCalibValues;
    double timestampAnalogdata;
    
    bool useTemperature;
    int lastTemperature;
    double timestampTemperature;
    
    
    eo_ftsens_privData(std::string name);
    ~eo_ftsens_privData();
    
    bool fromConfig(yarp::os::Searchable &config,  servConfigFTsensor_t &serviceConfig);
    bool initRegulars(servConfigFTsensor_t &serviceConfig);
    void printServiceConfig(servConfigFTsensor_t &serviceConfig);
    bool fillScaleFactor(servConfigFTsensor_t &serviceConfig);
    bool sendConfig2Strain(servConfigFTsensor_t &serviceConfig);
    bool updateStrainValues(eOprotID32_t id32, double timestamp, void* rxdata);
    bool fillTemperatureEthServiceInfo(eOmn_serv_parameter_t &ftSrv, eOmn_serv_parameter_t &tempSrv);
    
};

#endif

