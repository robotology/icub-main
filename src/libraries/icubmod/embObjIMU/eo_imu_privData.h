/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 * Author: Valentina Gaggero
 * This software may be modified and distributed under the terms of the
 * BSD-3-Clause license. See the accompanying LICENSE file for details.
 */

#ifndef __eo_imu_privData_h__
#define __eo_imu_privData_h__

#include "embObjGeneralDevPrivData.h"
#include "imuMeasureConverter.h"
#include <yarp/sig/Vector.h>
#include <mutex>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>


#include "serviceParser.h"


namespace yarp {
    namespace dev {
        class eo_imu_privData;
        class PositionMaps;
        class SensorsData;
    }
}


class yarp::dev::PositionMaps // data used for handling the received messsages
{
    std::uint8_t canpositionmap[eoas_sensors_numberof][eOcanports_number][16];
    std::uint8_t ethpositionmap[eoas_sensors_numberof];
public:
    PositionMaps();
    ~PositionMaps();
    bool init(servConfigImu_t &servCfg);
    bool getIndex(const eOas_inertial3_data_t *data, uint8_t &index, eOas_sensor_t &type);
    bool getIndex(eOas_sensor_t type, uint8_t canbus, uint8_t canaddress, uint8_t& index);
    static bool getCanAddress(const eOas_inertial3_data_t *data, uint8_t &canbus, uint8_t &canaddress);
};



typedef struct
{
    std::string name;
    std::string framename;
    yarp::sig::Vector values;
    uint8_t state;
    double timestamp;
} sensorInfo_t;

class yarp::dev::SensorsData
{
private:
    std::vector<std::vector<sensorInfo_t>> mysens;
    mutable std::mutex mutex;
    string errorstring;

public:
    std::vector<ImuMeasureConverter> measConverters;
    SensorsData();
    void init(servConfigImu_t &servCfg, string error_string);
    bool update(eOas_sensor_t type, uint8_t index, eOas_inertial3_data_t *newdata);
    bool updateStatus(eOas_sensor_t type, uint8_t index, eOas_inertial3_sensorstatus_t &status);
    bool outOfRangeErrorHandler(const std::out_of_range& oor) const;

    size_t getNumOfSensors(eOas_sensor_t type) const;
    uint8_t getSensorStatus(size_t sens_index, eOas_sensor_t type) const;
    bool getSensorName(size_t sens_index, eOas_sensor_t type, std::string &name) const;
    bool getSensorFrameName(size_t sens_index, eOas_sensor_t type, std::string &frameName) const;
    bool getSensorMeasure(size_t sens_index, eOas_sensor_t type, yarp::sig::Vector& out, double& timestamp) const;
};






class yarp::dev::eo_imu_privData : public yarp::dev::embObjDevPrivData
{
public:
    yarp::dev::SensorsData sens;
    yarp::dev::PositionMaps maps;

    eo_imu_privData(std::string name);
    ~eo_imu_privData();
    bool fromConfig(yarp::os::Searchable &config, servConfigImu_t &servCfg);
    bool sendConfing2board(servConfigImu_t &servCfg);
    bool initRegulars(void);
    yarp::dev::MAS_status sensorState_eo2yarp(eOas_sensor_t type, uint8_t eo_state);

    /* This function prints state of sensor when it is not ok:
     *  - for gyro, acc and mag: state=3 means taht the sensor is completely calibrated; if state is less than 3 than it has some calibration issues;
     *  - for "composed measure" they are completely calibarted if all tree sensors are calibrated, so they are completely calibrated if their state is 9 (3+3+3).
     *  For more information see IMUbosgh BNO055 data sheet page 68 */
    void debugPrintStateNotOK(eOas_sensor_t type, uint8_t eo_state);

};



#endif //__eo_imu_privData_h__