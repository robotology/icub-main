/*
 * Copyright (C) 2020 iCub Tech - Istituto Italiano di Tecnologia
 * Author:  Marco Accame
 * email:   marco.accame@iit.it
*/


#ifndef __embObjPOS_h__
#define __embObjPOS_h__

#include <string>
#include <mutex>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>


#include "embObjGeneralDevPrivData.h"

#include "serviceParser.h"


namespace yarp {
    namespace dev {
        class embObjPOS;
    }
}



class yarp::dev::embObjPOS: public yarp::dev::DeviceDriver,
                            public yarp::dev::IEncoderArrays,
                            public eth::IethResource
{

public:

    embObjPOS();
    ~embObjPOS();

    bool open(yarp::os::Searchable &config);
    bool close();

    // IEncoderArrays interface
    virtual size_t getNrOfEncoderArrays() const override; // this should return the number of enabledSensor, i.e. 2 or 4, depending on the device

    // This should be the status of the actuator at index sens_index.
    virtual yarp::dev::MAS_status getEncoderArrayStatus(size_t sens_index) const override;

    // This should be the name of the actuator at index sens_index, i.e. serviceConfig.idList[sens_index].
    virtual bool getEncoderArrayName(size_t sens_index, std::string &name) const override;

    // In this case, if we keep the analysis done around this service, out element should contain the position of the actuator at sens_index, i.e. m_data[sens_index]. 
    // Thus, it is actually a single value, not a vector of 2 or 4 elements.
    virtual bool getEncoderArrayMeasure(size_t sens_index, yarp::sig::Vector& out, double& timestamp) const override;

    // I think this cannot return 2 or 4, i.e. the number of enabled-sensors, which is the size of m_data, but it should return 1, as the encoder array is a single entity. 
    // Otherwise, it won't make sense to have the API getNrOfEncoderArrays(), which return the number of enabled sensors. 
    // So, I think that in this case we have a number of encoder arrays equal to the number of enabled sensors, and each one of these arrays has a size equal to 1.
    virtual size_t getEncoderArraySize(size_t sens_index) const override; // this should return the size of data at index sens_index, which should actually be 1. 



    // IethResource interface
    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);

private:

    servConfigPOS_t serviceConfig;
    yarp::dev::embObjDevPrivData m_PDdevice;
    mutable std::mutex m_mutex;
    yarp::sig::Vector m_data; //this should be a vector of 2 or 4 elements, depending on the device, if related to abduction or open-close. Each number is the position of the actuator.


private:

    bool fromConfig(yarp::os::Searchable &config, servConfigPOS_t &serviceConfig);
    bool sendConfig2boards(servConfigPOS_t &serviceConfig);
    bool sendStart2boards(void);
    bool initRegulars(void);
    void cleanup(void);

    void helper_remapperFromSensorToDataIndex(const uint32_t sensorId, uint32_t &dataIndex) const;


};


#endif

