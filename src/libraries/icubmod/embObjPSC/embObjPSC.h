/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * LGPL-2.1+ license. See the accompanying LICENSE file for details.
 */


#ifndef __embObjPSC_h__
#define __embObjPSC_h__

#include <string>
#include <mutex>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/sig/Vector.h>

//#include <iCub/FactoryInterface.h>
//#include <iCub/LoggerInterfaces.h>


#include "embObjGeneralDevPrivData.h"

#include "serviceParser.h"


namespace yarp {
    namespace dev {
        class embObjPSC;
    }
}



class yarp::dev::embObjPSC: public yarp::dev::IAnalogSensor,
                            public yarp::dev::DeviceDriver,
                            public eth::IethResource
{

public:

    embObjPSC();
    ~embObjPSC();

    bool open(yarp::os::Searchable &config);
    bool close();

    // IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);
    virtual int calibrateChannel(int ch);

    // IethResource interface
    virtual bool initialised();
    virtual eth::iethresType_t type();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);

private:

    yarp::dev::embObjDevPrivData m_PDdevice;


    std::mutex m_mutex;

    yarp::sig::Vector m_data;


private:

    bool fromConfig(yarp::os::Searchable &config, servConfigPSC_t &serviceConfig);
    bool sendConfig2PSCboards(servConfigPSC_t &serviceConfig);
    bool sendStart2PSCboards(void);
    bool initRegulars(void);
    void cleanup(void);
//     void printServiceConfig(void);


};


#endif

