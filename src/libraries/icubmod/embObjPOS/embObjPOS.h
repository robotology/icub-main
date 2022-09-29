
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
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/sig/Vector.h>



#include "embObjGeneralDevPrivData.h"

#include "serviceParser.h"


namespace yarp {
    namespace dev {
        class embObjPOS;
    }
}



class yarp::dev::embObjPOS: public yarp::dev::DeviceDriver,
                            public yarp::dev::IAnalogSensor,
                            public eth::IethResource
{

public:

    embObjPOS();
    ~embObjPOS();

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

    bool fromConfig(yarp::os::Searchable &config, servConfigPOS2_t &serviceConfig);
    bool sendConfig2boards(servConfigPOS2_t &serviceConfig);
    bool sendStart2boards(void);
    bool initRegulars(void);
    void cleanup(void);
//     void printServiceConfig(void);


};


#endif

