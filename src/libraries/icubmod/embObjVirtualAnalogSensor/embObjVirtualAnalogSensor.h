// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __analogVirtualSensorEth_h__
#define __analogVirtualSensorEth_h__


#include <iCub/FactoryInterface.h>
#include <yarp/dev/IVirtualAnalogSensor.h>

#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>


#include <yarp/os/LogStream.h>


namespace yarp{
    namespace dev{
        class embObjVirtualAnalogSensor;
    }
}



/*! class yarp::dev::embObjVirtualAnalogSensor
 * 
 */
class yarp::dev::embObjVirtualAnalogSensor:     public yarp::dev::IVirtualAnalogSensor,
                                                public yarp::dev::DeviceDriver,
                                                public eth::IethResource
{
private:

    string boardIPstring;
    string boardName;
    eOipv4addr_t ipv4addr;

    eth::TheEthManager *ethManager;
    eth::AbstractEthResource *res;

    ////////////////////
    // parameters
    int             _channels;
    short           _useCalibration;
    double          *_fullscale;             /** converts input values to HW fullscale */
    double          *_resolution;                /** number of bytes of resolution for this measure */

    bool            _verbose;
    VAS_status      _status;

    bool opened;

    // Read useful data from config and check for correctness
    bool fromConfig(yarp::os::Searchable &config);

public:


    embObjVirtualAnalogSensor();
    ~embObjVirtualAnalogSensor();
    
    // An open function yarp factory compatible
    bool open(yarp::os::Searchable &config);
    void cleanup(void);
    bool close();

    virtual bool initialised();
    virtual bool update(eOprotID32_t id32, double timestamp, void *rxdata);
    virtual eth::iethresType_t type();

    // IvirtualAnalogSensor interface
    virtual IVirtualAnalogSensor::VAS_status getVirtualAnalogSensorStatus(int ch);
    virtual int getVirtualAnalogSensorChannels();
    virtual bool updateVirtualAnalogSensorMeasure(yarp::sig::Vector &measure);
    virtual bool updateVirtualAnalogSensorMeasure(int ch, double &measure);
};


#endif


