// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup xsensemtx xsensemtx
 *
 * Provide Linux interface for the xsensemtx gyroscope.
 *
 * Copyright (C) 2006 Radu Bogdan Rasu, Alexis Maldonado
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __XSENSMTX__
#define __XSENSMTX__

#include "MTComm.h"
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/dev/PreciselyTimed.h>
#include <string>

namespace yarp{
    namespace dev{
        class XSensMTx;
    }
}

struct XSensMTxParameters
{
    std::string comPortString;
    short comPort;
};

/**
 *
 * @ingroup dev_impl
 *
 * Driver for XSens's MTx IMU unit.
 * @author Radu Bogdan Rusu, Alexis Maldonado
 */
class yarp::dev::XSensMTx : public IGenericSensor, public yarp::dev::IPreciselyTimed, public DeviceDriver
{
public:
    XSensMTx();
    ~XSensMTx();
    
    // IGenericSensor interface.
    virtual bool read(yarp::sig::Vector &out);
    virtual bool getChannels(int *nc);
    virtual bool open(yarp::os::Searchable &config);
    virtual bool calibrate(int ch, double v);
    virtual bool close();

    virtual yarp::os::Stamp getLastInputStamp();

    // Open the device
    bool open(const XSensMTxParameters &par);

private:
    bool start();
    bool stop();

    void *system_resources;
    int nchannels;
    yarp::os::Stamp lastStamp;
};


#endif
