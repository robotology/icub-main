// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __comanFTsensor_h__
#define __comanFTsensor_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <iCub/DebugInterfaces.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>

#include <boost/concept_check.hpp>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>

#include <robolli/DSP_board.h>
#include <robolli/Boards_iface.h>

#include <comanDevicesHandler.hpp>
#include <Debug.h>

namespace yarp{
    namespace dev{
        class comanFTsensor;
    }
}

// class AnalogData
// {
// private:
//     double *_data;
//     int _size;
//     int _bufferSize;
// public:
//     AnalogData(int ch, int buffsize): _data(0), _size(ch), _bufferSize(buffsize)
//     {
//         _data=new double[_bufferSize];
//         for(int k=0;k<_bufferSize;k++)
//             _data[k]=0;
//     }
//     ~AnalogData()
//     {
//         delete [] _data;
//     }
// 
//     inline double &operator[](int i)
//     { return _data[i]; }
// 
//     inline int size() 
//     { return _size; }
// 
//     inline double *getBuffer()
//     {return _data;}
// };



typedef int AnalogDataFormat;
/*! class yarp::dev::comanFTsensor
 *
 */
class yarp::dev::comanFTsensor:     public yarp::dev::DeviceDriver,
                                    public yarp::dev::IAnalogSensor
{
private:

    // handling classes
    comanDevicesHandler         *_comanHandler;
    Boards_ctrl                 *_boards_ctrl;
    Boards_ctrl::fts_map_t       _fts;

    // remapping of FT sensors
    int                     numberOfBoards;
    int                     *FTmap;

////////////////////
    // parameters
    int             _channels;
    short           _useCalibration;

//     AnalogData *data;
    short status;

    double timeStamp;
    double* scaleFactor;
    yarp::os::Semaphore mutex;

    yarp::os::Bottle initMsg;
    yarp::os::Bottle speedMsg;
    yarp::os::Bottle closeMsg;
    std::string deviceIdentifier;

    // Read useful data from config and check fir correctness
    bool fromConfig(yarp::os::Searchable &config);
    FtBoard* getFTpointer(int j);

public:

    comanFTsensor();
    ~comanFTsensor();

    bool open(yarp::os::Searchable &config);
    bool close();

    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);

    virtual int calibrateChannel(int ch);

    void setDeviceId(std::string id)
    {
        deviceIdentifier=id;
    }

    std::string getDeviceId()
    {
        return deviceIdentifier;
    }
};


#endif   // __comanFTsensor_h__


