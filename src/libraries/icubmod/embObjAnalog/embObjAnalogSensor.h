// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __analogSensorEth_h__
#define __analogSensorEth_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>
// #include <yarp/dev/PreciselyTimed.h>
#include <iCub/DebugInterfaces.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>
#include <ethManager.h>
#include <hostTransceiver.hpp>


#include "FeatureInterface_hid.h"         // Interface with embObj world (callback)

#include "Debug.h"


namespace yarp{
    namespace dev{
        class embObjAnalogSensor;
    }
}

class AnalogData
{
private:
    double *_data;
    int _size;
    int _bufferSize;
public:
    AnalogData(int ch, int buffsize): _data(0), _size(ch), _bufferSize(buffsize)
    {
        _data=new double[_bufferSize];
        for(int k=0;k<_bufferSize;k++)
            _data[k]=0;
    }
    ~AnalogData()
    {
        delete [] _data;
    }

    inline double &operator[](int i)
    { return _data[i]; }

    inline int size() 
    { return _size; }

    inline double *getBuffer()
    {return _data;}
};



typedef int AnalogDataFormat;
/*! class yarp::dev::embObjAnalogSensor
 * 
 */
class yarp::dev::embObjAnalogSensor:    public yarp::dev::IAnalogSensor,
                                        public yarp::dev::DeviceDriver,
                                        public IiCubFeature
{
public:
    enum AnalogDataFormat
    {
        ANALOG_FORMAT_8,
        ANALOG_FORMAT_16,
    };

    enum SensorStatus
    {
        ANALOG_IDLE=0,
        ANALOG_OK=1,
        ANALOG_NOT_RESPONDING=-1,
        ANALOG_SATURATION=-2,
        ANALOG_ERROR=-3,
    };

private:

    //! embObj stuff
    ethResources    *res;
    FEAT_ID         _fId;


    //! debug messages
    unsigned int    counterSat;
    unsigned int    counterError;
    unsigned int    counterTimeout;
    char            info[SIZE_INFO];   // debug per comodità

    ////////////////////
    // parameters
    int             _period;
    int             _channels;
    short           _useCalibration;

    // obsolete
//    short           boardId;            // used in CAN analogSensor
//    AnalogDataFormat dataFormat;        // used in CAN analogSensor
//    bool            isVirtualSensor;    //RANDAZ ... a new class for virtual sensor will be create


    AnalogData *data;
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
public:

    embObjAnalogSensor();
    ~embObjAnalogSensor();
//     bool handleAnalog(void *);
    
    // An open function yarp factory compatible
//     bool open(int channels, AnalogDataFormat f, short bId, short useCalib, bool isVirtualSensor);
    bool open(yarp::os::Searchable &config);
    
    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);

    virtual int calibrateChannel(int ch);
    
    // Internal stuff
    void resetCounters()
    {
        counterSat=0;
        counterError=0;
        counterTimeout=0;
    }

    void getCounters(unsigned int &sat, unsigned int &err, unsigned int &to)
    {
        sat=counterSat;
        err=counterError;
        to=counterTimeout;
    }

    void setDeviceId(std::string id)
    {
        deviceIdentifier=id;
    }

    std::string getDeviceId()
    {
        return deviceIdentifier;
    }

    // will this be substitute by an axis map?
#ifdef _OLD_STYLE_
    short getId()
    { return boardId;}

    short getStatus()
    { return status;}

    bool isOpen()
    {
        if (data)
            return true;
        else
            return false;
    }

    short getUseCalibration()
        {return useCalibration;}
    double* getScaleFactor()
        {return scaleFactor;}
    double getScaleFactor(int chan)
        {
            if (chan>=0 && chan<data->size())
                return scaleFactor[chan];
            else
                return 0;
        }
#endif



    // this method will be added in the virtualAnalogSensor, not here
    //virtual int sendVirtualData(const yarp::sig::Vector& value);
    
    // embObj interface
    bool init();
    virtual bool fillData(void *as_array);
};


#endif


