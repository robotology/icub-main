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
#include <iCub/DebugInterfaces.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>
#include <hostTransceiver.hpp>
#include <ethResource.h>
#include <ethManager.h>


#include "FeatureInterface.h"  

#include "IethResource.h"

#include <yarp/os/LogStream.h>


namespace yarp{
    namespace dev{
        class embObjAnalogSensor;
        class TheEthManager;
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
                                        public IethResource
{

public:

    enum { EMBAS_SIZE_INFO  = 128 };

    enum { NUMCHANNEL_STRAIN = 6, NUMCHANNEL_MAIS = 15, FORMATDATA_STRAIN = 16, FORMATDATA_MAIS = 8 };

    enum SensorStatus
    {
        ANALOG_IDLE=0,
        ANALOG_OK=1,
        ANALOG_NOT_RESPONDING=-1,
        ANALOG_SATURATION=-2,
        ANALOG_ERROR=-3,
    };

    enum AnalogSensorType
    {
        AS_NONE = 0,
        AS_MAIS = 1,
        AS_STRAIN = 2,
        AS_INERTIAL_MTB = 3
    };


private:

    //! eth messaging stuff
    TheEthManager       *ethManager;
    ethResources        *res;
    ethFeature_t        _fId;

    bool opened;
    bool verbosewhenok;


    //! debug messages
    unsigned int    counterSat;
    unsigned int    counterError;
    unsigned int    counterTimeout;
    char            info[EMBAS_SIZE_INFO];   

    ////////////////////
    // parameters
    int                 _period;
    int                 _channels;
    int                 _numofsensors;
    short               _useCalibration;
    //AnalogDataFormat    _format;
    AnalogSensorType    _as_type;

    uint8_t _fromInertialPos2DataIndexAccelerometers[eoas_inertial_pos_max_numberof];
    uint8_t _fromInertialPos2DataIndexGyroscopes[eoas_inertial_pos_max_numberof];

    AnalogData *data;
    short status;

    double timeStamp;
    double* scaleFactor;
    yarp::os::Semaphore mutex;


    // Read useful data from config and check for correctness
    bool fromConfig(yarp::os::Searchable &config);
    bool getFullscaleValues();
    bool sendConfig2Strain(void);
    bool sendConfig2SkinInertial(yarp::os::Searchable &config);
    bool sendConfig2Mais(void);
    bool fillDatOfStrain(void *as_array_raw);
    bool fillDatOfMais(void *as_array_raw);
    bool fillDatOfInertial(void *inertialdata);
    bool isEpManagedByBoard();

    bool configServiceInertials(Searchable& globalConfig);

    eOas_inertial_position_t getLocationOfInertialSensor(yarp::os::ConstString &strpos);

public:

    embObjAnalogSensor();
    ~embObjAnalogSensor();
//     bool handleAnalog(void *);
    
    // An open function yarp factory compatible
    bool open(yarp::os::Searchable &config);
    bool close();
    void cleanup(void);

    // IAnalogSensor interface
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

    virtual bool initialised();
    virtual bool update(eOprotID32_t id32, double timestamp, void* rxdata);
};


#endif

