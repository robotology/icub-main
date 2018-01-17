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
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>

#include "IethResource.h"
#include <ethManager.h>
#include <abstractEthResource.h>


#include <yarp/os/LogStream.h>


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
                                        public eth::IethResource
{

public:

    enum { NUMCHANNEL_STRAIN = 6, NUMCHANNEL_MAIS = 15, FORMATDATA_STRAIN = 16, FORMATDATA_MAIS = 8 };


    enum AnalogSensorType
    {
        AS_Type_NONE = 0,
        AS_Type_MAIS = 1,
        AS_Type_STRAIN = 2,
        AS_Type_INERTIAL_MTB = 3
    };

public:

    embObjAnalogSensor();
    ~embObjAnalogSensor();

    // An open function yarp factory compatible
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

    string boardIPstring;
    string boardName;
    eOipv4addr_t ipv4addr;

    //! eth messaging stuff
    eth::TheEthManager *ethManager;
    eth::AbstractEthResource *res;

    bool opened;
    bool verbosewhenok;

    unsigned int    counterSat;
    unsigned int    counterError;
    unsigned int    counterTimeout;

    ////////////////////
    // parameters
    int                 _period;
    int                 _channels;
    int                 _numofsensors;
    short               _useCalibration;
    AnalogSensorType    _as_type;

//    uint8_t _fromInertialPos2DataIndexAccelerometers[eoas_inertial1_pos_max_numberof];
//    uint8_t _fromInertialPos2DataIndexGyroscopes[eoas_inertial1_pos_max_numberof];

    AnalogData *analogdata;
    short status;

    double timeStamp;
    double* scaleFactor;
    yarp::os::Semaphore mutex;

private:

    // for all
    bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size);
    bool fromConfig(yarp::os::Searchable &config);
    bool init();
    void cleanup(void);


    // for strain
    bool fillDatOfStrain(void *as_array_raw);
    bool sendConfig2Strain(void);
    bool getFullscaleValues();


    // for mais
    bool fillDatOfMais(void *as_array_raw);
    bool sendConfig2Mais(void);


    // for inertials
    bool fillDatOfInertial(void *inertialdata);
    bool configServiceInertials(Searchable& globalConfig);
    bool sendConfig2SkinInertial(yarp::os::Searchable &config);
    //eOas_inertial1_position_t getLocationOfInertialSensor(yarp::os::ConstString &strpos);


    // for ??
    void resetCounters();
    void getCounters(unsigned int &saturations, unsigned int &errors, unsigned int &timeouts);
};


#endif

