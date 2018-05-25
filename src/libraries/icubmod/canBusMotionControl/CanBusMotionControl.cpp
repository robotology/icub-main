// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2008 Robotcub Consortium
* Author: Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/
///
/// $Id: CanBusMotionControl.cpp,v 1.62 2009/07/29 08:37:58 babybot Exp $
///
///

/// general purpose stuff.

#include <yarp/os/Time.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <ace/config.h>
#include <ace/Log_Msg.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

//the following activates the DEBUG macro in canControlUtil.h
//#define CAN_DEBUG
//#define CANBUSMC_DEBUG

#include "ThreadTable2.h"
#include "ThreadPool2.h"

#include <string>
#include <iostream>
#include <algorithm>
#include <string.h>

/// specific to this device driver.
#include "CanBusMotionControl.h"

#include "can_string_generic.h"
/// get the message types from the DSP code.
#include "messages.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/dev/ControlBoardInterfacesImpl.inl>

#include "canControlConstants.h"
#include "canControlUtils.h"

#ifdef WIN32
    #pragma warning(once:4355)
#endif

const int REPORT_PERIOD=6; //seconds
const double BCAST_STATUS_TIMEOUT=6; //seconds


using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

inline void PRINT_CAN_MESSAGE(const char *str, CanMessage &m)
{
#ifdef CANBUSMC_DEBUG
    yDebug("%s", str);
    yDebug(" S:%d R:%d Ch:%d M:%d\n", getSender(m), getRcp(m), m.getData()[0]&0x80, m.getData()[0]&0x7F);
#endif
}

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    yError("%s not yet implemented for CanBusMotionControl\n", txt);

    return false;
}

inline bool DEPRECATED (const char *txt)
{
    yError("%s has been deprecated for CanBusMotionControl\n", txt);

    return false;
}


//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
bool validate(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError("%s not found\n", key1.c_str());
        return false;
    }
    if(tmp.size()!=size)
    {
        yError("%s incorrect number of entries\n", key1.c_str());
        return false;
    }
    out=tmp;
    return true;
}

bool validate_optional(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        return false;
    }
    if(tmp.size()!=size)
    {
        yError("%s incorrect number of entries\n", key1.c_str());
        return false;
    }
    out=tmp;
    return true;
}

static can_string_generic cstring[CAN_MAX_CARDS];

///
///
struct BCastElement
{
    void resetStats()
    {
        _count=0;
        _minDt=10e22;
        _maxDt=0;
        _accDt=0;
        _lastDt=0;
        _stamp=Time::now();
    }

    BCastElement()
    {
        _value=0;
        resetStats();
    }

    int _value;
    double _stamp;
    double _accDt;
    int _count;
    double _maxDt;
    double _minDt;
    double _lastDt;

    inline void update(int value)
    { _value=value; }
    
    inline int getValue()
    { return _value; }

    inline void getStats(int &it, double &dT, double &min, double &max)
    {
        it=_count;
        if (_count==0)
            {
                dT=0;
                min=0;
                max=0;
            }
        else
            {        
                dT=_accDt/_count;
                min=_minDt;
                max=_maxDt;
            }
        //scale to ms
        dT*=1000;
        min*=1000;
        max*=1000;
    }

    inline void update(int value, double st)
    {
        double tmpDt=st-_stamp;
        _value=value;
        _stamp=st;
        _accDt+=tmpDt;
        _lastDt=tmpDt;
        if (tmpDt>_maxDt)
            _maxDt=tmpDt;
        if (tmpDt<_minDt)
            _minDt=tmpDt;

        _count++;
    }

    inline double getStamp()
    {
        return _stamp;
    }
};





///
class BCastBufferElement
{
public:
    // msg 1
    BCastElement _position_joint;
    BCastElement _position_rotor;
    BCastElement _speed_rotor;
    BCastElement _accel_rotor;

    // msg 2
    short _pid_value;
    double _update_v;

    // msg 3
    short _axisStatus;
    char  _canStatus;
    char  _boardStatus;
    unsigned char _controlmodeStatus;
    double _update_e;

    // msg 3b
    unsigned char _interactionmodeStatus;
    double _update_e2;

    // msg 4
    short _current;
    double _update_c;

    // msg 4b
    short _position_error;
    short _torque_error;   
    double _update_r;

    // msg 4c
    short _speed_joint;
    short _accel_joint;
    double _update_s;

    // msg 4d (special message)
    double _torque;
    double _update_t;
    
    // msg 6
    unsigned int _canTxError;
    // msg 7
    unsigned int _canRxError;

    // Main Loop Overflow Counter
    unsigned int _mainLoopOverflowCounter;

    int _address;

    BCastBufferElement () { zero (); }
    bool isFaultOk             () { return (!_axisStatus ) ; }
    bool isFaultUndervoltage   () { return _axisStatus & 0x01; }
    bool isFaultOverload       () { return _axisStatus & 0x02; }
    bool isOverCurrent         () { return _axisStatus & 0x08; }
    bool isFaultExternal       () { return _axisStatus & 0x04; }
    bool isHallSensorError     () { return _axisStatus & 0x10; }
    bool isAbsEncoderError     () { return _axisStatus & 0x20; }
    bool isOpticalEncoderError () { return _axisStatus & 0x40; }
    bool isCanTxOverflow       () { return _canStatus & 0x01; }
    bool isCanBusOff           () { return _canStatus & 0x02; }
    bool isCanTxError          () { return _canStatus & 0x04; }
    bool isCanRxError          () { return _canStatus & 0x08; }
    bool isCanTxOverrun        () { return _canStatus & 0x10; }
    bool isCanRxWarning        () { return _canStatus & 0x20; }
    bool isCanRxOverrun        () { return _canStatus & 0x40; }
    bool isMainLoopOverflow    () { return _boardStatus & 0x01; }
    bool isOverTempCh1         () { return _boardStatus & 0x02; }
    bool isOverTempCh2         () { return _boardStatus & 0x04; } 
    bool isTempErrorCh1        () { return _boardStatus & 0x08; }
    bool isTempErrorCh2        () { return _boardStatus & 0x10; } 
    void ControlStatus         (int net, short controlmode,short addr) 
    {
        switch (controlmode)
        {
        case    icubCanProto_controlmode_idle:
            yInfo ("[%d] board  %d MODE_IDLE \r\n", net, addr);
            break;
        case    icubCanProto_controlmode_position:
            yInfo ("[%d] board  %d MODE_POSITION \r\n", net, addr);
            break;
        case    icubCanProto_controlmode_velocity:
            yInfo ("[%d] board  %d MODE_VELOCITY \r\n", net, addr);
            break;
        case    icubCanProto_controlmode_torque:
            yInfo ("[%d] board  %d MODE_TORQUE \r\n", net, addr);
            break;
        case    icubCanProto_controlmode_impedance_pos:
            yInfo ("[%d] board  %d MODE_IMPEDANCE_POS \r\n", net, addr);
            break;
        case    icubCanProto_controlmode_impedance_vel:
            yInfo ("[%d] board  %d MODE_IMPEDANCE_VEL \r\n", net, addr);
            break;
        default:
            yError ("[%d] board  %d MODE_UNKNOWN \r\n", net, addr);
            break;
        }
    }


    void zero (void)
    {
        _position_joint._value = 0;
        _position_rotor._value = 0;
        _speed_rotor._value = 0;
        _accel_rotor._value = 0;
        _pid_value = 0;
        _current = 0;
        _axisStatus=0;
        _canStatus=0;
        _boardStatus=0;
        _controlmodeStatus=0;
        _interactionmodeStatus=0;
        _position_error = 0;
        _torque_error = 0;
        _speed_joint = 0;
        _accel_joint = 0;

        _torque=0;
        _update_t = .0;

        _update_v = .0;
        _update_e = .0;
        _update_e2= .0;
        _update_c = .0;
        _update_r = .0;
        _update_s = .0;

        _address=-1;
        _canTxError=0;
        _canRxError=0;
        _mainLoopOverflowCounter=0;
    }
};


#include <stdarg.h>
#include <stdio.h>
const int PRINT_BUFFER_LENGTH=255;
class CanBusResources
{
private:
    CanBusResources (const CanBusResources&);
    void operator= (const CanBusResources&);
    PolyDriver polyDriver;

public:
    ICanBus *iCanBus;
    ICanBufferFactory *iBufferFactory;
    ICanBusErrors *iCanErrors;

    CanBusResources ();
    ~CanBusResources ();

    bool initialize (const CanBusMotionControlParameters& parms);
    bool initialize (yarp::os::Searchable &config);
    bool uninitialize ();
    bool read ();

    bool startPacket ();
    bool addMessage (int msg_id, int joint);
    // add message, append request to request list
    bool addMessage (int id, int joint, int msg_id);

    bool writePacket ();

    bool printMessage (const CanMessage &m);
    bool dumpBuffers (void);
    inline int getJoints (void) const { return _njoints; }
    inline bool getErrorStatus (void) const { return _error_status;}

    void printMessage(const char *fmt, ...)
    {
        va_list ap; 
        va_start(ap, fmt); 
#ifdef WIN32
        _vsnprintf(buffer, DEBUG_PRINTF_BUFFER_LENGTH, fmt, ap); 
#else
        vsnprintf(buffer, DEBUG_PRINTF_BUFFER_LENGTH, fmt, ap); 
#endif
        yDebug("%s", buffer);
        va_end(ap);
    }

    char buffer[DEBUG_PRINTF_BUFFER_LENGTH];
public:
    enum { CAN_TIMEOUT = 20, CAN_POLLING_INTERVAL = 10 };

    // HANDLE _handle;/// the actual ddriver handle.
    bool _initialized;
    unsigned int _timeout;/// this is my thread timeout.

    int _polling_interval;/// thread polling interval.
    int _speed;/// speed of the bus.
    int _networkN;/// network number.

    int _txQueueSize;
    int _rxQueueSize;
    int _txTimeout;
    int _rxTimeout;

    unsigned int _readMessages;/// size of the last read buffer.
    unsigned int _writeMessages;/// size of the write packet.
    unsigned int _echoMessages;/// size of the last read buffer.

    CanBuffer _readBuffer;/// read buffer.
    CanBuffer _writeBuffer;/// write buffer.
    CanBuffer _replyBuffer;/// reply buffer.
    CanBuffer _echoBuffer;/// echo buffer.

    BCastBufferElement *_bcastRecvBuffer;/// local storage for bcast messages.

    unsigned char _my_address;/// 
    unsigned char _destinations[CAN_MAX_CARDS];/// list of connected cards (and their addresses).
    int _velShifts[CAN_MAX_CARDS];/// list of velocity shift
    unsigned char *_destInv;
    std::string* _jointNames;

    int _njoints;/// number of joints (ncards * 2).

    bool _error_status;/// error status of the last packet.

    /// used to spy on can messages.
    int _filter;/// don't print filtered messages.

    char _printBuffer[16384];                   /// might be better with dynamic allocation.
    RequestsQueue *requestsQueue;
};
inline CanBusResources& RES(void *res) { return *(CanBusResources *)res; }

class TBR_CanBackDoor: public BufferedPort<Bottle>
{
    CanBusResources *bus;
    Semaphore *semaphore;
    TBR_AnalogSensor *ownerSensor;
    bool canEchoEnabled;

public:
    TBR_CanBackDoor()
    {
        bus=0;
        ownerSensor=0;
        canEchoEnabled=false;
    }

    void setUp(CanBusResources *p, Semaphore *sema, bool echo, TBR_AnalogSensor *owner)
    {
        semaphore=sema;
        bus=p;
        ownerSensor= owner;
        useCallback();
        canEchoEnabled = echo;
    }

    virtual void onRead(Bottle &b);
};


void TBR_CanBackDoor::onRead(Bottle &b)
{
    if (!semaphore)
        return;

    double dval[6] = {0,0,0,0,0,0};

    semaphore->wait();
    //RANDAZ_TODO: parse vector b
    int len = b.size();
    int commandId = b.get(0).asInt();
    int i=0;

    static double timePrev=Time::now();
    static int    count_timeout=0;
    static int    count_saturation_i[6]={0,0,0,0,0,0};
    static int    count_saturation=0;
    double timeNow=Time::now();
    double diff = timeNow - timePrev;

    if (diff > 0.012)
    {
        count_timeout++;
        yWarning("PORT: %s **** TIMEOUT : %f COUNT: %d \n", this->getName().c_str(), diff, count_timeout);
    }
    timePrev=Time::now();

    switch (commandId)
    {
        case 1: //shoulder torque message
            dval[0] = b.get(1).asDouble(); //shoulder 1 pitch
            dval[1] = b.get(2).asDouble(); //shoulder 2 roll
            dval[2] = b.get(3).asDouble(); //shoulder 3 yaw
            dval[3] = b.get(4).asDouble(); //elbow
            dval[4] = b.get(5).asDouble(); //wrist pronosupination
            dval[5] = 0; 
        break;
        case 2: //legs torque message
            dval[0] = b.get(1).asDouble(); //hip pitch
            dval[1] = b.get(2).asDouble(); //hip roll
            dval[2] = b.get(3).asDouble(); //hip yaw
            dval[3] = b.get(4).asDouble(); //knee
            dval[4] = b.get(5).asDouble(); //ankle pitch
            dval[5] = b.get(6).asDouble(); //ankle roll
        break;
        case 3:
            dval[0] = b.get(6).asDouble(); //wrist yaw
            dval[1] = b.get(7).asDouble(); //wrist pitch
            dval[2] = 0;
            dval[3] = 0;
            dval[4] = 0;
            dval[5] = 0; 
        break;
        case 4:
            dval[0] = b.get(1).asDouble(); //torso yaw (respect gravity)
            dval[1] = b.get(2).asDouble(); //torso roll (lateral movement)
            dval[2] = b.get(3).asDouble(); //torso pitch (front-back movement)
            dval[3] = 0;
            dval[4] = 0;
            dval[5] = 0; 
        break;
        default:
            yWarning("Got unexpected message on backdoor: %s\n", this->getName().c_str());
            return;
        break;
    }

    if (bus && ownerSensor)
    {
       int fakeId = 0;
       int boardId = ownerSensor->getId();
       short int val[6] = {0,0,0,0,0,0};
       static double curr_time = Time::now();
       for (i=0; i<6; i++)
       {
           double fullScale = ownerSensor->getScaleFactor()[i];
           if (dval[i] >  fullScale) 
           {
               if (Time::now() - curr_time > 2)
                    {
                        yWarning("PORT: %s **** SATURATED CH:%d : %+4.4f COUNT: %d \n", this->getName().c_str(), i, dval[i], count_saturation);
                        curr_time = Time::now();
                    }
               dval[i] =  fullScale;
               count_saturation++;
           }
           else if (dval[i] < -fullScale)
           {
               if (Time::now() - curr_time > 2)
                    {
                        yWarning("PORT: %s **** SATURATED CH:%d : %+4.4f COUNT: %d \n", this->getName().c_str(), i, dval[i], count_saturation);
                        curr_time = Time::now();
                    }
               dval[i] = -fullScale;
               count_saturation++;
           }
           val[i] = (short int)(dval[i] / fullScale * 0x7fff)+0x8000; //check this!
       }

       bus->startPacket();
       fakeId = 0x300 + (boardId<<4)+ 0x0A;
       bus->_writeBuffer[0].setId(fakeId);
       bus->_writeBuffer[0].getData()[1]=(val[0] >> 8) & 0xFF;
       bus->_writeBuffer[0].getData()[0]= val[0] & 0xFF;
       bus->_writeBuffer[0].getData()[3]=(val[1] >> 8) & 0xFF;
       bus->_writeBuffer[0].getData()[2]= val[1] & 0xFF;
       bus->_writeBuffer[0].getData()[5]=(val[2] >> 8) & 0xFF;
       bus->_writeBuffer[0].getData()[4]= val[2] & 0xFF;
       bus->_writeBuffer[0].setLen(6);
       bus->_writeMessages++;
       bus->writePacket();

       if (canEchoEnabled)
       {
           if (bus->_readMessages<BUF_SIZE-1 && bus->_echoMessages<BUF_SIZE-1)
           {
               bus->_echoBuffer[bus->_echoMessages].setId(fakeId);
               bus->_echoBuffer[bus->_echoMessages].setLen(6);
               for (i=0; i<6; i++)
                    bus->_echoBuffer[bus->_echoMessages].getData()[i]=bus->_writeBuffer[0].getData()[i];
               bus->_echoMessages++;
           }
           else
           {
               yError("Echobuffer full \n");
           }
       }

       bus->startPacket();
       fakeId = 0x300 + (boardId<<4)+ 0x0B;
       bus->_writeBuffer[0].setId(fakeId);
       bus->_writeBuffer[0].getData()[1]=(val[3] >> 8) & 0xFF;
       bus->_writeBuffer[0].getData()[0]= val[3] & 0xFF;
       bus->_writeBuffer[0].getData()[3]=(val[4] >> 8) & 0xFF;
       bus->_writeBuffer[0].getData()[2]= val[4] & 0xFF;
       bus->_writeBuffer[0].getData()[5]=(val[5] >> 8) & 0xFF;
       bus->_writeBuffer[0].getData()[4]= val[5] & 0xFF;
       bus->_writeBuffer[0].setLen(6);
       bus->_writeMessages++;
       bus->writePacket();
       
       if (canEchoEnabled)
       {
           if (bus->_readMessages<BUF_SIZE-1 && bus->_echoMessages<BUF_SIZE-1)
           {
               bus->_echoBuffer[bus->_echoMessages].setId(fakeId);
               bus->_echoBuffer[bus->_echoMessages].setLen(6);
               for (i=0; i<6; i++)
                    bus->_echoBuffer[bus->_echoMessages].getData()[i]=bus->_writeBuffer[0].getData()[i];
               bus->_echoMessages++;
           }
           else
           {
               yError("Echobuffer full \n");
           }
       }

    }
    semaphore->post();
}

speedEstimationHelper::speedEstimationHelper(int njoints, SpeedEstimationParameters* estim_parameters )
{
    jointsNum=njoints;
    estim_params = new SpeedEstimationParameters [jointsNum];
    if (estim_parameters!=0)
        memcpy(estim_params, estim_parameters, sizeof(SpeedEstimationParameters)*jointsNum);
    else
        memset(estim_params, 0, sizeof(SpeedEstimationParameters)*jointsNum);
}

axisImpedanceHelper::axisImpedanceHelper(int njoints, ImpedanceLimits* imped_limits)
{
    jointsNum=njoints;
    impLimits = new ImpedanceLimits [jointsNum];
    if (impLimits != 0)
        memcpy(impLimits, imped_limits, sizeof(ImpedanceLimits)*jointsNum);
    else
        memset(impLimits, 0, sizeof(ImpedanceLimits)*jointsNum);
}

axisPositionDirectHelper::axisPositionDirectHelper(int njoints, const int *aMap, const double *angToEncs, double* _stepLimit)
{
    jointsNum=njoints;
    helper = new ControlBoardHelper(njoints, aMap, angToEncs, 0, 0); //NB: zeros=0 is mandatory for this algorithm
    maxHwStep = new double [jointsNum];
    maxUserStep = new double [jointsNum];
    for (int i=0; i<jointsNum; i++)
    {
        int    hw_i=0;
        double hw_ang=0;
        helper->posA2E(_stepLimit[i], i, hw_ang, hw_i);
        maxHwStep[hw_i] = fabs(hw_ang); //NB: fabs() is mandatory for this algorithm (because angToEncs is signed)
        maxUserStep[i] = _stepLimit[i];
    }
}

double axisPositionDirectHelper::getSaturatedValue (int hw_j, double hw_curr_value, double hw_ref_value)
{
    //here hw_j isthe joint number after the axis_map; hw_value is in encoder ticks
    double hw_maxval = hw_curr_value + getMaxHwStep(hw_j);
    double hw_minval = hw_curr_value - getMaxHwStep(hw_j);
    double hw_val    = (std::min)((std::max)(hw_ref_value, hw_minval), hw_maxval);

    return hw_val;
}

CanBusMotionControl::torqueControlHelper::torqueControlHelper(int njoints, double* p_angleToEncoders, double* p_newtonsTosens )
{
   jointsNum=njoints;
   newtonsToSensor = new double  [jointsNum];
   angleToEncoders = new double  [jointsNum];

   if (p_angleToEncoders!=0)
       memcpy(angleToEncoders, p_angleToEncoders, sizeof(double)*jointsNum);
   else
       for (int i=0; i<jointsNum; i++) {angleToEncoders[i]=1.0;}

   if (p_newtonsTosens!=0)
       memcpy(newtonsToSensor, p_newtonsTosens, sizeof(double)*jointsNum);
   else
       for (int i=0; i<jointsNum; i++) {newtonsToSensor[i]=1.0;}
}

axisTorqueHelper::axisTorqueHelper(int njoints, int* id, int* chan, double* maxTrq, double* newtons2sens )
{
    jointsNum=njoints;
    torqueSensorId = new int [jointsNum];
    torqueSensorChan = new int [jointsNum];
    maximumTorque = new double [jointsNum];
    newtonsToSensor = new double [jointsNum];
    if (id!=0)
        memcpy(torqueSensorId, id, sizeof(int)*jointsNum);
    else
        memset(torqueSensorId, 0, sizeof(int)*jointsNum);
    if (chan!=0)
        memcpy(torqueSensorChan, chan, sizeof(int)*jointsNum);
    else
        memset(torqueSensorChan, 0, sizeof(int)*jointsNum);
    if (maxTrq!=0)
        memcpy(maximumTorque, maxTrq, sizeof(double)*jointsNum);
    else
        memset(maximumTorque, 0, sizeof(double)*jointsNum);
    if (newtons2sens!=0)
        memcpy(newtonsToSensor, newtons2sens, sizeof(double)*jointsNum);
    else
        memset(newtonsToSensor, 0, sizeof(double)*jointsNum);

}

TBR_AnalogSensor::TBR_AnalogSensor():
data(0)
{
    isVirtualSensor=false;
    timeStamp=0;
    status=IAnalogSensor::AS_OK;
    useCalibration=0;
    scaleFactor=0;

    counterSat=0;
    counterError=0;
    counterTimeout=0;
    backDoor=0;
}

TBR_AnalogSensor::~TBR_AnalogSensor()
{
    if (!data)
        delete data;
    if (!scaleFactor)
        delete scaleFactor;
}

int TBR_AnalogSensor::getState(int ch)
{
    return status;
}

bool TBR_AnalogSensor::open(int channels, AnalogDataFormat f, short bId, short useCalib, bool isVirtualS)
{
    if (data)
        return false;
    if (scaleFactor)
        return false;

    data=new TBR_AnalogData(channels, channels+1);
    scaleFactor=new double[channels];
    int i=0;
    for (i=0; i<channels; i++) scaleFactor[i]=1;
    dataFormat=f;
    boardId=bId;
    useCalibration=useCalib;
    isVirtualSensor=isVirtualS;
    if (useCalibration==1 && dataFormat==TBR_AnalogSensor::ANALOG_FORMAT_16)
    {
        scaleFactor[0]=1;
        scaleFactor[1]=1;
        scaleFactor[2]=1;
        scaleFactor[3]=1;
        scaleFactor[4]=1;
        scaleFactor[5]=1;
    }

    return true;
}


int TBR_AnalogSensor::getChannels()
{
    return data->size();
}

int TBR_AnalogSensor::read(yarp::sig::Vector &out)
{
    // print errors
    mutex.wait();

    if (!data)
    {
        mutex.post();
        return false;
    }

     if (status!=IAnalogSensor::AS_OK) 
    {
        switch (status)
        {
            case IAnalogSensor::AS_OVF:
                {
                    counterSat++;
                }
                break;
            case IAnalogSensor::AS_ERROR:
                {
                    counterError++;
                }
                break;
            case IAnalogSensor::AS_TIMEOUT:
                {
                   counterTimeout++;
                }
                break;
            default:
            {
                counterError++;
            }
        }
        mutex.post();
        return status;
    }

    out.resize(data->size());
    for(int k=0;k<data->size();k++)
    {
        out[k]=(*data)[k];
    }
    
    mutex.post();
    return status;
}
 
int TBR_AnalogSensor::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

int TBR_AnalogSensor::calibrateSensor()
{
    return AS_OK;
}

bool TBR_AnalogSensor::decode16(const unsigned char *msg, int id, double *data)
{
    
    const char groupId=(id&0x00f);
    int baseIndex=0;
    {
        switch (groupId)
        {
        case 0xA:
            {
                for(int k=0;k<3;k++)
                    {
                        data[k]=(((unsigned short)(msg[2*k+1]))<<8)+msg[2*k]-0x8000;
                        if (useCalibration==1)
                        {
                            data[k]=data[k]*scaleFactor[k]/float(0x8000);
                        }
                    }
            }
            break;
        case 0xB:
            {
                for(int k=0;k<3;k++)
                    {
                        data[k+3]=(((unsigned short)(msg[2*k+1]))<<8)+msg[2*k]-0x8000;
                        if (useCalibration==1)
                        {
                            data[k+3]=data[k+3]*scaleFactor[k+3]/float(0x8000);
                        }
                    }
            }
            break;
        case 0xC:
            {} //skip these, they are not for us
            break;
        case 0xD:
            {} //skip these, they are not for us
            break;
        default:
            yError("Got unexpected class 0x3 msg(s)\n");
            return false;
            break;
        }
        //@@@DEBUG ONLY
        //yDebug("   %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    }

    return true;
}

bool TBR_AnalogSensor::decode8(const unsigned char *msg, int id, double *data)
{
    const char groupId=(id&0x00f);
    int baseIndex=0;
    {
        switch (groupId)
        {
        case 0xC:
            {
                for(int k=0;k<=6;k++)
                    data[k]=msg[k];
            }
            break;
        case 0xD:
            {
                for(int k=0;k<=7;k++)
                    data[7+k]=msg[k];
            }
            break;
        case 0xA:
            {} //skip these, they are not for us
            break;
        case 0xB:
            {} //skip these, they are not for us
            break;
        default:
            yWarning("Got unexpected class 0x3 msg(s): groupId 0x%x\n", groupId);
            return false;
            break;
        }
    }
    return true;
}


bool TBR_AnalogSensor::handleAnalog(void *canbus)
{
    CanBusResources& r = RES (canbus);

    unsigned int i=0;
    const int _networkN=r._networkN;

    bool ret=true; //return true by default

    mutex.wait();

    double timeNow=Time::now();
    for (unsigned int buff_num=0; buff_num<2; buff_num++)
    {
        unsigned int size = 0;
        CanBuffer* buffer_pointer=0;
        if (buff_num==0)
        {
            size = r._readMessages;
            buffer_pointer = &r._readBuffer;
        }
        else
        {
            size = r._echoMessages;
            buffer_pointer = &r._echoBuffer;
        }

        for (i = 0; i < size; i++)
        {
            unsigned int len=0;
            unsigned int msgid=0;
            unsigned char *buff=0;
            CanMessage& m = (*buffer_pointer)[i];
            buff=m.getData();
            msgid=m.getId();
            len=m.getLen();
        
            status=IAnalogSensor::AS_OK;
            const char type=((msgid&0x700)>>8);
            const char id=((msgid&0x0f0)>>4);

            if (type==0x03) //analog data
                {
                    if (id==boardId)
                    {
                        timeStamp=Time::now();
                        switch (dataFormat)
                        {
                            case ANALOG_FORMAT_8:
                                ret=decode8(buff, msgid, data->getBuffer());
                                status=IAnalogSensor::AS_OK;
                                break;
                            case ANALOG_FORMAT_16:
                                if (len==6) 
                                {
                                    ret=decode16(buff, msgid, data->getBuffer());
                                    status=IAnalogSensor::AS_OK;
                                }
                                else
                                {
                                    if (len==7 && buff[6] == 1)
                                    {
                                        status=IAnalogSensor::AS_OVF;
                                    }
                                    else
                                    {
                                        status=IAnalogSensor::AS_ERROR;
                                    }
                                    ret=decode16(buff, msgid, data->getBuffer());
                                }
                                break;
                            default:
                                ret=false;
                        }
                    }
                }
        }
    }

    //if 100ms have passed since the last received message
    if (timeStamp+0.1<timeNow)
        {
            status=IAnalogSensor::AS_TIMEOUT;
        }

    mutex.post();
    return ret;
}


bool CanBusMotionControlParameters:: setBroadCastMask(Bottle &list, int MASK)
{
    if (list.size()==2)
    {
        if (list.get(1).asInt()==1)
        {
            for(int j=0;j<_njoints;j++)
                _broadcast_mask[j]|=(1<<(MASK-1));
        }
        return true;
    }

    if (list.size()==(_njoints+1))
    {
        for(int k=0;k<_njoints;k++)
        {
            if ((list.get(k+1).asInt())==1)
                {
                    int tmp=_axisMap[k];//remap
                    _broadcast_mask[tmp]|=(1<<(MASK-1));
                }
        }
        return true;
    }

    return false;
}

bool CanBusMotionControlParameters::parsePosPidsGroup_OldFormat(Bottle& pidsGroup, int nj, Pid myPid[])
{
    int j=0;
    for(j=0;j<nj;j++)
    {
        char tmp[80];
        sprintf(tmp, "Pid%d", j); 

        Bottle &xtmp = pidsGroup.findGroup(tmp);
        myPid[j].kp = xtmp.get(1).asDouble();
        myPid[j].kd = xtmp.get(2).asDouble();
        myPid[j].ki = xtmp.get(3).asDouble();

        myPid[j].max_int = xtmp.get(4).asDouble();
        myPid[j].max_output = xtmp.get(5).asDouble();

        myPid[j].scale = xtmp.get(6).asDouble();
        myPid[j].offset = xtmp.get(7).asDouble();

        if (xtmp.size()==10)
        {
            myPid[j].stiction_up_val = xtmp.get(8).asDouble();
            myPid[j].stiction_down_val = xtmp.get(9).asDouble();
        }
    }
    return true;
}

bool CanBusMotionControlParameters::parseTrqPidsGroup_OldFormat(Bottle& pidsGroup, int nj, Pid myPid[])
{
    int j=0;
    for(j=0;j<nj;j++)
    {
        char tmp[80];
        sprintf(tmp, "TPid%d", j); 

        Bottle &xtmp = pidsGroup.findGroup(tmp);
        myPid[j].kp = xtmp.get(1).asDouble();
        myPid[j].kd = xtmp.get(2).asDouble();
        myPid[j].ki = xtmp.get(3).asDouble();

        myPid[j].max_int = xtmp.get(4).asDouble();
        myPid[j].max_output = xtmp.get(5).asDouble();

        myPid[j].scale = xtmp.get(6).asDouble();
        myPid[j].offset = xtmp.get(7).asDouble();

        if (xtmp.size()==10)
        {
            myPid[j].stiction_up_val = xtmp.get(8).asDouble();
            myPid[j].stiction_down_val = xtmp.get(9).asDouble();
        }
    }
    return true;
}
bool CanBusMotionControlParameters::parsePidsGroup_NewFormat(Bottle& pidsGroup, Pid myPid[])
{
    int j=0;
    Bottle xtmp;

    if (!validate(pidsGroup, xtmp, "kp", "Pid kp parameter", _njoints+1))           return false; for (j=0; j<_njoints; j++) myPid[j].kp = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "kd", "Pid kd parameter", _njoints+1))           return false; for (j=0; j<_njoints; j++) myPid[j].kd = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "ki", "Pid kp parameter", _njoints+1))           return false; for (j=0; j<_njoints; j++) myPid[j].ki = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "maxInt", "Pid maxInt parameter", _njoints+1))   return false; for (j=0; j<_njoints; j++) myPid[j].max_int = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "maxOutput", "Pid maxOutput parameter", _njoints+1))   return false; for (j=0; j<_njoints; j++) myPid[j].max_output = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "shift", "Pid shift parameter", _njoints+1))     return false; for (j=0; j<_njoints; j++) myPid[j].scale = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "ko", "Pid ko parameter", _njoints+1))           return false; for (j=0; j<_njoints; j++) myPid[j].offset = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "stictionUp", "Pid stictionUp", _njoints+1))     return false; for (j=0; j<_njoints; j++) myPid[j].stiction_up_val = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "stictionDwn", "Pid stictionDwn", _njoints+1))   return false; for (j=0; j<_njoints; j++) myPid[j].stiction_down_val = xtmp.get(j+1).asDouble();

    //optional kff
    xtmp = pidsGroup.findGroup("kff");         
    if (!xtmp.isNull())
    {
        for (j=0; j<_njoints; j++) myPid[j].kff = xtmp.get(j+1).asDouble();
    }
    else
    {
         for (j=0; j<_njoints; j++) myPid[j].kff = 0;
    }

    return true;
}

bool CanBusMotionControlParameters::parseImpedanceGroup_NewFormat(Bottle& pidsGroup, ImpedanceParameters vals[])
{
    int j=0;
    Bottle xtmp;
    if (!validate(pidsGroup, xtmp, "stiffness", "Pid stiffness parameter", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].stiffness = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "damping", "Pid damping parameter", _njoints+1))           return false; for (j=0; j<_njoints; j++) vals[j].damping   = xtmp.get(j+1).asDouble();
    return true;
}

bool CanBusMotionControlParameters::parseDebugGroup_NewFormat(Bottle& pidsGroup, DebugParameters vals[])
{
    int j=0;
    Bottle xtmp;

    if (!validate(pidsGroup, xtmp, "debug0", "debug0", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[0] = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "debug1", "debug1", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[1] = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "debug2", "debug2", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[2] = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "debug3", "debug3", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[3] = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "debug4", "debug4", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[4] = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "debug5", "debug5", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[5] = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "debug6", "debug6", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[6] = xtmp.get(j+1).asDouble();
    if (!validate(pidsGroup, xtmp, "debug7", "debug7", _njoints+1))       return false; for (j=0; j<_njoints; j++) vals[j].data[7] = xtmp.get(j+1).asDouble();

    return true;
}
bool CanBusMotionControlParameters::fromConfig(yarp::os::Searchable &p)
{
    if (!p.check("GENERAL","section for general motor control parameters")) {
        yError("Cannot understand configuration parameters\n");
        return false;
    }

    std::string dbg_string = p.toString().c_str();
    int i;
    int nj = p.findGroup("GENERAL").check("Joints",Value(1),
        "Number of degrees of freedom").asInt();
    alloc(nj);

    // Check useRawEncoderData = do not use calibration data!
    bool useRawJointEncoderData = false;
    bool useRawMotorEncoderData = false;
    Value use_jnt_raw = p.findGroup("GENERAL").find("useRawEncoderData");
    Value use_mot_raw = p.findGroup("GENERAL").find("useRawMotorEncoderData");

    if(use_jnt_raw.isNull())
    {
        useRawJointEncoderData = false;
    }
    else
    {
        if(!use_jnt_raw.isBool())
        {
            yError("useRawEncoderData bool param is different from accepted values (true / false). Assuming false\n");
            useRawJointEncoderData = false;
        }
        else
        {
            useRawJointEncoderData = use_jnt_raw.asBool();
            if(useRawJointEncoderData)
            {
                yWarning("canBusMotionControl using raw data from encoders! Be careful  See 'useRawEncoderData' param in config file \n");
                yWarning("DO NOT USE OR CALIBRATE THE ROBOT IN THIS CONFIGURATION! \n");
                yWarning("CHECK IF THE FAULT BUTTON IS PRESSED and press ENTER to continue \n");
                getchar(); 
            }
        }
    }

    if(use_mot_raw.isNull())
    {
        useRawMotorEncoderData = false;
    }
    else
    {
        if(!use_mot_raw.isBool())
        {
            yError("useRawEncoderData bool param is different from accepted values (true / false). Assuming false\n");
            useRawMotorEncoderData = false;
        }
        else
        {
            useRawMotorEncoderData = use_mot_raw.asBool();
            if(useRawMotorEncoderData)
            {
                yWarning("canBusMotionControl using raw data from  motor encoders");
            }
        }
    }
    // Check useRawEncoderData = do not use calibration data!
    Value use_limitedPWM = p.findGroup("GENERAL").find("useLimitedPWM");
    if(use_limitedPWM.isNull())
    {
        _pwmIsLimited = false;
    }
    else
    {
        if(!use_limitedPWM.isBool())
        {
            _pwmIsLimited = false;
        }
        else
        {
            _pwmIsLimited = use_limitedPWM.asBool();
        }
    }

    ///// CAN PARAMETERS
    Bottle& canGroup = p.findGroup("CAN");

    _networkName=canGroup.find("NetworkId").asString();

    if (canGroup.check("CanForcedDeviceNum"))
    {
        _networkN=canGroup.find("CanForcedDeviceNum").asInt();
    }
    else
        _networkN=canGroup.check("CanDeviceNum",Value(-1), "numeric identifier of device").asInt();

    //    yDebug<<can.toString();
    if (_networkN<0)
    {
        yError("can network id not valid, skipping device\n");
        return false;
    }

    _my_address=canGroup.check("CanMyAddress",Value(0),
                                 "numeric identifier of my address").asInt();

    _polling_interval=canGroup.check("CanPollingInterval",Value(20),
                                        "polling period").asInt();
    
    _timeout=canGroup.check("CanTimeout",Value(20),"timeout period").asInt();

    _txTimeout=canGroup.check("CanTxTimeout", Value(20), "tx timeout").asInt();
    _rxTimeout=canGroup.check("CanRxTimeout", Value(20), "rx timeout").asInt();

    // default values for CanTxQueueSize/CanRxQueueSize should be the 
    // maximum, difficult to pick a correct value, let the driver 
    // decide on this
    if (canGroup.check("CanTxQueueSize"))
        _txQueueSize=canGroup.find("CanTxQueueSize").asInt();
    else
        _txQueueSize=-1;

    if (canGroup.check("CanRxQueueSize"))
        _rxQueueSize=canGroup.find("CanRxQueueSize").asInt();
    else
        _rxQueueSize=-1;

    Bottle &canAddresses = canGroup.findGroup("CanAddresses",
                                         "a list of numeric identifiers");
    if (canAddresses.isNull())
    {
        yError("CanAddresses group not found in config file\n");
        return false;
    }
    for (i = 1; i < canAddresses.size(); i++) {
        _destinations[i-1] = (unsigned char)(canAddresses.get(i).asInt());
    }

    ////// GENERAL
    Bottle& general = p.findGroup("GENERAL");

    Bottle xtmp;
    if (!validate(general, xtmp, "AxisMap", "a list of reordered indices for the axes", nj+1))
        return false;
       
    for (i = 1; i < xtmp.size(); i++)
        _axisMap[i-1] = xtmp.get(i).asInt();
    
    if (!validate(general, xtmp, "AxisName", "a list of strings representing the axes names", nj + 1))
    { 
        //return false; //this parameter is not yet mandatory
    }
    //beware: axis name has to be remapped here because they are not set using the toHw() helper function  
    for (i = 1; i < xtmp.size(); i++)
        _axisName[_axisMap[i-1]] = xtmp.get(i).asString();

    if (!validate(general, xtmp, "AxisType", "a list of strings representing the axes types (revolute/prismatic)", nj + 1))
    {
        //return false; //this parameter is not yet mandatory
    }
    //beware: axis name has to be remapped here because they are not set using the toHw() helper function  
    for (i = 1; i < xtmp.size(); i++)
        _axisType[_axisMap[i - 1]] = xtmp.get(i).asString();

    if (!validate(general, xtmp, "Encoder", "a list of scales for the joint encoders", nj+1))
        return false;

    int test = xtmp.size();
    for (i = 1; i < xtmp.size(); i++) 
        _angleToEncoder[i-1] = xtmp.get(i).asDouble();

    if (!validate_optional(general, xtmp, "Rotor", "a list of scales for the rotor encoders", nj+1))
        {
            yWarning("Rotor: Using default value = 1\n");
            for(i=1;i<nj+1; i++) 
                _rotToEncoder[i-1] = 1.0;
        }
    else
        {
            int test = xtmp.size();
            for (i = 1; i < xtmp.size(); i++) 
                _rotToEncoder[i-1] = xtmp.get(i).asDouble();
        }

    if (!validate(general, xtmp, "fullscalePWM", " list of scales for the fullscalePWM conversion factor", nj + 1))
    {
        yError("fullscalePWM param not found in config file. Please update robot configuration files or contact https://github.com/robotology/icub-support");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++)
        _dutycycleToPwm[i - 1] = xtmp.get(i).asDouble()/100.0;

    if (!validate(general, xtmp, "ampsToSensor", "a list of scales for the ampsToSensor conversion factor", nj + 1))
    {
        yError("ampsToSensor param not found in config file. Please update robot configuration files or contact https://github.com/robotology/icub-support");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++)
        _ampsToSensor[i - 1] = xtmp.get(i).asDouble();

    if (!validate(general, xtmp, "Zeros","a list of offsets for the zero point", nj+1))
        return false;

    for (i = 1; i < xtmp.size(); i++) 
        _zeros[i-1] = xtmp.get(i).asDouble();

    if (useRawJointEncoderData)
    {
        for(i=1;i<nj+1; i++)
        {
            if (_angleToEncoder[i-1] > 0)
                _angleToEncoder[i-1] = 1;
            else
                _angleToEncoder[i-1] = -1;

            _zeros[i-1] = 0;
        }
    }

    if (useRawMotorEncoderData)
    {
        for(i=1;i<nj+1; i++)
        {
            if (_rotToEncoder[i-1] > 0)
                _rotToEncoder[i-1] = 1;
            else
                _rotToEncoder[i-1] = -1;
        }
    }

    if (!validate_optional(general, xtmp, "TorqueId","a list of associated joint torque sensor ids", nj+1))
    {
        yWarning("TorqueId: Using default value = 0 (disabled)\n");
        for(i=1;i<nj+1; i++) 
            _torqueSensorId[i-1] = 0;   
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorId[i-1] = xtmp.get(i).asInt();
    }
    
    if (!validate_optional(general, xtmp, "TorqueChan","a list of associated joint torque sensor channels", nj+1))
    {
        yWarning("TorqueChan: Using default value = 0 (disabled)\n");
        for(i=1;i<nj+1; i++) 
            _torqueSensorChan[i-1] = 0;   
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) _torqueSensorChan[i-1] = xtmp.get(i).asInt();
    }

    if (!validate(general, xtmp, "TorqueMax","full scale value for a joint torque sensor", nj+1))
    {
        return false;
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++) 
        {
                _maxTorque[i-1] = xtmp.get(i).asInt();
                _newtonsToSensor[i-1] = double(0x8000)/double(_maxTorque[i-1]);
        }
    }

    int j=0;
    ////// POSITION PIDS
    {
        Bottle posPidsGroup;
        posPidsGroup=p.findGroup("POS_PIDS", "Position Pid parameters new format");
        if (posPidsGroup.isNull() == false)
        {
           yInfo("Position Pids section found, new format\n");
           if (!parsePidsGroup_NewFormat(posPidsGroup, _pids))
           {
               yError() << "Position Pids section: error detected in parameters syntax";
               return false;
           }
           else
           {
               yInfo("Position Pids successfully loaded\n");
           }
        }
        else
        {
            Bottle posPidsGroup2=p.findGroup("PIDS", "Position Pid parameters old format");
            if (posPidsGroup2.isNull()==false)
            {
                yWarning("Position Pids section found, old format\n");
                parsePosPidsGroup_OldFormat (posPidsGroup2, nj, _pids);
            }
            else
            {   
                yError () << "no PIDS group found in config file, returning";
                return false;
            }
        }
    }
    
    ////// TORQUE PIDS
    {
        Bottle trqPidsGroup;
        Bottle trqPidsOldGroup;
        Bottle trqControlGroup;
        trqPidsGroup    = p.findGroup("TRQ_PIDS",       "Torque Pid parameters new format");
        trqPidsOldGroup = p.findGroup("TORQUE_PIDS",    "Torque Pid parameters old format");
        trqControlGroup = p.findGroup("TORQUE_CONTROL", "Torque Control parameters");

        if (trqPidsGroup.isNull()==false)
        {
           yInfo("TRQ_PIDS: Torque Pids section found, new format\n");
           if (!parsePidsGroup_NewFormat (trqPidsGroup, _tpids))
           {
               yError () << "TRQ_PIDS: error detected in parameters syntax";
               return false;
           }
           else
           {
                xtmp = trqPidsGroup.findGroup("kbemf"); 
                {for (j=0;j<nj;j++) this->_bemfGain[j] = 0; }

                xtmp = trqPidsGroup.findGroup("ktau"); 
                {for (j=0;j<nj;j++) this->_ktau[j] = 1; }
                
                xtmp = trqPidsGroup.findGroup("filterType"); 
                {for (j=0;j<nj;j++) this->_filterType[j] = 3; }
                
                xtmp = trqPidsGroup.findGroup("controlUnits"); 
                this->_torqueControlUnits = MACHINE_UNITS;
                
                _torqueControlEnabled = true;
           }
        }
        else if (trqPidsOldGroup.isNull()==false)
        {
            yWarning ("TORQUE_PIDS: Torque Pids section found, old format\n");
            if (!parseTrqPidsGroup_OldFormat (trqPidsOldGroup, nj, _tpids))
            {
                yError () << "TORQUE_PIDS: error detected in parameters syntax";
                return false;
            }
            else
            {
                xtmp = trqPidsGroup.findGroup("kbemf"); 
                {for (j=0;j<nj;j++) this->_bemfGain[j] = 0; }

                xtmp = trqPidsGroup.findGroup("ktau"); 
                {for (j=0;j<nj;j++) this->_ktau[j] = 1; }
            
                xtmp = trqPidsGroup.findGroup("filterType"); 
                {for (j=0;j<nj;j++) this->_filterType[j] = 3; }
            
                xtmp = trqPidsGroup.findGroup("controlUnits"); 
                this->_torqueControlUnits = MACHINE_UNITS;
            
                _torqueControlEnabled = true;
            }
        }
        else if (trqControlGroup.isNull()==false)
        {
           yInfo("TORQUE_CONTROL section found\n");
           if (!parsePidsGroup_NewFormat (trqControlGroup, _tpids))
           {
               yError () << "TORQUE_CONTROL: error detected in parameters syntax";
               return false;
           }
           else
           {
                xtmp = trqControlGroup.findGroup("kbemf"); 
                if (!xtmp.isNull())
                {for (j=0;j<nj;j++) this->_bemfGain[j] = xtmp.get(j+1).asDouble();}
                else
                {for (j=0;j<nj;j++) this->_bemfGain[j] = 0; yWarning ("TORQUE_PIDS: 'kbemf' param missing");}

                xtmp = trqControlGroup.findGroup("ktau"); 
                if (!xtmp.isNull())
                {for (j=0;j<nj;j++) this->_ktau[j] = xtmp.get(j+1).asDouble();}
                else
                {for (j=0;j<nj;j++) this->_ktau[j] = 1.0; yWarning ("TORQUE_PIDS: 'ktau' param missing");}
                
                xtmp = trqControlGroup.findGroup("filterType"); 
                if (!xtmp.isNull())
                {for (j=0;j<nj;j++) this->_filterType[j] = xtmp.get(j+1).asInt();}
                else
                {for (j=0;j<nj;j++) this->_filterType[j] = 3; yWarning ("TORQUE_PIDS: 'filterType' param missing");}
                
                xtmp = trqControlGroup.findGroup("controlUnits"); 
                if (!xtmp.isNull())
                {    
                     if      (xtmp.toString()==std::string("metric_units"))  {this->_torqueControlUnits=METRIC_UNITS;}
                     else if (xtmp.toString()==std::string("machine_units")) {this->_torqueControlUnits=MACHINE_UNITS;}
                     else    {yError() << "invalid controlUnits value"; return false;}
                }
                else
                {
                     yError ("TORQUE_PIDS: 'controlUnits' param missing. Cannot continue");
                     return false;
                }
                
                _torqueControlEnabled = true;
           }
        }
        else
        {   
            _torqueControlEnabled=false;
            yWarning("Torque control parameters not found for part %s, skipping...\n", _networkName.c_str());
        }
    }
    
    ////// DEBUG PARAMETERS
    {
        Bottle &debugGroup=p.findGroup("DEBUG_PARAMETERS","DEBUG parameters");
        if (debugGroup.isNull()==false)
        {
           yDebug("DEBUG_PARAMETERS section found\n");
           if (!parseDebugGroup_NewFormat (debugGroup, _debug_params))
           {
               yError("DEBUG_PARAMETERS section: error detected in parameters syntax\n");
               return false;
           }
           else
           {
               yInfo("DEBUG_PARAMETERS section: parameters successfully loaded\n");
           }
        }
        else
        {
           yDebug("DEBUG parameters section NOT found, skipping...\n");
        }
    }

    ////// IMPEDANCE DEFAULT VALUES
    {
        Bottle &impedanceGroup=p.findGroup("IMPEDANCE","IMPEDANCE parameters");
        if (impedanceGroup.isNull()==false)
        {
           yDebug("IMPEDANCE parameters section found\n");
           if (!parseImpedanceGroup_NewFormat (impedanceGroup, _impedance_params))
           {
               yError("IMPEDANCE section: error detected in parameters syntax\n");
               return false;
           }
           else
           {
               yInfo("IMPEDANCE section: parameters successfully loaded\n");
           }
        }
        else
        {
           yDebug("IMPEDANCE parameters section NOT found, skipping...\n");
        }
    }

    ////// IMPEDANCE LIMITS (UNDER TESTING)
    for(j=0;j<nj;j++)
    {
        _impedance_limits[j].min_damp=  0.001;
        _impedance_limits[j].max_damp=  9.888;
        _impedance_limits[j].min_stiff= 0.002;
        _impedance_limits[j].max_stiff= 9.889;
        _impedance_limits[j].param_a=   0.011;
        _impedance_limits[j].param_b=   0.012;
        _impedance_limits[j].param_c=   0.013;
    }

    /////// LIMITS
    Bottle &limits=p.findGroup("LIMITS");
    if (limits.isNull())
    {
        yError("Group LIMITS not found in configuration file\n");
        return false;
    }

    // motor oveload current
    if (!validate(limits, xtmp, "motorOverloadCurrents","a list of current limits", nj+1))
        return false;
    for(i=1;i<xtmp.size(); i++) _currentLimits[i-1]=xtmp.get(i).asDouble();

    // Motor pwm limit
    if (!validate(limits, xtmp, "motorPwmLimit", "a list of motor PWM limits", nj + 1))
    {
        yWarning("motorPwmLimit: Using default motorPwmLimit=1333\n");
        for (i = 1; i<nj + 1; i++)
            _motorPwmLimits[i - 1] = 1333;   //Default value
    }
    else
    {
        for (i = 1; i < xtmp.size(); i++)
        {
            _motorPwmLimits[i - 1] = xtmp.get(i).asDouble();
            if (_motorPwmLimits[i - 1] < 0)
            {
                yError() << "motorPwmLimit should be a positive value";
                return false;
            }
        }
    }

    // Max Joint velocity
    if (!validate_optional(limits, xtmp, "jntVelMax", "the maximum velocity command of a joint", nj + 1))
    {
        yWarning("jntVelMax: Using default jntVelMax=100 deg/s\n");
        for (i = 1; i<nj + 1; i++)
            _maxJntCmdVelocity[i - 1] = 100;   //Default value
    }
    else
    {
        for (i = 1; i<xtmp.size(); i++)
        {
            _maxJntCmdVelocity[i - 1] = xtmp.get(i).asDouble();
            if (_maxJntCmdVelocity[i - 1]<0)
            {
                yError() << "Invalid jntVelMax parameter <0\n";
                return false;
            }
        }
    }

    if (!validate_optional(limits, xtmp, "maxPosStep", "the maximum amplitude of a position direct step", nj+1))
    {
        yWarning("maxPosStep: Using default MaxPosStep=10 degs\n");
        for(i=1;i<nj+1; i++)
            _maxStep[i-1] = 10;   //Default value
    }
    else
    {
        for(i=1;i<xtmp.size(); i++) 
        {
            _maxStep[i-1]=xtmp.get(i).asDouble();
            if (_maxStep[i-1]<0)
            {
                yError () << "Invalid MaxPosStep parameter <0\n";
                return false;
            }
        }
    }

    // max joint position
    if (!validate(limits, xtmp, "jntPosMax","a list of maximum angles (in degrees)", nj+1))
        return false;
    for(i=1;i<xtmp.size(); i++) _limitsMax[i-1]=xtmp.get(i).asDouble();

    // min joint position
    if (!validate(limits, xtmp, "jntPosMin","a list of minimum angles (in degrees)", nj+1))
        return false;
    for(i=1;i<xtmp.size(); i++) _limitsMin[i-1]=xtmp.get(i).asDouble();
    for (i = 0; i<nj; i++)
    {
        if (_limitsMax[i] < _limitsMin[i])
        {
            yError("invalid limit on joint %d : Max value (%f) < Min value(%f)\n", i, _limitsMax[i], _limitsMin[i]);
            return false;
        }
    }

    /////// [VELOCITY]
    Bottle &velocityGroup=p.findGroup("VELOCITY");
    if (!velocityGroup.isNull())
        {
            /////// Shifts
            if (!validate(velocityGroup, xtmp, "Shifts", "a list of shifts to be used in the vmo control", nj+1))
            {
                yWarning("Shifts: Using default Shifts=4\n");
                for(i=1;i<nj+1; i++)
                    _velocityShifts[i-1] = 4;   //Default value
            }
            else
            {
                for(i=1;i<xtmp.size(); i++) 
                    _velocityShifts[i-1]=xtmp.get(i).asInt();
            }

            /////// Timeout
            xtmp.clear();
            if (!validate(velocityGroup, xtmp, "Timeout", "a list of timeout to be used in the vmo control", nj+1))
            {
                    yWarning("Timeout: Using default Timeout=1000, i.e 1s\n");
                    for(i=1;i<nj+1; i++)
                        _velocityTimeout[i-1] = 1000;   //Default value
            }
            else
                {
                    for(i=1;i<xtmp.size(); i++) 
                        _velocityTimeout[i-1]=xtmp.get(i).asInt();
                }

            /////// Joint Speed Estimation
            xtmp.clear();
            if (!validate_optional(velocityGroup, xtmp, "JNT_speed_estimation", "a list of shift factors used by the firmware joint speed estimator", nj+1))
            {
                    yWarning("JNT_speed_estimation: Using default value=5\n");
                    for(i=1;i<nj+1; i++)
                        _estim_params[i-1].jnt_Vel_estimator_shift = 5;   //Default value
                }
            else
                {
                    for(i=1;i<xtmp.size(); i++) 
                        _estim_params[i-1].jnt_Vel_estimator_shift = xtmp.get(i).asInt();
                }

            /////// Motor Speed Estimation
            xtmp.clear();
            if (!validate_optional(velocityGroup, xtmp, "MOT_speed_estimation", "a list of shift factors used by the firmware motor speed estimator", nj+1))
                {
                    yWarning("MOT_speed_estimation: Using default value=5\n");
                    for(i=1;i<nj+1; i++)
                        _estim_params[i-1].mot_Vel_estimator_shift = 5;   //Default value
                }
            else
                {
                    for(i=1;i<xtmp.size(); i++) 
                        _estim_params[i-1].mot_Vel_estimator_shift = xtmp.get(i).asInt();
                }

            /////// Joint Acceleration Estimation
            xtmp.clear();
            if (!validate_optional(velocityGroup, xtmp, "JNT_accel_estimation", "a list of shift factors used by the firmware joint speed estimator", nj+1))
            {
                    yWarning("JNT_accel_estimation: Using default value=5\n");
                    for(i=1;i<nj+1; i++)
                        _estim_params[i-1].jnt_Acc_estimator_shift = 5;   //Default value
            }
            else
                {
                    for(i=1;i<xtmp.size(); i++) 
                        _estim_params[i-1].jnt_Acc_estimator_shift = xtmp.get(i).asInt();
                }

            /////// Motor Acceleration Estimation
            xtmp.clear();
            if (!validate_optional(velocityGroup, xtmp, "MOT_accel_estimation", "a list of shift factors used by the firmware motor speed estimator", nj+1))
                {
                    yWarning("MOT_accel_estimation: Using default value=5\n");
                    for(i=1;i<nj+1; i++)
                        _estim_params[i-1].mot_Acc_estimator_shift = 5;   //Default value
                }
            else
                {
                    for(i=1;i<xtmp.size(); i++) 
                        _estim_params[i-1].mot_Acc_estimator_shift = xtmp.get(i).asInt();
                }

        }
    else
    {
        yWarning("A suitable value for [VELOCITY] Shifts was not found. Using default Shifts=4\n");
        for(i=1;i<nj+1; i++)
            _velocityShifts[i-1] = 4;   //Default value

        yWarning("A suitable value for [VELOCITY] Timeout was not found. Using default Timeout=1000, i.e 1s.\n");
        for(i=1;i<nj+1; i++)
            _velocityTimeout[i-1] = 1000;   //Default value

        yWarning("A suitable value for [VELOCITY] speed estimation was not found. Using default shift factor=5.\n");
        for(i=1;i<nj+1; i++)
        {
            _estim_params[i-1].jnt_Vel_estimator_shift = 5;   //Default value
            _estim_params[i-1].jnt_Acc_estimator_shift = 5;   
            _estim_params[i-1].mot_Vel_estimator_shift = 5;   
            _estim_params[i-1].mot_Acc_estimator_shift = 5;  
        }
    }

    bool ret=true;
    if (!canGroup.findGroup("broadcast_pos").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_pos");
        ret=ret&&setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__POSITION);
    }

    if (!canGroup.findGroup("broadcast_pid").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_pid");
        ret=ret&&setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__PID_VAL);
    }
    if (!canGroup.findGroup("broadcast_fault").isNull())
    {
         xtmp=canGroup.findGroup("broadcast_fault");
         ret=ret&&setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__STATUS);
     }
    if (!canGroup.findGroup("broadcast_current").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_current");
        ret=ret&&setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__CURRENT);
    }
    if (!canGroup.findGroup("broadcast_overflow").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_overflow");
        ret=ret&&setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__OVERFLOW);
    }
    if (!canGroup.findGroup("broadcast_canprint").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_canprint");
        ret=ret&&setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__PRINT);
    }
    if (!canGroup.findGroup("broadcast_vel_acc").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_vel_acc");
        ret=ret&&setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__VELOCITY);
    }

    if (!canGroup.findGroup("broadcast_pid_err").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_pid_err");
        setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__PID_ERROR); //@@@ not checking return value in order to keep this option not mandatory
    }

    if (!canGroup.findGroup("broadcast_rotor_pos").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_rotor_pos");
        setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__MOTOR_POSITION);
    }

    if (!canGroup.findGroup("broadcast_rotor_vel_acc").isNull())
    {
        xtmp=canGroup.findGroup("broadcast_rotor_vel_acc");
        setBroadCastMask(xtmp, ICUBCANPROTO_PER_MC_MSG__MOTOR_SPEED);
    }

    if (!ret)
        yError("Invalid configuration file, check broadcast_* parameters\n");

    return ret;
}

CanBusMotionControlParameters::CanBusMotionControlParameters()
{
    _networkN=0;
    _destinations=0;
    _axisMap=0;
    _angleToEncoder=0;
    _rotToEncoder=0;
    _zeros=0;
    _pids=0;
    _tpids=0;
    _torqueControlEnabled=false;
    _pwmIsLimited=false;
    _limitsMax=0;
    _limitsMin=0;
    _currentLimits=0;
    _velocityShifts=0;
    _optical_factor=0;
    _velocityTimeout=0;
    _torqueSensorId=0;
    _torqueSensorChan=0;
    _maxTorque=0;
    _newtonsToSensor=0;
    _ampsToSensor = 0;
    _dutycycleToPwm = 0;
    _debug_params=0;
    _impedance_params=0;
    _impedance_limits=0;
    _estim_params=0;
    _bemfGain=0;
    _ktau=0;
    _axisName = 0;
    _axisType = 0;
    _motorPwmLimits = 0;
    _maxJntCmdVelocity = 0;

    _my_address = 0;
    _polling_interval = 10;
    _timeout = 20;
    _njoints = 0;

    _txQueueSize = 2047;/** max len of the buffer for the esd driver */
    _rxQueueSize = 2047;
    _txTimeout = 20;/** 20ms timeout */
    _rxTimeout = 20;
    _broadcast_mask=0;
    _torqueControlUnits=MACHINE_UNITS;
}

bool CanBusMotionControlParameters::alloc(int nj)
{
    _networkN = 0;
    _destinations = allocAndCheck<unsigned char> (CAN_MAX_CARDS);
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _rotToEncoder = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);
    _torqueSensorId= allocAndCheck<int>(nj);
    _torqueSensorChan= allocAndCheck<int>(nj);
    _maxTorque=allocAndCheck<double>(nj);
    _newtonsToSensor=allocAndCheck<double>(nj);
    _ampsToSensor = allocAndCheck<double>(nj);
    _dutycycleToPwm = allocAndCheck<double>(nj);
    _bemfGain=allocAndCheck<double>(nj);
    _ktau=allocAndCheck<double>(nj);
    _maxStep=allocAndCheck<double>(nj);
    _maxJntCmdVelocity = allocAndCheck<double>(nj);
    _motorPwmLimits = allocAndCheck<double>(nj);
    _filterType=allocAndCheck<int>(nj);
    _axisName = new std::string[nj];
    _axisType = new std::string[nj];

    _pids=allocAndCheck<Pid>(nj);
    _tpids=allocAndCheck<Pid>(nj);
    _debug_params=allocAndCheck<DebugParameters>(nj);
    _impedance_params=allocAndCheck<ImpedanceParameters>(nj);
    _impedance_limits=allocAndCheck<ImpedanceLimits>(nj);
    _estim_params=allocAndCheck<SpeedEstimationParameters>(nj);

    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _currentLimits=allocAndCheck<double>(nj);
    _optical_factor=allocAndCheck<double>(nj);
    _velocityShifts=allocAndCheck<int>(CAN_MAX_CARDS);
    _velocityTimeout=allocAndCheck<int>(CAN_MAX_CARDS);
    memset(_limitsMin, 0, sizeof(double)*nj);
    memset(_limitsMax, 0, sizeof(double)*nj);
    memset(_currentLimits, 0, sizeof(double)*nj);
    memset(_velocityShifts, 0, sizeof(int)*nj);
    memset(_optical_factor, 0, sizeof(double)*nj);
    memset(_velocityTimeout, 0, sizeof(int)*nj);
    memset(_maxStep, 0, sizeof(double)*nj);
    memset(_maxJntCmdVelocity, 0, sizeof(double)*nj);
    memset(_motorPwmLimits, 0, sizeof(double)*nj);

    _my_address = 0;
    _polling_interval = 10;
    _timeout = 20;
    _njoints = nj;

    _txQueueSize = 2047;/** max len of the buffer for the esd driver */
    _rxQueueSize = 2047;
    _txTimeout = 20;/** 20ms timeout */
    _rxTimeout = 20;

    _broadcast_mask=allocAndCheck<int>(nj);
    for(int j=0;j<nj;j++)
    {
        _broadcast_mask[j]=CAN_BCAST_NONE;
    }

    return true;
}

CanBusMotionControlParameters::~CanBusMotionControlParameters()
{
    checkAndDestroy<double>(_zeros);
    checkAndDestroy<double>(_angleToEncoder);
    checkAndDestroy<double>(_rotToEncoder);
    checkAndDestroy<int>(_axisMap);
    checkAndDestroy<unsigned char>(_destinations);
    checkAndDestroy<int>(_velocityShifts);
    checkAndDestroy<double>(_optical_factor);
    checkAndDestroy<int>(_velocityTimeout);
    checkAndDestroy<int>(_torqueSensorId);
    checkAndDestroy<int>(_torqueSensorChan);
    checkAndDestroy<double>(_maxTorque);
    checkAndDestroy<double>(_newtonsToSensor);
    checkAndDestroy<double>(_ampsToSensor);
    checkAndDestroy<double>(_dutycycleToPwm);
    checkAndDestroy<double>(_bemfGain);
    checkAndDestroy<double>(_ktau);
    checkAndDestroy<double>(_maxStep);
    checkAndDestroy<double>(_maxJntCmdVelocity);
    checkAndDestroy<double>(_motorPwmLimits);
    checkAndDestroy<int>(_filterType);
    checkAndDestroy<std::string>(_axisName);
    checkAndDestroy<std::string>(_axisType);

    checkAndDestroy<Pid>(_pids);
    checkAndDestroy<Pid>(_tpids);
    checkAndDestroy<SpeedEstimationParameters>(_estim_params);
    checkAndDestroy<DebugParameters>(_debug_params);
    checkAndDestroy<ImpedanceParameters>(_impedance_params);
    checkAndDestroy<ImpedanceLimits>(_impedance_limits);
    checkAndDestroy<double>(_limitsMax);
    checkAndDestroy<double>(_limitsMin);
    checkAndDestroy<double>(_currentLimits);
    checkAndDestroy<int>(_broadcast_mask);
}

///
///
///
///
///
///

CanBusResources::CanBusResources ()
{
    iCanBus=0;
    iBufferFactory=0;
    iCanErrors=0;

    _timeout = CAN_TIMEOUT;
    _polling_interval = CAN_POLLING_INTERVAL;
    _speed = 0;/// default 1Mbit/s
    _networkN = 0;
    _destInv=0;
    _jointNames = 0;
    _initialized=false;

    memset (_destinations, 0, sizeof(unsigned char) * CAN_MAX_CARDS);
    memset (_velShifts, 0, sizeof(int) * CAN_MAX_CARDS);

    _my_address = 0;
    _njoints = 0;

    _readMessages = 0;
    _writeMessages = 0;
    _echoMessages = 0;
    _bcastRecvBuffer = NULL;

    _error_status = true;
    requestsQueue=0;
}

CanBusResources::~CanBusResources () 
{ 
    uninitialize(); 
}

bool CanBusResources::initialize (yarp::os::Searchable &config)
{
    CanBusMotionControlParameters params;
    bool ret;

    if (!params.fromConfig(config))
        return false;

    polyDriver.open(config);
    if (!polyDriver.isValid())
    {
        yError("Could not instantiate CAN device\n");
        return false;
    }

    polyDriver.view(iCanBus);
    polyDriver.view(iBufferFactory);
    polyDriver.view(iCanErrors);

    if ((iCanBus==0) || (iBufferFactory==0))
    {
        yError("CanBusResources::initialize() could not get ICanBus or ICanBufferFactory interface");
        return false;
    }

    ret=initialize(params);
    return ret;
}

bool CanBusResources::initialize (const CanBusMotionControlParameters& parms)
{
    yTrace("Calling CanBusResources::initialize\n");
    if (_initialized)
        return false;

    // general variable init.
    _speed = 0;/// default 1Mbit/s
    _networkN = parms._networkN;

    _readMessages = 0;
    _writeMessages = 0;
    _error_status = true;

    memcpy (_destinations, parms._destinations, sizeof(unsigned char)*CAN_MAX_CARDS);
    memcpy (_velShifts, parms._velocityShifts, sizeof(int)*CAN_MAX_CARDS);

    for (int k = 0; k < CAN_MAX_CARDS; k++)
        _velShifts[k] = (1 << ((int) _velShifts[k]));

    _destInv=allocAndCheck<unsigned char>(CAN_MAX_CARDS);

    _my_address = parms._my_address;
    _polling_interval = parms._polling_interval;
    _timeout = parms._timeout;
    _njoints = parms._njoints;

    _jointNames = new std::string[_njoints];
    for (int i = 0; i < _njoints; i++) _jointNames[i] = parms._axisName[i];

    _txQueueSize=parms._txQueueSize;
    _rxQueueSize=parms._rxQueueSize;
    _filter = -1;

    for(int id=0;id<CAN_MAX_CARDS;id++)
        _destInv[id]=-1;

    //now fill inverted map
    printf("printing destinations and inverted map\n");
    for(int jj=0;jj<_njoints;jj+=2)
    {
        _destInv[_destinations[jj/2]]=jj;
        printf("%d %d\n",jj,_destinations[jj/2]);
     }

    _bcastRecvBuffer = allocAndCheck<BCastBufferElement> (_njoints);
    for (int j=0; j<_njoints ;j++)
        {
            _bcastRecvBuffer[j]._update_e=Time::now();
            _bcastRecvBuffer[j]._position_joint.resetStats();
            _bcastRecvBuffer[j]._position_rotor.resetStats();
            _bcastRecvBuffer[j]._speed_rotor.resetStats();
            _bcastRecvBuffer[j]._accel_rotor.resetStats();
        }

    //previously initialized
    iCanBus->canSetBaudRate(_speed);
    unsigned int i=0;

#if ICUB_CANMASKS_STRICT_FILTER
    yInfo("using ICUB_CANMASKS_STRICT_FILTER option\n");
    // sets all message ID's for class 0
    for (int j=0; j<CAN_MAX_CARDS; j++)
    {
        unsigned int sent_addr = 0x000 + (           0x000<<4) + _destinations[j];
        unsigned int recv_addr = 0x000 + (_destinations[j]<<4) + 0x000;
        iCanBus->canIdAdd(sent_addr);
        iCanBus->canIdAdd(recv_addr);
    }
    yDebug("class 0 set\n");

    // set all message ID's for class 1
    for (int j=0; j<CAN_MAX_CARDS; j++)
    {
        unsigned int start_addr = 0x100 + (_destinations[j]<<4) + 0x000;
        unsigned int end_addr   = 0x100 + (_destinations[j]<<4) + 0x00F;
        for (i = start_addr; i < end_addr; i++)
            iCanBus->canIdAdd(i);
    }
    yDebug("class 1 set\n");

#else
    yInfo("opening all CAN masks\n");
    // sets all message ID's for class 0
    for (i = 0x000; i < 0x0ff; i++)
        iCanBus->canIdAdd(i);
    yDebug("class 0 set\n");

    // set all message ID's for class 1
    for (i = 0x100; i < 0x1ff; i++)
        iCanBus->canIdAdd(i);
    yDebug("class 1 set\n");
#endif

    // set all message ID's for class 2
    for (i = 0x200; i < 0x2ff; i++)
        iCanBus->canIdAdd(i);
    yDebug("class 2 set\n");

    // set all message ID's for class 3
    for (i = 0x300; i < 0x3ff; i++)
        iCanBus->canIdAdd(i);
    yDebug("class 3 set\n");

    _readBuffer=iBufferFactory->createBuffer(BUF_SIZE);
    _writeBuffer=iBufferFactory->createBuffer(BUF_SIZE);
    _replyBuffer=iBufferFactory->createBuffer(BUF_SIZE);
    _echoBuffer=iBufferFactory->createBuffer(BUF_SIZE);
    yDebug("Can read/write buffers created, buffer size: %d\n", BUF_SIZE);

    requestsQueue = new RequestsQueue(_njoints, ICUBCANPROTO_POL_MC_CMD_MAXNUM);

    _initialized=true;

    yInfo("CanBusResources::initialized correctly\n");
    return true;
}


bool CanBusResources::uninitialize ()
{
    // remember uninitialize might be called more than once
    // need to check for already destroyed pointers.
    if (!_initialized)
        return true;

    //yTrace("CanBusResources::uninitialize\n");
    checkAndDestroy<BCastBufferElement> (_bcastRecvBuffer);

    if (_initialized)
    {
        iBufferFactory->destroyBuffer(_readBuffer);
        iBufferFactory->destroyBuffer(_writeBuffer);
        iBufferFactory->destroyBuffer(_replyBuffer);
        iBufferFactory->destroyBuffer(_echoBuffer);
        _initialized=false;
    }

    if (requestsQueue!=0)
    {
        delete requestsQueue;
        requestsQueue=0;
    }

    if (_destInv!=0)
    {
        delete [] _destInv;
        _destInv=0;
    }

    if (_jointNames != 0)
    {
        delete[] _jointNames;
        _jointNames = 0;
    }

    _initialized=false;
    return true;
}


bool CanBusResources::read ()
{
    unsigned int messages=BUF_SIZE;
    bool res=false;

    _readMessages=0;
    res=iCanBus->canRead(_readBuffer, messages, &_readMessages);
    return res;
}

bool CanBusResources::startPacket ()
{
    _writeMessages = 0;
    return true;
}

bool CanBusResources::addMessage (int msg_id, int joint)
{
    unsigned int id = _destinations[joint/2] & 0x0f;
    unsigned char data;
    data=msg_id;
    if ((joint % 2) == 1)
        data |= 0x80;

    _writeBuffer[_writeMessages].setId(id);
    _writeBuffer[_writeMessages].setLen(1);
    _writeBuffer[_writeMessages].getData()[0]=data;
    _writeMessages ++;

    return true;
}

bool CanBusResources::addMessage (int id, int joint, int msg_id)
{
    unsigned char *data=_writeBuffer[_writeMessages].getData();
    unsigned int destId= _destinations[joint/2] & 0x0f;

    data[0] = msg_id;
    if ((joint % 2) == 1)
        data[0] |= 0x80;

    _writeBuffer[_writeMessages].setId(destId);
    _writeBuffer[_writeMessages].setLen(1);
    _writeMessages ++;

    CanRequest rq;
    rq.threadId=id;
    rq.joint=joint;
    rq.msg=msg_id;
    requestsQueue->append(rq);

    return true;
}

bool CanBusResources::writePacket ()
{
    if (_writeMessages < 1)
        return false;

    unsigned int sent=0;

    DEBUG_FUNC("Sending buffer:\n");
    for(unsigned int k=0; k<_writeMessages;k++)
    {
        PRINT_CAN_MESSAGE("El:", _writeBuffer[k]);
    }

    bool res;
    res=iCanBus->canWrite(_writeBuffer, _writeMessages, &sent);
    if (!res)
    {
        return false;
    }

    return true;
}

bool CanBusResources::printMessage (const CanMessage& m)
{
    unsigned int id;
    unsigned int len;
    const unsigned char *data=m.getData();

    id=m.getId();
    len=m.getLen();


    int ret = sprintf (_printBuffer, "class: %2d s: %2x d: %2x c: %1d msg: %3d (%x) ",
        (id & 0x700) >> 8,
        (id & 0xf0) >> 4, 
        (id & 0x0f), 
        ((data[0] & 0x80)==0)?0:1,
        (data[0] & 0x7f),
        (data[0] & 0x7f));

    if (len > 1)
    {
        ret += sprintf (_printBuffer+ret, "x: "); 
    }

    unsigned int j;
    for (j = 1; j < len; j++)
    {
        ret += sprintf (_printBuffer+ret, "%x ", data[j]);
    }

    printf("%s", _printBuffer);

    return true;
}

bool CanBusResources::dumpBuffers (void)
{
    unsigned int j;

    /// dump the error.
    yDebug("CAN: write buffer\n");
    for (j = 0; j < _writeMessages; j++)
        printMessage (_writeBuffer[j]);

    yDebug("CAN: reply buffer\n");
    for (j = 0; j < _writeMessages; j++)
        printMessage (_replyBuffer[j]);

    yDebug("CAN: echo buffer\n");
    for (j = 0; j < _echoMessages; j++)
        printMessage (_echoBuffer[j]);

    yDebug("CAN: read buffer\n");
    for (j = 0; j < _readMessages; j++)
        printMessage (_readBuffer[j]);
    yDebug("CAN: -------------\n");

    return true;
}


CanBusMotionControl::CanBusMotionControl() :
RateThread(10),
//ImplementPositionControl<CanBusMotionControl, IPositionControl>(this),
ImplementPositionControl2(this),
//ImplementVelocityControl<CanBusMotionControl, IVelocityControl>(this),
ImplementVelocityControl2(this),
ImplementPidControl(this),
ImplementEncodersTimed(this),
ImplementControlCalibration<CanBusMotionControl, IControlCalibration>(this),
ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>(this),
ImplementControlLimits2(this),
ImplementTorqueControl(this),
ImplementImpedanceControl(this),
ImplementControlMode2(this),
ImplementPositionDirect(this),
ImplementInteractionMode(this),
ImplementMotorEncoders(this),
ImplementMotor(this),
ImplementRemoteVariables(this),
ImplementAxisInfo(this),
ImplementPWMControl(this),
ImplementCurrentControl(this),
_mutex(1),
_done(0)
{
    system_resources = (void *) new CanBusResources;
    ACE_ASSERT (system_resources != NULL);
    _opened = false;
    _axisTorqueHelper = 0;
    _firmwareVersionHelper = 0;
    _speedEstimationHelper = 0;
    _MCtorqueControlEnabled = false;
    _ref_command_positions = 0;
    _ref_positions = 0;
    _max_vel_jnt_cmd = 0;
    _ref_command_speeds = 0;
    _ref_speeds = 0;
    _ref_accs=0;
    _ref_torques=0;
    _last_position_move_time = 0;
    mServerLogger = NULL;
}


CanBusMotionControl::~CanBusMotionControl ()
{
    checkAndDestroy<double>(_ref_command_positions);
    checkAndDestroy<double>(_ref_command_speeds);
    checkAndDestroy<double>(_ref_positions);
    checkAndDestroy<double>(_max_vel_jnt_cmd);
    checkAndDestroy<double>(_ref_speeds);
    checkAndDestroy<double>(_ref_accs);
    checkAndDestroy<double>(_ref_torques);
    checkAndDestroy<double>(_last_position_move_time);

    if (system_resources != NULL)
        delete (CanBusResources *)system_resources;
    system_resources = NULL;
}

bool CanBusMotionControl::open (Searchable &config)
{
    yTrace("Opening CanBusMotionControl Control\n");
    std::string dbg_string = config.toString().c_str();
    
    CanBusResources& res = RES (system_resources);
    CanBusMotionControlParameters p;
    bool ret=false;
    _mutex.wait();

    if(!p.fromConfig(config))
    {
        _mutex.post();
        yError() << "One or more errors found while parsing device configuration";
        return false;
    }

    std::string str=config.toString().c_str();
    Property prop;
    prop.fromString(str.c_str());
    networkName=config.find("NetworkId").asString();
    canDevName=config.find("canbusdevice").asString(); //for backward compatibility
    if (canDevName=="") canDevName=config.findGroup("CAN").find("canbusdevice").asString();
    if(canDevName=="") { yError() << "cannot find parameter 'canbusdevice'\n"; return false;}
    prop.unput("device");
    prop.unput("subdevice");
    prop.put("device", canDevName.c_str());
    std::string canPhysDevName = config.find("physDevice").asString(); //for backward compatibility
    if (canPhysDevName=="") canPhysDevName = config.findGroup("CAN").find("physDevice").asString();
    prop.put("physDevice",canPhysDevName.c_str());
    prop.put("canDeviceNum", p._networkN);
    prop.put("canTxTimeout", p._txTimeout);
    prop.put("canRxTimeout", p._rxTimeout);
    if (p._txQueueSize!=-1)
        prop.put("canTxQueueSize", p._txQueueSize);
    if (p._rxQueueSize!=-1)
        prop.put("canRxQueueSize", p._rxQueueSize);

    ret=res.initialize(prop);

    if (!ret)
    {
        _mutex.post();
        yError() << "Unable to unitialize CAN driver";
        return false;
    }

    // used for printing debug messages.
    _filter = -1;
    _writerequested = false;
    _noreply = false;

    double *tmpZeros = new double [p._njoints];
    double *tmpOnes  = new double [p._njoints];
    for (int i=0; i< p._njoints; i++)
    {
        tmpZeros[i]=0.0;
        tmpOnes[i]=1.0;
    }

    ImplementPositionControl2::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);
    ImplementVelocityControl2::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

//    ImplementPositionControl<CanBusMotionControl, IPositionControl>::
//        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);
//
//    ImplementVelocityControl<CanBusMotionControl, IVelocityControl>::
//        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementPidControl::initialize(p._njoints, p._axisMap, p._angleToEncoder, NULL, p._newtonsToSensor, p._ampsToSensor, p._dutycycleToPwm);

    ImplementEncodersTimed::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementMotorEncoders::initialize(p._njoints, p._axisMap, p._rotToEncoder, tmpZeros);

    ImplementMotor::initialize(p._njoints, p._axisMap);

    ImplementControlCalibration<CanBusMotionControl, IControlCalibration>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementControlLimits2::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementControlMode2::initialize(p._njoints, p._axisMap);
    ImplementTorqueControl::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros, p._newtonsToSensor, p._ampsToSensor, nullptr,nullptr,nullptr);
    _axisTorqueHelper = new axisTorqueHelper(p._njoints,p._torqueSensorId,p._torqueSensorChan, p._maxTorque, p._newtonsToSensor);
    
    if      (p._torqueControlUnits==CanBusMotionControlParameters::MACHINE_UNITS) {}
    else if (p._torqueControlUnits==CanBusMotionControlParameters::METRIC_UNITS)  {}
    else    {yError() << "Invalid _torqueControlUnits value: %d" << p._torqueControlUnits; return false;}
    
    _axisImpedanceHelper = new axisImpedanceHelper(p._njoints, p._impedance_limits);
    ImplementImpedanceControl::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros, p._newtonsToSensor);
    ImplementPositionDirect::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);
    ImplementInteractionMode::initialize(p._njoints, p._axisMap);
    _axisPositionDirectHelper = new axisPositionDirectHelper(p._njoints, p._axisMap, p._angleToEncoder, p._maxStep);
    ImplementAxisInfo::initialize(p._njoints, p._axisMap);
    ImplementRemoteVariables::initialize(p._njoints, p._axisMap);
    ImplementCurrentControl::initialize(p._njoints, p._axisMap, p._ampsToSensor);
    ImplementPWMControl::initialize(p._njoints, p._axisMap, p._dutycycleToPwm);

    delete [] tmpZeros; tmpZeros=0;
    delete [] tmpOnes;  tmpOnes=0;

    // temporary variables used by the ddriver.
    _ref_command_positions = allocAndCheck<double>(p._njoints);
    _ref_positions = allocAndCheck<double>(p._njoints);
    _max_vel_jnt_cmd = allocAndCheck<double>(p._njoints);
    _ref_command_speeds = allocAndCheck<double>(p._njoints);
    _ref_speeds = allocAndCheck<double>(p._njoints);
    _ref_accs = allocAndCheck<double>(p._njoints);
    _ref_torques = allocAndCheck<double>(p._njoints);
    _last_position_move_time=allocAndCheck<double>(p._njoints);
    memset(_last_position_move_time, 0, sizeof(double)*p._njoints);

    for (int i = 0; i<p._njoints; i++)
    {
        _ref_command_positions[i] = NAN;
        _ref_speeds[i] = 0.0;
        _ref_command_speeds[i] = 0.0;
    }

    _mutex.post ();

    // default initialization for this device driver.
    yarp::os::Time::delay(0.005);
    setPids(VOCAB_PIDTYPE_POSITION, p._pids);
    
    if (p._torqueControlEnabled==true)
    {
        _MCtorqueControlEnabled = true;
        yarp::os::Time::delay(0.005);
        setPids(VOCAB_PIDTYPE_TORQUE, p._tpids);
        
        for (int i=0; i<p._njoints; i++)
        {
            MotorTorqueParameters ps;
            ps.bemf = p._bemfGain[i];
            this->setMotorTorqueParams(i,ps);

            yarp::os::Time::delay(0.002);
            this->setFilterTypeRaw(i,p._filterType[i]);
            yarp::os::Time::delay(0.002);
        }
    }
    
    //set the source of the torque measurments to the boards
    #if 0
    for (int j=0; j<p._njoints; j++)
    {
        yarp::os::Time::delay(0.001);
        this->setTorqueSource(j,p._torqueSensorId[j],p._torqueSensorChan[j]);
    }
    #endif

    // intial value !=0 for velocity reference during trajectory generation
    for (int j = 0; j<p._njoints; j++)
    {
        yarp::os::Time::delay(0.002);
        setRefSpeed(j, 10.0);
    }

    // impedance parameters
    for (int j=0; j<p._njoints; j++)
    {
        yarp::os::Time::delay(0.001);
        setImpedance(j,p._impedance_params[j].stiffness,p._impedance_params[j].damping);
    }

    int i;
    for(i = 0; i < p._njoints; i++)
    {
        yarp::os::Time::delay(0.001);
        setBCastMessages(i, p._broadcast_mask[i]);
    }

    // set limits, on encoders and max current
    for(i = 0; i < p._njoints; i++)
    {
        yarp::os::Time::delay(0.001);
        setLimits(i, p._limitsMin[i], p._limitsMax[i]);
        yarp::os::Time::delay(0.001);
        setVelLimits(i, 0, p._maxJntCmdVelocity[i]);
        yarp::os::Time::delay(0.001);
        setMaxCurrent(i, p._currentLimits[i]);
    }

    // set limits, on encoders and max current
    for(i = 0; i < p._njoints; i++)
    {   
        yarp::os::Time::delay(0.001);
        setVelocityShiftRaw(i, p._velocityShifts[i]);
        yarp::os::Time::delay(0.001);
        setVelocityTimeoutRaw(i, p._velocityTimeout[i]);
        yarp::os::Time::delay(0.001);
        setPWMLimit(i, p._motorPwmLimits[i]);
    }

    // set parameters for speed/acceleration estimation
    for(i = 0; i < p._njoints; i++)
    {
        yarp::os::Time::delay(0.001);
        setSpeedEstimatorShiftRaw(i,p._estim_params[i].jnt_Vel_estimator_shift,
                                    p._estim_params[i].jnt_Acc_estimator_shift,
                                    p._estim_params[i].mot_Vel_estimator_shift,
                                    p._estim_params[i].mot_Acc_estimator_shift);
    }
    _speedEstimationHelper = new speedEstimationHelper(p._njoints, p._estim_params);
    
    // disable the controller, cards will start with the pid controller & pwm off
    for (i = 0; i < p._njoints; i++)
    {
        yarp::os::Time::delay(0.001);
        setControlMode(i, VOCAB_CM_IDLE);
    }
    const Bottle &analogList=config.findGroup("analog").tail();
    //    if (analogList!=0)
        if (analogList.size()>0)
        {
            for(int k=0;k<analogList.size();k++)
            {
                std::string analogId=analogList.get(k).asString().c_str();

                TBR_AnalogSensor *as=instantiateAnalog(config, analogId);
                if (as!=0)
                    {
                        analogSensors.push_back(as);
                    }
            }

        }

    threadPool = new ThreadPool2(res.iBufferFactory);

    RateThread::setRate(p._polling_interval);
    RateThread::start();

    //get firmware versions
    firmware_info* info = new firmware_info[p._njoints];
    can_protocol_info icub_interface_protocol;
    icub_interface_protocol.major=CAN_PROTOCOL_MAJOR;
    icub_interface_protocol.minor=CAN_PROTOCOL_MINOR;
    for (int j=0; j<p._njoints; j++) 
    {
        yarp::os::Time::delay(0.001);
        bool b=getFirmwareVersionRaw(j,icub_interface_protocol,&(info[j]));
        if (b==false) yError() << "Unable to read firmware version";
    }
    _firmwareVersionHelper = new firmwareVersionHelper(p._njoints, info, icub_interface_protocol);
    _firmwareVersionHelper->printFirmwareVersions();

#ifdef ICUB_CANPROTOCOL_STRICT
    ////////////////////////////
    if (!_firmwareVersionHelper->checkFirmwareVersions())
    {
        RateThread::stop();
        _opened = false;
        yError() << "checkFirmwareVersions() failed. CanBusMotionControl::open returning false,";
        return false;
    }
    /////////////////////////////////
#else
    yWarning("*********************************************************************************\n");
    yWarning("**** WARNING: ICUB_CANPROTOCOL_STRICT not defined, skipping firmware check! *****\n");
    yWarning("*********************************************************************************\n");
#endif

    _opened = true;
    yDebug() << "CanBusMotionControl::open returned true\n";
    return true;
}

bool CanBusMotionControl::readFullScaleAnalog(int analog_address, int ch, double* fullScale)
{
    CanBusResources& res = RES (system_resources);
    int destId=0x0200|analog_address;
    *fullScale=1e-20;
    unsigned int i=0;
    res.startPacket();
    res._writeBuffer[0].setId(destId);
    res._writeBuffer[0].getData()[0]=0x18;
    res._writeBuffer[0].getData()[1]=ch;
    res._writeBuffer[0].setLen(2);
    res._writeMessages++;
    res.writePacket();
                
    long int timeout=0;
    bool full_scale_read=false;
    do 
    {
        res.read();
        for (i=0; i<res._readMessages; i++)
        {
            CanMessage& m = res._readBuffer[i];
            unsigned int currId=m.getId();
            if (currId==(0x0200|(analog_address<<4)))
                if (m.getLen()==4 &&
                    m.getData()[0]==0x18 &&
                    m.getData()[1]==ch)
                    {
                        *fullScale=m.getData()[2]<<8 | m.getData()[3];
                        full_scale_read=true;
                        break;
                    }
        }
        yarp::os::Time::delay(0.002);
        timeout++;
    }
    while(timeout<32 && full_scale_read==false);

    if (full_scale_read==false) 
        {                            
            yError() << "Trying to get fullscale data from sensor: no answer received or message lost (ch:" << ch <<")";
            return false;
        }

    return true;
}

TBR_AnalogSensor *CanBusMotionControl::instantiateAnalog(yarp::os::Searchable& config, std::string deviceid)
{
    CanBusResources& res = RES (system_resources);
    TBR_AnalogSensor *analogSensor=0;

    //std::string groupName=std::string("ANALOG-");
    //groupName+=deviceid;
    //Bottle analogConfig=config.findGroup(groupName.c_str());
    Bottle &analogConfig=config.findGroup(deviceid.c_str());
    if (analogConfig.size()>0)
    {
        yDebug("Initializing analog device %s\n", deviceid.c_str());
        
        analogSensor=new TBR_AnalogSensor;
        analogSensor->setDeviceId(deviceid);

        bool isVirtualSensor=false;
        char analogId=analogConfig.find("CanAddress").asInt();
        char analogFormat=analogConfig.find("Format").asInt();
        int analogChannels=analogConfig.find("Channels").asInt();
        int analogCalibration=analogConfig.find("UseCalibration").asInt();
        //int SensorFullScale=analogConfig.find("FullScale").asInt();

        if (analogConfig.check("PortName"))
        {
            isVirtualSensor = true;
            std::string virtualPortName = analogConfig.find("PortName").asString();
            bool   canEchoEnabled = analogConfig.find("CanEcho").asInt();
            analogSensor->backDoor = new TBR_CanBackDoor();
            analogSensor->backDoor->setUp(&res, &_mutex, canEchoEnabled, analogSensor);
            std::string rn("/");
            rn += config.find("robotName").asString().c_str();
            //rn+=String("/");
            rn+=virtualPortName;
            analogSensor->backDoor->open(rn.c_str()); 
            //RANDAZ_TODO if needed set other parameters to backDoor
        }

        switch (analogFormat)
        {
            case 8:
                analogSensor->open(analogChannels, TBR_AnalogSensor::ANALOG_FORMAT_8, analogId, analogCalibration, isVirtualSensor);
                break;
            case 16:
                analogSensor->open(analogChannels, TBR_AnalogSensor::ANALOG_FORMAT_16, analogId, analogCalibration, isVirtualSensor);
                break;
        }

        int destId=0x0200|analogSensor->getId();

        if (analogConfig.check("Period"))
        {
            int period=analogConfig.find("Period").asInt();
            res.startPacket();

            res.startPacket();
            res._writeBuffer[0].setId(destId);
            res._writeBuffer[0].getData()[0]=0x08;
            res._writeBuffer[0].getData()[1]=period;
            res._writeBuffer[0].setLen(2);
            res._writeMessages++;
            res.writePacket();

        }

        //init message for mais board
        if (analogChannels==16 && analogFormat==8)
        {
                res.startPacket();
                res._writeBuffer[0].setId(destId);
                res._writeBuffer[0].getData()[0]=0x07;
                res._writeBuffer[0].getData()[1]=0x00;
                res._writeBuffer[0].setLen(2);
                res._writeMessages++;
                res.writePacket();
        }
        //init message for strain board
        else if (analogChannels==6 && analogFormat==16 && isVirtualSensor==false)
        {
                //calibrated astrain board
                if (analogCalibration==1)
                {
                    //get the full scale values from the strain board
                    for (int ch=0; ch<6; ch++)
                    {
                        bool b=false;
                        int attempts = 0;
                        while(attempts<15) 
                        {
                            b = readFullScaleAnalog(analogSensor->getId(), ch, &analogSensor->getScaleFactor()[ch]);
                            if (b==true) 
                                {
                                    if (attempts>0)    yWarning("Trying to get fullscale data from sensor: channel recovered (ch:%d)\n", ch);
                                    break;
                                }
                            attempts++;
                            yarp::os::Time::delay(0.020);
                        }
                        if (attempts>=15)
                        {
                            yError("Trying to get fullscale data from sensor %s: all attempts failed (ch:%d)\n", deviceid.c_str(), ch);
                            yError("Device %s cannot be opened.\n", deviceid.c_str());
                            return 0; //@@@delete missing, but TBR_AnalogSensor will be deprecated soon
                        }
                    }

                    // debug messages
                    #if 1
                         yDebug("Sensor Fullscale Id %#4X: %f %f %f %f %f %f ",analogId,
                                 analogSensor->getScaleFactor()[0],
                                 analogSensor->getScaleFactor()[1],
                                 analogSensor->getScaleFactor()[2],
                                 analogSensor->getScaleFactor()[3],
                                 analogSensor->getScaleFactor()[4],
                                 analogSensor->getScaleFactor()[5]);
                    #endif

                    // start the board
                    res.startPacket();
                    res._writeBuffer[0].setId(destId);
                    res._writeBuffer[0].getData()[0]=0x07;
                    res._writeBuffer[0].getData()[1]=0x00;
                    res._writeBuffer[0].setLen(2);
                    res._writeMessages++;
                    res.writePacket();
                }
                //not calibrated strain board
                else
                {
                    res.startPacket();
                    res._writeBuffer[0].setId(destId);
                    res._writeBuffer[0].getData()[0]=0x07;
                    res._writeBuffer[0].getData()[1]=0x03;
                    res._writeBuffer[0].setLen(2);
                    res._writeMessages++;
                    res.writePacket();
                }
        }
        else if (analogChannels==6 && analogFormat==16 && isVirtualSensor==true)
        {    
            //set the full scale values for a VIRTUAL sensor
            for (int jnt=0; jnt<_axisTorqueHelper->getNumberOfJoints(); jnt++)
            {
                int cfgId     = _axisTorqueHelper->getTorqueSensorId(jnt);
                int cfgChan   = _axisTorqueHelper->getTorqueSensorChan(jnt);
                if (cfgId == analogId)
                {
                    for (int ch=0; ch<6; ch++)
                    {
                        if (cfgChan == ch)
                        {
                            double maxTrq = _axisTorqueHelper->getMaximumTorque(jnt);
                            analogSensor->getScaleFactor()[ch]=maxTrq;
                            break;
                        }
                    }
                }
            }

            // debug messages
            #if 1
                 yDebug("Sensor Fullscale Id %#4X: %f  %f  %f  %f  %f  %f",analogId,
                        analogSensor->getScaleFactor()[0],
                        analogSensor->getScaleFactor()[1],
                        analogSensor->getScaleFactor()[2],
                        analogSensor->getScaleFactor()[3],
                        analogSensor->getScaleFactor()[4],
                        analogSensor->getScaleFactor()[5]);
            #endif
        }
    }
    return analogSensor;
}

void CanBusMotionControl::finiAnalog(TBR_AnalogSensor *analogSensor)
{
    CanBusResources& res = RES (system_resources);

    if (analogSensor->isOpen())
            {
                res.startPacket();
                int destId=0x0200|analogSensor->getId();

                res.startPacket();
                res._writeBuffer[0].setId(destId);
                res._writeBuffer[0].getData()[0]=0x07;
                res._writeBuffer[0].getData()[1]=0x01;
                res._writeBuffer[0].setLen(2);
                res._writeMessages++;

                //debug
#if 0
                yDebug("---> Len:%d %x %x %x\n", 
                    res._writeBuffer[0].getLen(),
                    res._writeBuffer[0].getId(),
                    res._writeBuffer[0].getData()[0],
                    res._writeBuffer[0].getData()[1]);
#endif
                res.writePacket();

                //shut down backdoor
                if (analogSensor->backDoor)
                    analogSensor->backDoor->close();
            }
}

bool CanBusMotionControl::close (void)
{
    CanBusResources& res = RES(system_resources);

    //yDebug("CanBusMotionControl::close\n");

    if (_opened) {
        // disable the controller, pid controller & pwm off
        int i;
        for (i = 0; i < res._njoints; i++)
        {
            setControlMode(i, VOCAB_CM_IDLE);
        }

        if (isRunning())
        {
            /// default initialization for this device driver.
            int i;
            for(i = 0; i < res.getJoints(); i++)
                setBCastMessages(i, 0x00);
        }

        RateThread::stop ();/// stops the thread first (joins too).

        ImplementPositionControl2::uninitialize();
        ImplementVelocityControl2::uninitialize();

//        ImplementPositionControl<CanBusMotionControl, IPositionControl>::uninitialize ();
//        ImplementVelocityControl<CanBusMotionControl, IVelocityControl>::uninitialize();

        ImplementPidControl::uninitialize();
        ImplementEncodersTimed::uninitialize();
        ImplementMotorEncoders::uninitialize();
        ImplementControlCalibration<CanBusMotionControl, IControlCalibration>::uninitialize();
        ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>::uninitialize();
        ImplementControlLimits2::uninitialize();

        ImplementControlMode2::uninitialize();
        ImplementTorqueControl::uninitialize();
        ImplementImpedanceControl::uninitialize();
        ImplementPositionDirect::uninitialize();
        ImplementInteractionMode::uninitialize();
        ImplementRemoteVariables::uninitialize();
        ImplementAxisInfo::uninitialize();
        ImplementCurrentControl::uninitialize();
        ImplementPWMControl::uninitialize();

        //stop analog sensors
        std::list<TBR_AnalogSensor *>::iterator it=analogSensors.begin();
        while(it!=analogSensors.end())
        {
            if ((*it))
            {
                finiAnalog(*it);
                delete (*it);
                (*it)=0;
            }
            it++;
        }
        
    }

    if (threadPool != 0)
       {delete threadPool; threadPool = 0;}
    if (_axisTorqueHelper != 0)
       {delete _axisTorqueHelper; _axisTorqueHelper = 0;}
    if (_firmwareVersionHelper != 0)
       {delete _firmwareVersionHelper; _firmwareVersionHelper =0;}

    checkAndDestroy<double> (_ref_command_positions);
    checkAndDestroy<double> (_ref_command_speeds);
    checkAndDestroy<double> (_ref_positions);
    checkAndDestroy<double> (_ref_speeds);
    checkAndDestroy<double> (_ref_accs);
    checkAndDestroy<double> (_ref_torques);
    checkAndDestroy<double> (_max_vel_jnt_cmd);

    int ret = res.uninitialize ();
    _opened = false;

    return ret;
}

void CanBusMotionControl::handleBroadcasts()
{        
    CanBusResources& r = RES (system_resources);

    double before=Time::now();
    unsigned int i=0;
    const int _networkN=r._networkN;

    for (unsigned int buff_num=0; buff_num<2; buff_num++)
    {
        unsigned int size = 0;
        CanBuffer* buffer_pointer=0;
        if (buff_num==0)
        {
            size = r._readMessages;
            buffer_pointer = &r._readBuffer;
        }
        else
        {
            size = r._echoMessages;
            buffer_pointer = &r._echoBuffer;
        }

        for (i = 0; i < size; i++)
        {
            unsigned int len=0;
            unsigned int id=0;
            unsigned char *data=0;
            CanMessage& m = (*buffer_pointer)[i];
            data=m.getData();
            id=m.getId();
            len=m.getLen();

            if ((id & 0x700) == 0x300) // class = 3 These messages come from analog sensors
            {
                // 4 next bits = source address, next 4 bits = msg type
                const unsigned int addr = ((id & 0x0f0) >> 4);
                const unsigned int type =   id & 0x00f;
                if  (type == 0x0A || type == 0x0B) //strain messages
                {
                    int off = (type-0x0A)*3;
                    for (int axis=0; axis<r.getJoints(); axis++)
                    {
                        int attached_board = _axisTorqueHelper->getTorqueSensorId(axis);
                        if ( attached_board == addr)
                        {
                            for(int chan=0;chan<3;chan++)
                            {
                                int attached_channel = _axisTorqueHelper->getTorqueSensorChan(axis);
                                if ( attached_channel == chan+off)
                                {
                                    double scaleFactor = 1/_axisTorqueHelper->getNewtonsToSensor(axis);
                                    r._bcastRecvBuffer[axis]._torque=(((unsigned short)(data[2*chan+1]))<<8)+data[2*chan]-0x8000;
                                    r._bcastRecvBuffer[axis]._torque=r._bcastRecvBuffer[axis]._torque*scaleFactor;
                                    //r._bcastRecvBuffer[axis]._torque=0;
                                    r._bcastRecvBuffer[axis]._update_t = before;
                                }
                            }
                        }
                    }
                }
            }
            else if ((id & 0x700) == 0x100) // class = 1 These messages come from the control boards.
            {
                // 4 next bits = source address, next 4 bits = msg type
                // this allows sending two 32-bit numbers is a single CAN message.
                //
                // need an array here for storing the messages on a per-joint basis.
                const int addr = ((id & 0x0f0) >> 4);
                int j;
                bool found=false;
                for (j = 0; j < CAN_MAX_CARDS; j++)
                {
                    if (r._destinations[j] == addr)
                    {
                        found = true;
                        break;
                    }
                }

                if (!found)
                {
                    static int count=0;
                    if (count%5000==0)
                    {
                        yError("%s [%d] Warning, got unexpected broadcast msg(s), last one from address %d, (original) id  0x%x, len %d\n", canDevName.c_str(), _networkN, addr, id, len);
                        char tmp1 [255]; tmp1[0]=0;
                        char tmp2 [255]; tmp2[0]=0;
                        for (j = 0; j < CAN_MAX_CARDS; j++)
                        {
                            sprintf (tmp1, "%d ", r._destinations[j]);
                            strcat  (tmp2,tmp1);
                        }
                        yError("%s [%d] valid addresses are (%s)\n",canDevName.c_str(), _networkN,tmp2);
                        count++;
                        j=-1; //error
                    }
                }
                else
                {
                    j *= 2;

                    /* less sign nibble specifies msg type */
                    switch (id & 0x00f)
                    {
                    case ICUBCANPROTO_PER_MC_MSG__OVERFLOW:

                        yError ("CAN PACKET LOSS, board %d buffer full\r\n", (((id & 0x0f0) >> 4)-1));

                        break;

                    case ICUBCANPROTO_PER_MC_MSG__PRINT:

                        if (data[0] == ICUBCANPROTO_PER_MC_MSG__PRINT    ||
                            data[0] == ICUBCANPROTO_PER_MC_MSG__PRINT + 128)
                        {    
                            int addr = (((id & 0x0f0) >> 4)-1);

                            int string_id = cstring[addr].add_string(&r._readBuffer[i]);
                            if (string_id != -1) 
                            {
                                cstring[addr].print(string_id, canDevName.c_str(), r._networkN);
                                cstring[addr].clear_string(string_id);
                            }
                        }
                        break;

                    case ICUBCANPROTO_PER_MC_MSG__POSITION:
                        {
                            // r._bcastRecvBuffer[j]._position = *((int *)(data));
                            // r._bcastRecvBuffer[j]._update_p = before;
                            int tmp=*((int *)(data));
                            r._bcastRecvBuffer[j]._position_joint.update(tmp, before);

                            j++;
                            if (j < r.getJoints())
                                {
                                    tmp =*((int *)(data+4));
                                    //r._bcastRecvBuffer[j]._position = *((int *)(data+4));
                                    //r._bcastRecvBuffer[j]._update_p = before;
                                    r._bcastRecvBuffer[j]._position_joint.update(tmp, before);
                                }
                        }
                        break;

                    case ICUBCANPROTO_PER_MC_MSG__MOTOR_POSITION:
                        {
                            // r._bcastRecvBuffer[j]._position = *((int *)(data));
                            // r._bcastRecvBuffer[j]._update_p = before;
                            int tmp=*((int *)(data));
                            r._bcastRecvBuffer[j]._position_rotor.update(tmp, before);

                            j++;
                            if (j < r.getJoints())
                                {
                                    tmp =*((int *)(data+4));
                                    //r._bcastRecvBuffer[j]._position = *((int *)(data+4));
                                    //r._bcastRecvBuffer[j]._update_p = before;
                                    r._bcastRecvBuffer[j]._position_rotor.update(tmp, before);
                                }
                        }
                        break;

                    case ICUBCANPROTO_PER_MC_MSG__MOTOR_SPEED:
                    {
                        int tmp;
                        tmp =*((short *)(data));
                        r._bcastRecvBuffer[j]._speed_rotor.update(tmp, before);
                        tmp =*((short *)(data+4));
                        r._bcastRecvBuffer[j]._accel_rotor.update(tmp, before);
                        r._bcastRecvBuffer[j]._update_s = before;
                        j++;
                        if (j < r.getJoints())
                        {
                            tmp =*((short *)(data+2));
                            r._bcastRecvBuffer[j]._speed_rotor.update(tmp, before);
                            tmp =*((short *)(data+6));
                            r._bcastRecvBuffer[j]._accel_rotor.update(tmp, before);
                            r._bcastRecvBuffer[j]._update_s = before;
                        }
                        break;
                    }
                    break;
    #if 0
                    case ICUBCANPROTO_PER_MC_MSG__TORQUE:
                        {
                            int tmp=0; //*((int *)(data));
                            r._bcastRecvBuffer[j]._torque=tmp;
                            r._bcastRecvBuffer[j]._update_t=before;

                            j++;
                            if (j < r.getJoints())
                                {
                                    tmp = 0;//*((int *)(data+4));
                                    r._bcastRecvBuffer[j]._torque=tmp;
                                    r._bcastRecvBuffer[j]._update_t=before;
                                }
                        }
    #endif

                    case ICUBCANPROTO_PER_MC_MSG__PID_VAL:
                        r._bcastRecvBuffer[j]._pid_value = *((short *)(data));
                        r._bcastRecvBuffer[j]._update_v = before;

                        j++;
                        if (j < r.getJoints())
                        {
                            r._bcastRecvBuffer[j]._pid_value = *((short *)(data+2));
                            r._bcastRecvBuffer[j]._update_v = before;
                        }
                        break;

                    case ICUBCANPROTO_PER_MC_MSG__STATUS:
                        // fault signals.
                        r._bcastRecvBuffer[j]._axisStatus= *((short *)(data));
                        r._bcastRecvBuffer[j]._canStatus= *((char *)(data+4));
                        r._bcastRecvBuffer[j]._boardStatus= *((char *)(data+5));
                        r._bcastRecvBuffer[j]._update_e = before;
                        r._bcastRecvBuffer[j]._controlmodeStatus=*((char *)(data+1));
                        r._bcastRecvBuffer[j]._address=addr;
                        r._bcastRecvBuffer[j]._canTxError+=*((char *) (data+6));
                        r._bcastRecvBuffer[j]._canRxError+=*((char *) (data+7));                                    
    #if 0                    
                        if (_networkN==1)
                            {
                                for(int m=0;m<8;m++)
                                    yDebug("%.2x ", data[m]);
                                yDebug("\n");
                            }
    #endif

                        bool bFlag;

                        if ((bFlag=r._bcastRecvBuffer[j].isOverCurrent())) yError ("%s [%d] board %d OVERCURRENT AXIS 0\n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,9,yarp::os::Value((int)bFlag));

                        //r._bcastRecvBuffer[j].ControlStatus(r._networkN, r._bcastRecvBuffer[j]._controlmodeStatus,addr); 
                        //if (r._bcastRecvBuffer[j].isFaultOk()) yInfo("Board %d OK\n", addr);
                    
                        logJointData(canDevName.c_str(),_networkN,j,21,yarp::os::Value((int)r._bcastRecvBuffer[j]._controlmodeStatus));

                        if ((bFlag=r._bcastRecvBuffer[j].isFaultUndervoltage())) yError ("%s [%d] board %d FAULT UNDERVOLTAGE AXIS 0\n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,7,yarp::os::Value((int)bFlag));
                    
                        if ((bFlag=r._bcastRecvBuffer[j].isFaultExternal())) yWarning ("%s [%d] board %d FAULT EXT AXIS 0\n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,10,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isFaultOverload())) yError ("%s [%d] board %d FAULT OVERLOAD AXIS 0\n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,8,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isHallSensorError())) yError ("%s [%d] board %d HALL SENSOR ERROR AXIS 0\n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,11,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isAbsEncoderError())) yError ("%s [%d] board %d ABS ENCODER ERROR AXIS 0\n", canDevName.c_str(), _networkN, addr);        
                        logJointData(canDevName.c_str(),_networkN,j,12,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isOpticalEncoderError())) yError ("%s [%d] board %d OPTICAL ENCODER ERROR AXIS 0\n", canDevName.c_str(), _networkN, addr);        
                        logJointData(canDevName.c_str(),_networkN,j,12,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isCanTxOverflow())) yError ("%s [%d] board %d CAN TX OVERFLOW \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,16,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isCanBusOff())) yError ("%s [%d] board %d CAN BUS_OFF \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,13,yarp::os::Value((int)bFlag));

                        if (r._bcastRecvBuffer[j].isCanTxError()) yError ("%s [%d] board %d CAN TX ERROR \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,14,yarp::os::Value((int)r._bcastRecvBuffer[j]._canTxError));

                        if (r._bcastRecvBuffer[j].isCanRxError()) yError ("%s [%d] board %d CAN RX ERROR \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,15,yarp::os::Value((int)r._bcastRecvBuffer[j]._canRxError));

                        if ((bFlag=r._bcastRecvBuffer[j].isCanTxOverrun())) yError ("%s [%d] board %d CAN TX OVERRUN \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,17,yarp::os::Value((int)bFlag));

                        if (r._bcastRecvBuffer[j].isCanRxWarning()) yError ("%s [%d] board %d CAN RX WARNING \n", canDevName.c_str(), _networkN, addr);

                        if ((bFlag=r._bcastRecvBuffer[j].isCanRxOverrun())) yError ("%s [%d] board %d CAN RX OVERRUN \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,17,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isMainLoopOverflow())) 
                        {
                            r._bcastRecvBuffer[j]._mainLoopOverflowCounter++;
                            //yWarning ("%s [%d] board %d MAIN LOOP TIME EXCEDEED \n", canDevName.c_str(), _networkN, addr);
                        }
                        logJointData(canDevName.c_str(),_networkN,j,18,yarp::os::Value((int)bFlag));

                        if ((bFlag=r._bcastRecvBuffer[j].isOverTempCh1())) yError ("%s [%d] board %d OVER TEMPERATURE CH 1 \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,19,yarp::os::Value((int)bFlag));
                    
                        if ((bFlag=r._bcastRecvBuffer[j].isOverTempCh2())) yError ("%s [%d] board %d OVER TEMPERATURE CH 2 \n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j+1,19,yarp::os::Value((int)bFlag));
                    
                        if ((bFlag=r._bcastRecvBuffer[j].isTempErrorCh1())) yError ("%s [%d] board %d ERROR TEMPERATURE CH 1\n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j,20,yarp::os::Value((int)bFlag));
                    
                        if ((bFlag=r._bcastRecvBuffer[j].isTempErrorCh2())) yError ("%s [%d] board %d ERROR TEMPERATURE CH 2\n", canDevName.c_str(), _networkN, addr);
                        logJointData(canDevName.c_str(),_networkN,j+1,20,yarp::os::Value((int)bFlag));

                        j++;

                        if (j < r.getJoints())
                        {
                            r._bcastRecvBuffer[j]._address=addr;
                            r._bcastRecvBuffer[j]._axisStatus= *((short *)(data+2));
                            r._bcastRecvBuffer[j]._update_e = before;    
                            r._bcastRecvBuffer[j]._controlmodeStatus=*((char *)(data+3));
                            // r._bcastRecvBuffer[j].ControlStatus(r._networkN, r._bcastRecvBuffer[j]._controlmodeStatus,addr); 
                        
                            logJointData(canDevName.c_str(),_networkN,j,21,yarp::os::Value((int)r._bcastRecvBuffer[j]._controlmodeStatus));

                            if ((bFlag=r._bcastRecvBuffer[j].isOverCurrent())) yError ("%s [%d] board %d OVERCURRENT AXIS 1\n", canDevName.c_str(), _networkN, addr);
                            logJointData(canDevName.c_str(),_networkN,j,9,yarp::os::Value((int)bFlag));
                        
                            if ((bFlag=r._bcastRecvBuffer[j].isFaultUndervoltage())) yError ("%s [%d] board %d FAULT UNDERVOLTAGE AXIS 1\n", canDevName.c_str(), _networkN, addr);
                            logJointData(canDevName.c_str(),_networkN,j,7,yarp::os::Value((int)bFlag));
                        
                            if ((bFlag=r._bcastRecvBuffer[j].isFaultExternal())) yWarning ("%s [%d] board %d FAULT EXT AXIS 1\n", canDevName.c_str(), _networkN, addr);
                            logJointData(canDevName.c_str(),_networkN,j,10,yarp::os::Value((int)bFlag));
                        
                            if ((bFlag=r._bcastRecvBuffer[j].isFaultOverload())) yError ("%s [%d] board %d FAULT OVERLOAD AXIS 1\n", canDevName.c_str(), _networkN, addr);
                            logJointData(canDevName.c_str(),_networkN,j,8,yarp::os::Value((int)bFlag));
                        
                            if ((bFlag=r._bcastRecvBuffer[j].isHallSensorError())) yError ("%s [%d] board %d HALL SENSOR ERROR AXIS 1\n", canDevName.c_str(), _networkN, addr);
                            logJointData(canDevName.c_str(),_networkN,j,11,yarp::os::Value((int)bFlag));
                        
                            if ((bFlag=r._bcastRecvBuffer[j].isAbsEncoderError())) yError ("%s [%d] board %d ABS ENCODER ERROR AXIS 1\n", canDevName.c_str(), _networkN, addr);
                            logJointData(canDevName.c_str(),_networkN,j,12,yarp::os::Value((int)bFlag));

                            if ((bFlag=r._bcastRecvBuffer[j].isOpticalEncoderError())) yError ("%s [%d] board %d OPTICAL ENCODER ERROR AXIS 0\n", canDevName.c_str(), _networkN, addr);        
                            logJointData(canDevName.c_str(),_networkN,j,12,yarp::os::Value((int)bFlag));
                        }    

                        break;

                    case ICUBCANPROTO_PER_MC_MSG__ADDITIONAL_STATUS:
                        r._bcastRecvBuffer[j]._interactionmodeStatus=*((char *)(data)) & 0x0F;
                        r._bcastRecvBuffer[j]._update_e2 = before;
                        j++;
                        if (j < r.getJoints())
                        {
                            r._bcastRecvBuffer[j]._interactionmodeStatus=(*((char *)(data)) >> 4) & 0x0F;
                            r._bcastRecvBuffer[j]._update_e2 = before;
                        }
                        break;

                    case ICUBCANPROTO_PER_MC_MSG__CURRENT:
                        r._bcastRecvBuffer[j]._current = *((short *)(data));
                        r._bcastRecvBuffer[j]._update_c = before;
                        j++;
                        if (j < r.getJoints())
                        {
                            r._bcastRecvBuffer[j]._current = *((short *)(data+2));
                            r._bcastRecvBuffer[j]._update_c = before;
                        }
                        break;

                    case ICUBCANPROTO_PER_MC_MSG__PID_ERROR:
                        r._bcastRecvBuffer[j]._position_error = *((short *)(data));
                        r._bcastRecvBuffer[j]._torque_error =   *((short *)(data+4));
                        r._bcastRecvBuffer[j]._update_r = before;
                        j++;
                        if (j < r.getJoints())
                        {
                            r._bcastRecvBuffer[j]._position_error = *((short *)(data+2));
                            r._bcastRecvBuffer[j]._torque_error = *((short *)(data+6));
                            r._bcastRecvBuffer[j]._update_r = before;
                        }
                        break;

                    case ICUBCANPROTO_PER_MC_MSG__VELOCITY:
                        // also receives the acceleration values.
                        r._bcastRecvBuffer[j]._speed_joint = *((short *)(data));
                        r._bcastRecvBuffer[j]._accel_joint = *((short *)(data+4));
                        r._bcastRecvBuffer[j]._update_s = before;
                        j++;
                        if (j < r.getJoints())
                        {
                            r._bcastRecvBuffer[j]._speed_joint = *((short *)(data+2));
                            r._bcastRecvBuffer[j]._accel_joint = *((short *)(data+6));
                            r._bcastRecvBuffer[j]._update_s = before;
                        }
                        break;

                    default:
                        break;
                    }
                }
            }
        }
    }
}

///
///
///
bool CanBusMotionControl::threadInit()
{
    CanBusResources& r = RES (system_resources);

    _writerequested = false;
    _noreply = false;

    r._error_status = true;

    previousRun=0;
    averagePeriod=0;
    averageThreadTime=0;
    currentRun=0;
    myCount=-1;
    lastReportTime=Time::now();

    return true;
}

void CanBusMotionControl::threadRelease()
{
    //doing nothing
}

void CanBusMotionControl:: run()
{
    CanBusResources& r = RES (system_resources);
    unsigned int i = 0;

    myCount++;
    double before = Time::now();
    currentRun=before;
    if (myCount>0)
        averagePeriod+=(currentRun-previousRun)*1000;

    ////// HANDLE TIMEOUTS
    // check timeout on messages
    std::list<CanRequest> timedout;
    for(int j=0;j<r.requestsQueue->getNJoints();j++)
        {
            for(int m=0;m<r.requestsQueue->getNMessages();m++)
            {
                ThreadFifo *fifo=r.requestsQueue->getFifo(j,m);
                ACE_ASSERT (fifo!=0);
                std::list<ThreadId>::iterator it=fifo->begin();
                std::list<ThreadId>::iterator end=fifo->end();
                while(it!=end)
                    {
                        //increase wait time
                        (*it).waitTime+=r._polling_interval;
                        if ((*it).waitTime>=r._timeout)
                            {
                                // ok we have it... this is a timed out message
                                CanRequest rq;
                                rq.joint=j;
                                rq.msg=m;
                                rq.threadId=(*it).id;
                                timedout.push_back(rq); //store this request, so we can wake up the waiting thread
                                it=fifo->erase(it); //it now points to the next element
                                yError("%s [%d] thread:%d msg:%d joint:%d timed out\n", 
                                        canDevName.c_str(),
                                        r._networkN,
                                        rq.threadId, rq.msg, rq.joint);

                                // ???
                                //logJointData(canDevName.c_str(),r._networkN,j,3,yarp::os::Value(1));
                            }
                        else
                            ++it;
                    }
            }
        }

    //now go through the list of timedout requests and wake up waiting threads
    std::list<CanRequest>::iterator it=timedout.begin();
    std::list<CanRequest>::iterator end=timedout.end();
    while(it!=end)
        {
            int tid=(*it).threadId;
            ThreadTable2 *t=threadPool->getThreadTable(tid);
            t->timeout(); //notify one message timedout
            ++it;
        }
    //////////////////////////////////////////////////////////////////
    // report error LOOP
    if ((currentRun-lastReportTime)>REPORT_PERIOD)
        {
            double avPeriod=getEstPeriod();
            double avThTime=getEstUsed();//averageThreadTime/myCount;
            unsigned int it=getIterations();
            resetStat();
            yDebug("%s [%d] thread ran %d times, req.dT:%d[ms], av.dT:%.2lf[ms] av.loopT :%.2lf[ms]\n", 
                    canDevName.c_str(),
                    r._networkN, 
                    it, 
                    r._polling_interval,
                    avPeriod,
                    avThTime);

            const char *can=canDevName.c_str();
            logNetworkData(can,r._networkN,10,yarp::os::Value(r._polling_interval));
            logNetworkData(can,r._networkN,11,yarp::os::Value((double)avPeriod));
            logNetworkData(can,r._networkN,12,yarp::os::Value((double)avThTime));

            if (r.iCanErrors)
                {
                    CanErrors errors;
                    r.iCanErrors->canGetErrors(errors);
                    yDebug(" Can Errors --  Device Rx:%u, Device Tx:%u, RxOvf: %u, TxOvf: %u, BusOff: %d -- Driver Fifo Rx:%u, Driver Fifo Tx:%u\n", 
                            errors.rxCanErrors,
                            errors.txCanErrors,
                            errors.rxCanFifoOvr,
                            errors.txCanFifoOvr,
                            errors.busoff,
                            errors.rxBufferOvr,
                            errors.txBufferOvr);

                    logNetworkData(can,r._networkN,5,yarp::os::Value(errors.rxCanErrors));
                    logNetworkData(can,r._networkN,6,yarp::os::Value(errors.txCanErrors));
                    
                    logNetworkData(can,r._networkN,7,yarp::os::Value((int)errors.rxCanFifoOvr));
                    logNetworkData(can,r._networkN,8,yarp::os::Value((int)errors.txCanFifoOvr));

                    logNetworkData(can,r._networkN,9,yarp::os::Value((int)errors.busoff));

                    logNetworkData(can,r._networkN,3,yarp::os::Value((int)errors.rxBufferOvr));
                    logNetworkData(can,r._networkN,4,yarp::os::Value((int)errors.txBufferOvr));
                }
            else
                {
                    yDebug("     Device has no ICanBusErr interface\n");
                }
                

            int j=0;
            /// reports board errors
            char tmp[255];
            char message[255];
            sprintf(message, "%s [%d] printing boards infos:\n", canDevName.c_str(), r._networkN);

            bool errorF=false;
            for (j=0; j<r._njoints ;j++)
            {
                if ( r._bcastRecvBuffer[j]._mainLoopOverflowCounter>0)
                {
                    errorF=true;
                    int addr=r._destinations[j/2];
                    yWarning("%s [%d] board %d MAIN LOOP TIME EXCEDEED %d TIMES!\n", canDevName.c_str(), r._networkN, addr,r._bcastRecvBuffer[j]._mainLoopOverflowCounter);
                    logJointData(canDevName.c_str(),r._networkN,j,18,yarp::os::Value((int)r._bcastRecvBuffer[j]._mainLoopOverflowCounter));                
                    r._bcastRecvBuffer[j]._mainLoopOverflowCounter=0;
                }
                else
                {
                    logJointData(canDevName.c_str(),r._networkN,j,18,yarp::os::Value(0));
                }
            }

            for (j=0; j<r._njoints ;j+=2)
                {
                    int addr=r._destinations[j/2];
                    if ( (r._bcastRecvBuffer[j]._canTxError>0)||(r._bcastRecvBuffer[j]._canRxError>0))
                        {
                            errorF=true;
                            sprintf(tmp, "Id:%d T:%u R:%u ", addr, r._bcastRecvBuffer[j]._canTxError, r._bcastRecvBuffer[j]._canRxError);
                            sprintf(message, "%s%s", message, tmp);

                            logJointData(canDevName.c_str(),r._networkN,j,14,yarp::os::Value((int)r._bcastRecvBuffer[j]._canTxError));
                            logJointData(canDevName.c_str(),r._networkN,j,15,yarp::os::Value((int)r._bcastRecvBuffer[j]._canRxError));
                        }
                    else
                    {
                        logJointData(canDevName.c_str(),r._networkN,j,14,yarp::os::Value(0));
                        logJointData(canDevName.c_str(),r._networkN,j,15,yarp::os::Value(0));
                    }
                }
            if (!errorF)
                {
                    sprintf(tmp, "None");
                    sprintf(message, "%s%s", message, tmp);
                }

            yDebug("%s\n", message);

            //Check statistics on boards
            for (j=0; j<r._njoints ;j++)
                {
                    double lastRecv = r._bcastRecvBuffer[j]._update_e;

                    logJointData(canDevName.c_str(),r._networkN,j,2,yarp::os::Value((int)(currentRun-lastRecv)));

                    if ( (currentRun-lastRecv)>BCAST_STATUS_TIMEOUT)
                    {
                            int ch=j%2;
                            int addr=r._destinations[j/2];
                            yWarning("%s [%d] have not heard from board %d (channel %d) since %.2lf seconds\n", 
                                    canDevName.c_str(),
                                    r._networkN, 
                                    addr, ch, currentRun-lastRecv);

                            logJointData(canDevName.c_str(),r._networkN,j,3,yarp::os::Value(1));
                    }
                    else
                    {
                        logJointData(canDevName.c_str(),r._networkN,j,3,yarp::os::Value(0));
                    }
                }

            //Check position update frequency
            for(j=0; j<r._njoints; j++)
                {
                    double dT;
                    double max;
                    double min;
                    int it;
                    r._bcastRecvBuffer[j]._position_joint.getStats(it, dT, min, max);
                    r._bcastRecvBuffer[j]._position_joint.resetStats();

                    logJointData(canDevName.c_str(),r._networkN,j,5,yarp::os::Value(max));

                    double POS_LATENCY_WARN_THR=averagePeriod*1.5;
                    if (max>POS_LATENCY_WARN_THR)
                    {
                        yWarning("%s [%d] jnt %d, warning encoder latency above threshold (lat: %.2lf [ms], received %d msgs)\n",
                                canDevName.c_str(),
                                r._networkN,
                                j, 
                                max,
                                it);
                    }

                    if (it<1)
                    {
                        yWarning("%s [%d] joint %d, warning not enough encoder messages (received %d msgs)\n",
                                canDevName.c_str(),
                                r._networkN,
                                j, 
                                it);

                        logJointData(canDevName.c_str(),r._networkN,j,4,yarp::os::Value(1));
                    }
                    else
                    {
                        logJointData(canDevName.c_str(),r._networkN,j,4,yarp::os::Value(0));
                    }
                }

            ///////////////////check analog
            std::list<TBR_AnalogSensor *>::iterator analogIt=analogSensors.begin();
            while(analogIt!=analogSensors.end())
            {
                TBR_AnalogSensor *pAnalog=(*analogIt);
                if (pAnalog)
                {
                    unsigned int sat;
                    unsigned int err;
                    unsigned int tout; 
                    pAnalog->getCounters(sat, err, tout);

                    const char* devName=canDevName.c_str();
                    int id=pAnalog->getId();

                    #ifdef _USE_INTERFACEGUI
                    logAnalogData(devName,r._networkN,id,3,yarp::os::Value((int)sat));
                    logAnalogData(devName,r._networkN,id,4,yarp::os::Value((int)err));
                    logAnalogData(devName,r._networkN,id,5,yarp::os::Value((int)tout));
                    #endif

                    if (sat+err+tout!=0)
                    {
                        yWarning("%s [%d] analog %s saturated:%u errors: %u timeout:%u\n",
                                devName,
                                r._networkN,
                                pAnalog->getDeviceId().c_str(),
                                sat, err, tout);
                    }
                    pAnalog->resetCounters();
                }
                analogIt++;
            }

            myCount=0;
            lastReportTime=currentRun;
            averagePeriod=0;
            averageThreadTime=0;
        }

    //DEBUG_FUNC("CanBusMotionControl::thread running [%d]: wait\n", mycount);
    _mutex.wait ();
    //DEBUG_FUNC("posted\n");

    if (r.read () != true)
        r.printMessage("%s [%d] CAN: read failed\n", canDevName.c_str(), r._networkN);

    // handle all broadcast messages.
    // (class 1, 8 bits of the ID used to define the message type and source address).

    handleBroadcasts();

    std::list<TBR_AnalogSensor *>::iterator analogIt=analogSensors.begin();
    while(analogIt!=analogSensors.end())
    {
        TBR_AnalogSensor *pAnalog=(*analogIt);
        if (pAnalog)
        {
            if (!pAnalog->handleAnalog(system_resources))
            {
                yWarning("%s [%d] analog sensor received unexpected class 0x03 messages\n", canDevName.c_str(), r._networkN);
            }
        }
        else
            yWarning("Got null pointer this is unusual\n");
            
        analogIt++;
    }
 
    //
    // handle class 0 messages - polling messages.
    // (class 0, 8 bits of the ID used to represent the source and destination).
    // the first byte of the message is the message type and motor number (0 or 1).
    //
    if (r.requestsQueue->getPending()>0)
        {
            DEBUG_FUNC("There are %d pending messages, read msgs: %d\n", 
                  r.requestsQueue->getPending(), r._readMessages);
            for (i = 0; i < r._readMessages; i++)
                {
                    unsigned char *msgData;

                    CanMessage& m = r._readBuffer[i];
                    msgData=m.getData();      

                    if (getClass(m) == 0) /// class 0 msg.
                        {
                            PRINT_CAN_MESSAGE("Received \n", m);
                            /// legitimate message directed here, checks whether replies to any message.
                            int j=getJoint(m,r._destInv); //get joint from message
                            int id=r.requestsQueue->pop(j, msgData[0]);
                            if(id==-1)
                                {
                                    yWarning("%s [%d] Received message but no threads waiting for it. (id: 0x%x, Class:%d MsgData[0]:%d)\n ", canDevName.c_str(), r._networkN, m.getId(), getClass(m), msgData[0]);
                                    continue;
                                }
                            ThreadTable2 *t=threadPool->getThreadTable(id);
                            if (t==0)
                                {
                                    yWarning("Asked a bad thread id, this is probably a bug, check threadPool\n");
                                    continue;
                                }
                            DEBUG_FUNC("Pushing reply\n");
                            //push reply to thread's list of replies
                            if (!t->push(m))
                                yError("error while pushing a reply, this is probably an error\n");
                        }
                }
        }
    else
        {
            //DEBUG_FUNC("Thread loop: no pending messages\n");
        }

    //    counter ++;
    /*if (counter > r._timeout)
      {
      /// complains.
      r.printMessage("CAN: timeout - still %d messages unacknowledged\n", remainingMsgs);
      r._error_status = false;
      }*/

    r._echoMessages = 0; //echo buffer cleanup

    _mutex.post ();

    double now = Time::now();
    averageThreadTime+=(now-before)*1000;
    previousRun=before; //save last run time
}


    // ControlMode
bool CanBusMotionControl::getControlModesRaw(int *v)
{
    DEBUG_FUNC("Calling GET_CONTROL_MODES\n");
    CanBusResources& r = RES(system_resources);
    int i;
    int temp;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++)
    {
        temp = int(r._bcastRecvBuffer[i]._controlmodeStatus);
        v[i]=from_modeint_to_modevocab(temp);
    }
    _mutex.post();
    return true;
}
/*
//@@@ TO BE REMOVED LATER (AFTER INCLUDING FIRMWARE_SHARED)
#ifndef icubCanProto_controlmode_calibration
#define icubCanProto_controlmode_calibration 0x060
#endif

#ifndef icubCanProto_controlmode_forceIdle
#define icubCanProto_controlmode_forceIdle 0x09
#endif

#ifndef icubCanProto_interactionmode_stiff
#define icubCanProto_interactionmode_stiff 0x00
#endif 

#ifndef icubCanProto_interactionmode_compliant
#define icubCanProto_interactionmode_compliant 0x01
#endif

#ifndef icubCanProto_interactionmode_unknownError
#define icubCanProto_interactionmode_unknownError 0xFF
#endif
*/
//---------------------------------------------------------

unsigned char  CanBusMotionControl::from_interactionvocab_to_interactionint (int interactionvocab)
{
    switch (interactionvocab)
    {
    case VOCAB_IM_STIFF:
        return icubCanProto_interactionmode_stiff;
        break;
    case VOCAB_IM_COMPLIANT:
        return icubCanProto_interactionmode_compliant;
        break;
    case VOCAB_IM_UNKNOWN:
        default:
        return icubCanProto_interactionmode_unknownError;
        break;
    }
}

int CanBusMotionControl::from_interactionint_to_interactionvocab (unsigned char interactionint)
{
    switch (interactionint)
    {
    case icubCanProto_interactionmode_stiff:
        return VOCAB_IM_STIFF;
        break;
    case icubCanProto_interactionmode_compliant:
        return VOCAB_IM_COMPLIANT;
        break;
    case icubCanProto_interactionmode_unknownError:
        default:
        return icubCanProto_interactionmode_unknownError;
        break;
    }
}


unsigned char CanBusMotionControl::from_modevocab_to_modeint (int modevocab)
{
    switch (modevocab)
    {
    case VOCAB_CM_IDLE:
        return icubCanProto_controlmode_idle;
        break;
    case VOCAB_CM_POSITION:
        return icubCanProto_controlmode_position;
        break;
    case VOCAB_CM_MIXED:
        return icubCanProto_controlmode_mixed;
        break;
    case VOCAB_CM_POSITION_DIRECT:
        return icubCanProto_controlmode_direct;
        break;
    case VOCAB_CM_VELOCITY:
        return icubCanProto_controlmode_velocity;
        break;
    case VOCAB_CM_TORQUE:
        return icubCanProto_controlmode_torque;
        break;
    case VOCAB_CM_IMPEDANCE_POS:
        return icubCanProto_controlmode_impedance_pos;
        break;
    case VOCAB_CM_IMPEDANCE_VEL:
        return icubCanProto_controlmode_impedance_vel;
        break;
    case VOCAB_CM_PWM:
        return  icubCanProto_controlmode_openloop;
        break;
    case VOCAB_CM_CURRENT:
        return  VOCAB_CM_UNKNOWN;
        yError("'VOCAB_CM_CURRENT' error condition detected");
        break;

    case VOCAB_CM_FORCE_IDLE: 
        return icubCanProto_controlmode_forceIdle;
        break;

    default:
        return VOCAB_CM_UNKNOWN;
        yError ("'VOCAB_CM_UNKNOWN' error condition detected");
        break;
    }
}

int CanBusMotionControl::from_modeint_to_modevocab (unsigned char modeint)
{
    switch (modeint)
    {
    case icubCanProto_controlmode_idle:
        return VOCAB_CM_IDLE;
        break;
    case icubCanProto_controlmode_position:
        return VOCAB_CM_POSITION;
        break;
    case icubCanProto_controlmode_mixed:
        return VOCAB_CM_MIXED;
        break;
    case icubCanProto_controlmode_direct:
        return VOCAB_CM_POSITION_DIRECT;
        break;
    case icubCanProto_controlmode_velocity:
        return VOCAB_CM_VELOCITY;
        break;
    case icubCanProto_controlmode_torque:
        return VOCAB_CM_TORQUE;
        break;
    case icubCanProto_controlmode_impedance_pos:
        return VOCAB_CM_IMPEDANCE_POS;
        break;
    case icubCanProto_controlmode_impedance_vel:
        return VOCAB_CM_IMPEDANCE_VEL;
        break;
    case icubCanProto_controlmode_openloop:
        return VOCAB_CM_PWM;
        break;

    //internal status
    case  icubCanProto_controlmode_hwFault:
        return VOCAB_CM_HW_FAULT;
        break;
    case  icubCanProto_controlmode_notConfigured:
        return VOCAB_CM_NOT_CONFIGURED;
        break;
    case  icubCanProto_controlmode_configured:
        return VOCAB_CM_CONFIGURED;
        break;
    case  icubCanProto_controlmode_unknownError:
        yError ("'icubCanProto_controlmode_unknownError' error condition detected");
        return VOCAB_CM_UNKNOWN;
        break;

    case  icubCanProto_controlmode_calibration:
/*  Commented out because generate duplicate case in switch, the previous one should be enough
 *  with this new version of protocol... ask RANDAZZ
    case  icubCanProto_calibration_type0_hard_stops:
    case  icubCanProto_calibration_type1_abs_sens_analog:
    case  icubCanProto_calibration_type2_hard_stops_diff:
    case  icubCanProto_calibration_type3_abs_sens_digital:
    case  icubCanProto_calibration_type4_abs_and_incremental:
*/
        return VOCAB_CM_CALIBRATING;
        break;

    default:
        yError ("'VOCAB_CM_UNKNOWN' error condition detected");
        return VOCAB_CM_UNKNOWN;
        break;
    }
}

bool CanBusMotionControl::getControlModeRaw(int j, int *v)
{
    CanBusResources& r = RES(system_resources);
    if (!(j>= 0 && j <= r.getJoints())) 
    {
        *v=VOCAB_CM_UNKNOWN;
        return false;
    }

    short s;

    DEBUG_FUNC("Calling GET_CONTROL_MODE\n");
    //_readWord16 (CAN_GET_CONTROL_MODE, j, s); 

    _mutex.wait();
    s = r._bcastRecvBuffer[j]._controlmodeStatus;
  
    *v=from_modeint_to_modevocab(s);

    _mutex.post();

    return true;
}

// IControl Mode 2
bool CanBusMotionControl::getControlModesRaw(const int n_joints, const int *joints, int *modes)
{
    DEBUG_FUNC("Calling GET_CONTROL_MODE MULTIPLE JOINTS \n");
    if (joints==0) return false;
    if (modes==0) return false;

    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < n_joints; i++)
    {
        getControlModeRaw(joints[i], &modes[i]);
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::setControlModeRaw(const int j, const int mode)
{
    if (!(j >= 0 && j <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (mode == VOCAB_CM_TORQUE && _MCtorqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; return false;}

    DEBUG_FUNC("Calling SET_CONTROL_MODE_RAW SINGLE JOINT\n");
    bool ret = true;

    int v = from_modevocab_to_modeint(mode);
    if (v==VOCAB_CM_UNKNOWN) return false;
    _writeByte8(ICUBCANPROTO_POL_MC_CMD__SET_CONTROL_MODE,j,v);

    int current_mode = VOCAB_CM_UNKNOWN;
    int timeout = 0;

    do
    {
        getControlModeRaw(j,&current_mode);
        if (current_mode==mode) {ret = true; break;}
        if (current_mode==VOCAB_CM_IDLE     && mode==VOCAB_CM_FORCE_IDLE) {ret = true; break;}
        if (current_mode==VOCAB_CM_HW_FAULT)
        {
            if (mode!=VOCAB_CM_FORCE_IDLE) {yError ("Unable to set the control mode of a joint (%s j:%d) in HW_FAULT", networkName.c_str(), j);}
            ret = true; break;
        }
        yarp::os::Time::delay(0.010);
        if (timeout >0) yWarning ("setControlModeRaw delay (%s j:%d), current mode: %s, requested: %s", networkName.c_str(), j, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(mode).c_str());
        timeout++;
    }
    while (timeout < 10);
    if (timeout>=10)
    {
        ret = false;
        yError ("100ms Timeout occured in setControlModeRaw (%s j:%d), current mode: %s, requested: %s", networkName.c_str(), j, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(mode).c_str());
    }

    return ret;
}

bool CanBusMotionControl::setControlModesRaw(const int n_joints, const int *joints, int *modes)
{
    DEBUG_FUNC("Calling SET_CONTROL_MODE_RAW MULTIPLE JOINTS\n");
    if (n_joints==0) return false;
    if (joints==0) return false;
    bool ret = true;
    for (int i=0;i<n_joints; i++)
    {
        if (modes[i] == VOCAB_CM_TORQUE && _MCtorqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        int v = from_modevocab_to_modeint(modes[i]);
        if (v==VOCAB_CM_UNKNOWN) ret = false;
        _writeByte8(ICUBCANPROTO_POL_MC_CMD__SET_CONTROL_MODE,joints[i],v);

        int current_mode = VOCAB_CM_UNKNOWN;
        int timeout = 0;
        do
        {
            getControlModeRaw(joints[i],&current_mode);
            if (current_mode==modes[i]) {ret = true; break;}
            if (current_mode==VOCAB_CM_IDLE)
            {
                if (modes[i]!=VOCAB_CM_FORCE_IDLE) {yError ("Unable to set the control mode of a joint (%s j:%d) in HW_FAULT", networkName.c_str(), joints[i]);}
                ret = true; break;
            }
            yarp::os::Time::delay(0.010);
            if (timeout >0) yWarning ("setControlModesRaw delay (%s j:%d), current mode: %s, requested: %s", networkName.c_str(), joints[i], yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
            timeout++;
        }
        while (timeout < 10);
        if (timeout>=10)
        {
            ret = false;
            yError ("100ms Timeout occured in setControlModesRaw(M) (%s j:%d), current mode: %s, requested: %s", networkName.c_str(), joints[i], yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
        }
    }

    return ret;
}

bool CanBusMotionControl::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    listOfKeys->addString("filterType");
    listOfKeys->addString("PWMLimit");
    return true;
}

bool CanBusMotionControl::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    val.clear();
    CanBusResources& res = RES(system_resources);
    if (key == "filterType")
    {
        Bottle& r = val.addList(); for (int i = 0; i< res.getJoints(); i++) { int tmp = 0; getFilterTypeRaw(i, &tmp);  r.addInt(tmp); }
        return true;
    }
    if (key == "PWMLimit")
    {
        Bottle& r = val.addList(); for (int i = 0; i< res.getJoints(); i++) { double tmp = 0; getPWMLimitRaw(i, &tmp);  r.addDouble(tmp); }
        return true;
    }
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool CanBusMotionControl::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    CanBusResources& r = RES(system_resources);
    size_t _njoints = r.getJoints();

    std::string s1 = val.toString();
    if (val.size() != _njoints)
    {
        yWarning("setRemoteVariable(): Protocol error %s", s1.c_str());
        return false;
    }

    if (key == "filterType")
    {
        for (int i = 0; i < r.getJoints(); i++)
        {
            int filter_type = val.get(i).asInt();
            this->setFilterTypeRaw(i, filter_type);
        }
        return true;
    }
    if (key == "PWMLimit")
    {
        for (int i = 0; i < r.getJoints(); i++)
        {
            double limit = val.get(i).asDouble();
            this->setPWMLimitRaw(i, (int)(limit));
        }
        return true;
    }
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return false;
}

bool CanBusMotionControl::getAxisNameRaw(int axis, std::string& name)
{
    CanBusResources& r = RES(system_resources);
    if (axis >= 0 && axis < r.getJoints())
    {
        name = r._jointNames[axis];
        return true;
    }
    else
    {
        name = "ERROR";
        return false;
    }
}

bool CanBusMotionControl::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type)
{
    CanBusResources& r = RES(system_resources);
    if (axis >= 0 && axis < r.getJoints())
    {
        //type = joint_type[axis];
        type = VOCAB_JOINTTYPE_REVOLUTE;
        return true;
    }
    else
    {
        return false;
    }
}

bool CanBusMotionControl::setControlModesRaw(int *modes)
{
    DEBUG_FUNC("Calling SET_CONTROL_MODE_RAW ALL JOINT\n");
    CanBusResources& r = RES(system_resources);
    bool ret = true;

    for (int i = 0; i < r.getJoints(); i++)
    {
        if (modes[i] == VOCAB_CM_TORQUE && _MCtorqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        int v = from_modevocab_to_modeint(modes[i]);
        if (v==VOCAB_CM_UNKNOWN) return false;
        _writeByte8(ICUBCANPROTO_POL_MC_CMD__SET_CONTROL_MODE,i,v);

        int current_mode = VOCAB_CM_UNKNOWN;
        int timeout = 0;
        do
        {
            getControlModeRaw(i,&current_mode);
            if (current_mode==modes[i]) {ret = true; break;}
            if (current_mode==VOCAB_CM_IDLE     && modes[i]==VOCAB_CM_FORCE_IDLE) {ret = true; break;}
            if (current_mode==VOCAB_CM_HW_FAULT)
            {
                if (modes[i]!=VOCAB_CM_FORCE_IDLE) {yError ("Unable to set the control mode of a joint (%s j:%d) in HW_FAULT", networkName.c_str(), i);}
                ret = true; break;
            }
            yarp::os::Time::delay(0.010);
            if (timeout >0) yWarning ("setControlModesRaw delay (%s j:%d), current mode: %s, requested: %s", networkName.c_str(), i, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
            timeout++;
        }
        while (timeout < 10);
        if (timeout>=10)
        {
            ret = false;
            yError ("100ms Timeout occured in setControlModesRaw (%s j:%d), current mode: %s, requested: %s", networkName.c_str(), i, yarp::os::Vocab::decode(current_mode).c_str(), yarp::os::Vocab::decode(modes[i]).c_str());
        }
    }

    return ret;
}

// return the number of controlled axes.
bool CanBusMotionControl::getAxes(int *ax)
{
    CanBusResources& r = RES(system_resources);
    *ax = r.getJoints();

    return true;
}

bool CanBusMotionControl::helper_setPosPidRaw (int axis, const Pid &pid)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;
    _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_P_GAIN, axis, S_16(pid.kp));
    _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_D_GAIN, axis, S_16(pid.kd));
    _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_I_GAIN, axis, S_16(pid.ki));
    _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_ILIM_GAIN, axis, S_16(pid.max_int));
    _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_OFFSET, axis, S_16(pid.offset));
    _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_SCALE, axis, S_16(pid.scale));
    _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_TLIM, axis, S_16(pid.max_output));
    _writeWord16Ex (ICUBCANPROTO_POL_MC_CMD__SET_POS_STICTION_PARAMS, axis, S_16(pid.stiction_up_val), S_16(pid.stiction_down_val), false);
    return true;
}

bool CanBusMotionControl::setPidRaw (const PidControlTypeEnum& pidtype, int axis, const Pid &pid)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    switch (pidtype)
    {
        case VOCAB_PIDTYPE_POSITION:
            helper_setPosPidRaw(axis,pid);
        break;
        case VOCAB_PIDTYPE_VELOCITY:
            helper_setVelPidRaw(axis,pid);
        break;
        case VOCAB_PIDTYPE_CURRENT:
            helper_setCurPidRaw(axis,pid);
        break;
        case VOCAB_PIDTYPE_TORQUE:
            helper_setTrqPidRaw(axis,pid);
        break;
        default:
            yError()<<"Invalid pidtype:"<<pidtype;
        break;
    }
    return true;
}

bool CanBusMotionControl::getImpedanceRaw (int axis, double *stiff, double *damp)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        //@@@ TODO: check here
        //*stiff = 0;
        //*damp = 0;
        return true;
    }
 
    CanBusResources& r = RES(system_resources);
    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_IMPEDANCE_PARAMS);
    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getImpedanceRaw: message timed out\n");
        //@@@ TODO: check here
        //*stiff = 0;
        //*damp = 0;
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        //@@@ TODO: check here
        //*stiff = 0;
        //*damp = 0;
        return false;
    }

    unsigned char *data;
    data=m->getData()+1;
    *stiff= *((short *)(data));
    data+=2;
    *damp= *((short *)(data)); 
    *damp/= 1000;

    t->clear();

    return true;
}

bool CanBusMotionControl::getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    CanBusResources& r = RES(system_resources);
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    _mutex.wait();
    // *** This method is implementented reading data without sending/receiving data from the Canbus ***
    *min_stiff=_axisImpedanceHelper->getImpedanceLimits()->get_min_stiff(); 
    *max_stiff=_axisImpedanceHelper->getImpedanceLimits()->get_max_stiff(); 
    *min_damp= _axisImpedanceHelper->getImpedanceLimits()->get_min_damp(); 
    *max_damp= _axisImpedanceHelper->getImpedanceLimits()->get_max_damp(); 
    int k=castToMapper(yarp::dev::ImplementTorqueControl::helper)->toUser(j);
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getImpedanceOffsetRaw (int axis, double *off)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        //@@@ TODO: check here
        //*off = 0;
        return true;
    }
 
    CanBusResources& r = RES(system_resources);
    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_IMPEDANCE_OFFSET);
    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

     if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getImpedanceOffset: message timed out\n");
        //@@@ TODO: check here
        //*off=0;
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        //@@@ TODO: check here
        //*off=0;
        return false;
    }

    unsigned char *data;
    data=m->getData()+1;
    *off= *((short *)(data));

    t->clear();

    return true;
}

bool CanBusMotionControl::setImpedanceRaw (int axis, double stiff, double damp)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("setImpedanceRaw\n");

    if (!ENABLED(axis))
        return true;

    CanBusResources& r = RES(system_resources);
    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_IMPEDANCE_PARAMS, axis);
        *((short *)(r._writeBuffer[0].getData()+1)) = S_16(stiff);
        *((short *)(r._writeBuffer[0].getData()+3)) = S_16(damp*1000);
        *((short *)(r._writeBuffer[0].getData()+5)) = S_16(0);
        *((char  *)(r._writeBuffer[0].getData()+7)) = 0;
        r._writeBuffer[0].setLen(8);
        r.writePacket();
    _mutex.post();

    //yDebug("stiffness is: %d \n", S_16(stiff));
    return true;
}

bool CanBusMotionControl::setTorqueSource (int axis, char board_id, char board_chan )
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("setTorqueSource\n");

    if (!ENABLED(axis))
        return true;

    CanBusResources& r = RES(system_resources);
    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_TORQUE_SOURCE, axis);
        *((char *)(r._writeBuffer[0].getData()+1)) = board_id;
        *((char *)(r._writeBuffer[0].getData()+2)) = board_chan;
        *((char *)(r._writeBuffer[0].getData()+3)) = 0;
        *((char *)(r._writeBuffer[0].getData()+4)) = 0;
        *((char *)(r._writeBuffer[0].getData()+5)) = 0;
        *((char *)(r._writeBuffer[0].getData()+6)) = 0;
        *((char *)(r._writeBuffer[0].getData()+7)) = 0;
        r._writeBuffer[0].setLen(8);
        r.writePacket();
    _mutex.post();

    return true;
}

bool CanBusMotionControl::setImpedanceOffsetRaw (int axis, double off)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("setImpedanceOffsetRaw\n");

    if (!ENABLED(axis))
        return true;

    CanBusResources& r = RES(system_resources);
    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_IMPEDANCE_OFFSET, axis);
        *((short *)(r._writeBuffer[0].getData()+1)) = S_16(off);
        r._writeBuffer[0].setLen(3);
        r.writePacket();
    _mutex.post();

    return true;
}

bool CanBusMotionControl::helper_getPosPidRaw (int axis, Pid *out)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    short s;
    short s2;

    DEBUG_FUNC("Calling GET_P_GAIN\n");
    _readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_P_GAIN, axis, s); out->kp = double(s);
    DEBUG_FUNC("Calling CAN_GET_D_GAIN\n");
    _readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_D_GAIN, axis, s); out->kd = double(s);
    DEBUG_FUNC("Calling CAN_GET_I_GAIN\n");
    _readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_I_GAIN, axis, s); out->ki = double(s);
    DEBUG_FUNC("Calling CAN_GET_ILIM_GAIN\n");
    _readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_ILIM_GAIN, axis, s); out->max_int = double(s);
    DEBUG_FUNC("Calling CAN_GET_OFFSET\n");
    _readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_OFFSET, axis, s); out->offset= double(s);
    DEBUG_FUNC("Calling CAN_GET_SCALE\n");
    _readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_SCALE, axis, s); out->scale = double(s);
    DEBUG_FUNC("Calling CAN_GET_TLIM\n");
    _readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_TLIM, axis, s); out->max_output = double(s);
    DEBUG_FUNC("Calling CAN_GET_POS_STICTION_PARAMS\n");
    _readWord16Ex (ICUBCANPROTO_POL_MC_CMD__GET_POS_STICTION_PARAMS, axis, s, s2 ); out->stiction_up_val = double(s); out->stiction_down_val = double(s2);
    DEBUG_FUNC("Get PID done!\n");
    
    return true;
}

bool CanBusMotionControl::getPidRaw (const PidControlTypeEnum& pidtype, int axis, Pid *pid)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    switch (pidtype)
    {
        case VOCAB_PIDTYPE_POSITION:
            helper_getPosPidRaw(axis,pid);
        break;
        case VOCAB_PIDTYPE_VELOCITY:
            helper_getVelPidRaw(axis,pid);
        break;
        case VOCAB_PIDTYPE_CURRENT:
            helper_getCurPidRaw(axis,pid);
        break;
        case VOCAB_PIDTYPE_TORQUE:
            helper_getTrqPidRaw(axis,pid);
        break;
        default:
            yError()<<"Invalid pidtype:"<<pidtype;
        break;
    }
    return true;
}

bool CanBusMotionControl::getPidsRaw (const PidControlTypeEnum& pidtype, Pid *pids)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        getPidRaw(pidtype,i,&pids[i]);
    }

    return true;
}

bool CanBusMotionControl::helper_setTrqPidRaw(int axis, const Pid &pid)
{
     /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("setTorquePidRaw\n");

    if (!ENABLED(axis))
        return true;

    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_TORQUE_PID, axis);
        *((short *)(r._writeBuffer[0].getData()+1)) = S_16(pid.kp);
        *((short *)(r._writeBuffer[0].getData()+3)) = S_16(pid.ki);
        *((short *)(r._writeBuffer[0].getData()+5)) = S_16(pid.kd);
        *((short *)(r._writeBuffer[0].getData()+7)) = S_16(pid.scale);
        r._writeBuffer[0].setLen(8);
        r.writePacket();
    _mutex.post();
    //yDebug(stderr, ">>>>>>>>>>>pid.kp set to %f\n",pid.kp);
    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_TORQUE_PIDLIMITS, axis);
        *((short *)(r._writeBuffer[0].getData()+1)) = S_16(pid.offset);
        *((short *)(r._writeBuffer[0].getData()+3)) = S_16(pid.max_output);
        *((short *)(r._writeBuffer[0].getData()+5)) = S_16(pid.max_int);
        *((short *)(r._writeBuffer[0].getData()+7)) = S_16(0);
        r._writeBuffer[0].setLen(8);
        r.writePacket();
    _mutex.post();
    _writeWord16Ex (ICUBCANPROTO_POL_MC_CMD__SET_TORQUE_STICTION_PARAMS, axis, S_16(pid.stiction_up_val), S_16(pid.stiction_down_val), false);
    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_MODEL_PARAMS, axis);
        *((short *)(r._writeBuffer[0].getData()+1)) = S_16(pid.kff);
        *((short *)(r._writeBuffer[0].getData()+3)) = S_16(0);
        *((short *)(r._writeBuffer[0].getData()+5)) = S_16(0);
        *((short *)(r._writeBuffer[0].getData()+7)) = S_16(0);
        r._writeBuffer[0].setLen(8);
        r.writePacket();
    _mutex.post();
    return true;
}

bool CanBusMotionControl::helper_getTrqPidRaw (int axis, Pid *out)
{
    DEBUG_FUNC("Calling CAN_GET_TORQUE_PID \n");

    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

 
 
    if (!ENABLED(axis))
    {
        //@@@ TODO: check here
        // value = 0;
        return true;
    }
 

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_TORQUE_PID);
    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getTorquePid: message timed out\n");
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    unsigned char *data;
    data=m->getData()+1;
    out->kp= *((short *)(data));
    data+=2;
    out->ki= *((short *)(data));
    data+=2;
    out->kd= *((short *)(data));
    data+=2;
    out->scale= *((char *)(data));

    t->clear();

    _mutex.wait();
    
    DEBUG_FUNC("Calling CAN_GET_TORQUE_PIDLIMITS\n");
   
    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_TORQUE_PIDLIMITS);
    r.writePacket();

    // ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getTorquePidLimits: message timed out\n");
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    m=t->get(0);
    if (m==0)
    {
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    data=m->getData()+1;
    out->offset= *((short *)(data));
    data+=2;
    out->max_output= *((short *)(data));
    data+=2;
    out->max_int= *((short *)(data));

    t->clear();

    _mutex.wait();
    
    DEBUG_FUNC("Calling CAN_GET_MODEL_PARAMS\n");
   
    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_MODEL_PARAMS);
    r.writePacket();

    // ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("CAN_GET_MODEL_PARAMS: message timed out\n");
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    m=t->get(0);
    if (m==0)
    {
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    data=m->getData()+1;
    out->kff= *((short *)(data));

    t->clear();

    DEBUG_FUNC("Calling CAN_GET_TORQUE_STICTION_PARAMS\n");
    short s1;
    short s2;
    _readWord16Ex (ICUBCANPROTO_POL_MC_CMD__GET_TORQUE_STICTION_PARAMS, axis, s1, s2 );
    out->stiction_up_val = double(s1); 
    out->stiction_down_val = double(s2);

    return true;
}

bool CanBusMotionControl::setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids)
{
    CanBusResources& r = RES(system_resources);
    if (pids==0) return false;

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        setPidRaw(pidtype, i, pids[i]);
    }

    return true;
}

/// cmd is a SingleAxis poitner with 1 double arg
bool CanBusMotionControl::setPidReferenceRaw (const PidControlTypeEnum& pidtype, int j, double ref)
{
    return NOT_YET_IMPLEMENTED("setPidReferenceRaw");
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::setPidReferencesRaw (const PidControlTypeEnum& pidtype, const double *refs)
{
    CanBusResources& r = RES(system_resources);
    if (refs==0) return false;

    int i=0;
    bool ret = true;
    for (i = 0; i < r.getJoints(); i++)
    {
        ret = ret & setPidReferenceRaw(pidtype,i,refs[i]);
    }

    return ret;
}

/// cmd is a SingleAxis poitner with 1 double arg
bool CanBusMotionControl::setRefTorqueRaw (int j, double ref_trq)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    //I'm sending a DWORD but the value MUST be clamped to S_16. Do not change.
    return _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_TORQUE, axis, S_16(ref_trq));
}

bool CanBusMotionControl::setRefTorquesRaw(const int n_joint, const int *joints, const double *t)
{
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        ret &= setRefTorqueRaw(joints[j], t[j]);
    }
    return ret;
}

/// cmd is a SingleAxis pointer with 1 double arg
bool CanBusMotionControl::getTorqueRaw (int j, double *trq)
{
    CanBusResources& r = RES(system_resources);
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    int k=castToMapper(yarp::dev::ImplementTorqueControl::helper)->toUser(j);
    _mutex.wait();
    *trq = double(r._bcastRecvBuffer[k]._torque);
    _mutex.post();

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::setRefTorquesRaw (const double *ref_trqs)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        //I'm sending a DWORD but the value MUST be clamped to S_16. Do not change.
        if (_writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_TORQUE, i, S_16(ref_trqs[i])) != true)
            return false;
    }

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getTorquesRaw (double *trqs)
{
    CanBusResources& r = RES(system_resources);
    bool ret = true;
    for (int j = 0; j < r.getJoints() && ret; j++)
    {
        ret &= getTorqueRaw(j, &trqs[j]);
    }
    return ret;
}

bool CanBusMotionControl::getTorqueRangeRaw (int j, double *min, double *max)
{
    CanBusResources& r = RES(system_resources);
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    _mutex.wait();
    // *** This method is implementented reading data without sending/receiving data from the Canbus ***
    *min=0; //set output to zero (default value)
    *max=0; //set output to zero (default value)
    int k=castToMapper(yarp::dev::ImplementTorqueControl::helper)->toUser(j);

    std::list<TBR_AnalogSensor *>::iterator it=analogSensors.begin();
    while(it!=analogSensors.end())
    {
        if (*it)
        {
            int id = (*it)->getId();
            int cfgId   = _axisTorqueHelper->getTorqueSensorId(k);
            if (cfgId == 0) 
            {
                *min=0;
                *max=0;
                break;
            }
            else if (cfgId==id)
            {
                *min=-_axisTorqueHelper->getMaximumTorque(k);
                *max=+_axisTorqueHelper->getMaximumTorque(k);
                break;
            }
        }
        it++;
    }
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getTorqueRangesRaw (double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getTorqueRangesRaw");
}

bool CanBusMotionControl::getPidErrorRaw(const PidControlTypeEnum& pidtype, int axis, double *err)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;

    _mutex.wait();
    switch (pidtype)
    {
        case VOCAB_PIDTYPE_POSITION:
        *err = double(r._bcastRecvBuffer[axis]._position_error);
        break;
        case VOCAB_PIDTYPE_TORQUE:
        *err = double(r._bcastRecvBuffer[axis]._torque_error);
        break;
        case VOCAB_PIDTYPE_VELOCITY:
        *err = 0; //not yet implemented
        NOT_YET_IMPLEMENTED("getPidErrorRaw VOCAB_PIDTYPE_VELOCITY");
        break;
        case VOCAB_PIDTYPE_CURRENT:
        *err = 0; //not yet implemented
        NOT_YET_IMPLEMENTED("getPidErrorRaw VOCAB_PIDTYPE_CURRENT");
        break;
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getTargetPositionRaw(int axis, double *ref)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;
    _mutex.wait();
    *(ref) = this->_ref_command_positions[axis];
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getTargetPositionsRaw(double *ref)
{
    CanBusResources& r = RES(system_resources);
    bool ret = true;
    for (int j = 0; j < r.getJoints() && ret; j++)
    {
        ret &= getTargetPositionRaw(j, &ref[j]);
    }
    return ret;
}

bool CanBusMotionControl::getTargetPositionsRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int j = 0; j < nj && ret; j++)
    {
        ret &= getTargetPositionRaw(jnts[j], &refs[j]);
    }
    return ret;
}

bool CanBusMotionControl::getRefVelocityRaw(int axis, double *ref)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;
    _mutex.wait();
    *(ref) = this->_ref_command_speeds[axis];
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getRefVelocitiesRaw(double *ref)
{
    CanBusResources& r = RES(system_resources);
    bool ret = true;
    for (int j = 0; j < r.getJoints() && ret; j++)
    {
        ret &= getRefVelocityRaw(j, &ref[j]);
    }
    return ret;
}

bool CanBusMotionControl::getRefVelocitiesRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int j = 0; j < nj && ret; j++)
    {
        ret &= getRefVelocityRaw(jnts[j], &refs[j]);
    }
    return ret;
}

bool CanBusMotionControl::getRefPositionRaw(int axis, double *ref)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;
    *ref = _ref_positions[axis];
    return true;
}

bool CanBusMotionControl::getRefPositionsRaw(double *ref)
{
    CanBusResources& r = RES(system_resources);
    bool ret = true;
    for (int j = 0; j < r.getJoints() && ret; j++)
    {
        ret &= getRefPositionRaw(j, &ref[j]);
    }
    return ret;
}

bool CanBusMotionControl::getRefPositionsRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int j = 0; j < nj && ret; j++)
    {
        ret &= getRefPositionRaw(jnts[j], &refs[j]);
    }
    return ret;
}

bool CanBusMotionControl::getPidErrorsRaw(const PidControlTypeEnum& pidtype, double *errs)
{
    CanBusResources& r = RES(system_resources);
    for (int i = 0; i < r.getJoints(); i++)
    {
        errs[i] = getPidErrorRaw(pidtype, i, &errs[i]);
    }
    return true;
}

bool CanBusMotionControl::getParameterRaw(int axis, unsigned int type, double* value)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        //@@@ TODO: check here
        // value = 0;
        return true;
    }
 
    CanBusResources& r = RES(system_resources);
    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, type);
    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getParameterRaw: message timed out\n");
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    unsigned char *data;
    data=m->getData()+1;
    *value= *((short *)(data));

    t->clear();

    return true;
}

bool CanBusMotionControl::getDebugParameterRaw(int axis, unsigned int index, double* value)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        //@@@ TODO: check here
        // value = 0;
        return true;
    }
 
    CanBusResources& r = RES(system_resources);
    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_DEBUG_PARAM);
    *((unsigned char *)(r._writeBuffer[0].getData()+1)) = index;
    r._writeBuffer[0].setLen(2);
    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getDebugParameterRaw: message timed out\n");
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        //@@@ TODO: check here
        // value=0;
        return false;
    }

    unsigned char *data;
    data=m->getData()+1;
    *value= *((short *)(data));

    t->clear();

    return true;
}

bool CanBusMotionControl::getDebugReferencePositionRaw(int axis, double* value)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    int val = 0;
    if (_readDWord (ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_POSITION, axis, val) == true)
        *value = double (val);
    else
        return false;

    return true;
}


bool CanBusMotionControl::getFirmwareVersionRaw (int axis, can_protocol_info const& icub_interface_protocol, firmware_info* fw_info)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        //@@@ TODO: check here
        // value = 0;
        return true;
    }
 
    CanBusResources& r = RES(system_resources);
    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    fw_info->network_name=this->canDevName;
    fw_info->joint=axis;
    fw_info->board_can_id=r._destinations[axis/2] & 0x0f;
    fw_info->network_number=r._networkN;

    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_FIRMWARE_VERSION);
    *((unsigned char *)(r._writeBuffer[0].getData()+1)) = (unsigned char)(icub_interface_protocol.major & 0xFF);
    *((unsigned char *)(r._writeBuffer[0].getData()+2)) = (unsigned char)(icub_interface_protocol.minor & 0xFF);
    r._writeBuffer[0].setLen(3);
    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getFirmwareVersion: message timed out\n");
        fw_info->board_type= 0;
        fw_info->fw_major= 0;
        fw_info->fw_version= 0;
        fw_info->fw_build= 0;
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        yError("getFirmwareVersion: message error\n");
        fw_info->board_type= 0;
        fw_info->fw_major= 0;
        fw_info->fw_version= 0;
        fw_info->fw_build= 0;
        return false;
    }

    unsigned char *data;
    data=m->getData()+1;
    fw_info->board_type= *((char *)(data));
    data+=1;
    fw_info->fw_major= *((char *)(data));
    data+=1;
    fw_info->fw_version= *((char *)(data));
    data+=1;
    fw_info->fw_build= *((char *)(data));
    data+=1;
    fw_info->can_protocol.major = *((char *)(data));
    data+=1;
    fw_info->can_protocol.minor = *((char *)(data));
    data+=1;
    fw_info->ack = *((char *)(data));
    t->clear();

    return true;
}

bool CanBusMotionControl::getPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double *ref)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
    {
        return false;
    }

    switch (pidtype)
    {
        case VOCAB_PIDTYPE_POSITION:
        {
            int value = 0;
            if (_readDWord (ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_POSITION, axis, value) == true)
            { *ref = double (value);}
            else
            { *ref =0; yError() << "Invalid _readDWord (ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_POSITION)"; return false; }
        }
        break;
        case VOCAB_PIDTYPE_VELOCITY:
        {
            *ref=0;
            NOT_YET_IMPLEMENTED("getPidReferenceRaw VOCAB_PIDTYPE_VELOCITY");
        }
        break;
        case VOCAB_PIDTYPE_CURRENT:
        {
            *ref=0;
            NOT_YET_IMPLEMENTED("getPidReferenceRaw VOCAB_PIDTYPE_CURRENT");
        }
        break;
        case VOCAB_PIDTYPE_TORQUE:
        {
            *ref=0;
            NOT_YET_IMPLEMENTED("getPidReferenceRaw VOCAB_PIDTYPE_TORQUE");
        }
        break;
        default:
        {
            *ref=0;
            yError()<<"Invalid pidtype:"<<pidtype;
        }
        break;
    }
    return true;
}

bool CanBusMotionControl::getPidReferencesRaw(const PidControlTypeEnum& pidtype, double *refs)
{
    bool ret = true;
    CanBusResources& r = RES(system_resources);
    if (refs==0) return false;

    for (int i = 0; i < r.getJoints(); i++)
    {
        ret &= getPidReferenceRaw(pidtype, i, &refs[i]);
    }
    return ret;
}

bool CanBusMotionControl::getPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double *err)
{
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool CanBusMotionControl::getPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, double *errs)
{
    return NOT_YET_IMPLEMENTED("getErrorLimits");
}

bool CanBusMotionControl::setPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimit");
}

bool CanBusMotionControl::setPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, const double *limits)
{
    return NOT_YET_IMPLEMENTED("setErrorLimits");
}

bool CanBusMotionControl::resetPidRaw(const PidControlTypeEnum& pidtype, int j)
{
    return NOT_YET_IMPLEMENTED("resetPid");
}

bool CanBusMotionControl::enablePidRaw(const PidControlTypeEnum& pidtype, int axis)
{
    return NOT_YET_IMPLEMENTED("enablePidRaw");
}

bool CanBusMotionControl::setPidOffsetRaw(const PidControlTypeEnum& pidtype, int axis, double v)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_OFFSET, axis, S_16(v));

}

bool CanBusMotionControl::setParameterRaw(int axis, unsigned int type, double value)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (type, axis, S_16(value));

}

bool CanBusMotionControl::setDebugParameterRaw(int axis, unsigned int index, double value)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    
    CanBusResources& r = RES(system_resources);
    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_DEBUG_PARAM, axis);
        *((unsigned char *)(r._writeBuffer[0].getData()+1)) = (unsigned char)(index & 0xFF);
        *((short *)(r._writeBuffer[0].getData()+2)) = S_16(value);
        *((short *)(r._writeBuffer[0].getData()+4)) = 0;
        *((short *)(r._writeBuffer[0].getData()+6)) = 0;
        r._writeBuffer[0].setLen(8);
        r.writePacket();
    _mutex.post();

    return true;
}

bool CanBusMotionControl::setDebugReferencePositionRaw(int axis, double value)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    //return _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_POSITION, axis, S_32(value));
    return _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_COMMAND_POSITION, axis, S_32(value));
}

bool CanBusMotionControl::getPidOutputRaw(const PidControlTypeEnum& pidtype, int axis, double *out)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;
    _mutex.wait();

    switch (pidtype)
    {
        case VOCAB_PIDTYPE_POSITION:
            *(out) = double(r._bcastRecvBuffer[axis]._pid_value);
        break;
        case VOCAB_PIDTYPE_VELOCITY:
            *(out) = double(r._bcastRecvBuffer[axis]._pid_value);
        break;
        case VOCAB_PIDTYPE_CURRENT:
            *(out) = double(r._bcastRecvBuffer[axis]._pid_value);
        break;
        case VOCAB_PIDTYPE_TORQUE:
            *(out) = double(r._bcastRecvBuffer[axis]._pid_value);
        break;
        default:
            yError()<<"Invalid pidtype:"<<pidtype;
        break;
    }
    
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs)
{
    CanBusResources& r = RES(system_resources);
    for (int i = 0; i < r.getJoints(); i++)
    {
        getPidOutput(pidtype, i, &outs[i]);
    }
    return true;
}

bool CanBusMotionControl::disablePidRaw(const PidControlTypeEnum& pidtype, int axis)
{
    return NOT_YET_IMPLEMENTED("disablePidRaw");
}

bool CanBusMotionControl::positionMoveRaw(int axis, double ref)
{
    /// prepare can message.
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED (axis))
    {
        // still fills the _ref_position structure.
        _ref_command_positions[axis] = ref;
        return true;
    }

    if (yarp::os::Time::now()-_last_position_move_time[axis]<MAX_POSITION_MOVE_INTERVAL) 
    {
        yWarning() << "Performance warning: You are using positionMove commands at high rate (<"<< MAX_POSITION_MOVE_INTERVAL*1000.0 <<" ms). Probably position control mode is not the right control mode to use.";
    }
    _last_position_move_time[axis] = yarp::os::Time::now();

    int mode = 0;
    getControlModeRaw(axis, &mode);
    if (mode != VOCAB_CM_POSITION &&
        mode != VOCAB_CM_MIXED    &&
        mode != VOCAB_CM_IMPEDANCE_POS &&
        mode != VOCAB_CM_IDLE)
    {
        yError() << "positionMoveRaw: skipping command because " << networkName.c_str() << " joint " << axis << "is not in VOCAB_CM_POSITION mode";
        return true;
    }

    _mutex.wait();

    r.startPacket();
    r.addMessage (ICUBCANPROTO_POL_MC_CMD__POSITION_MOVE, axis);

    _ref_command_positions[axis] = ref;
    *((int*)(r._writeBuffer[0].getData() + 1)) = S_32(_ref_command_positions[axis]);/// pos
    *((short*)(r._writeBuffer[0].getData() + 5)) = S_16(_ref_speeds[axis]);/// speed
    r._writeBuffer[0].setLen(7);

    r.writePacket();

    _mutex.post();

    return true;
}

bool CanBusMotionControl::positionMoveRaw(const double *refs)
{
    CanBusResources& r = RES(system_resources);
    int i = 0;
    if (refs == 0) return false;
    bool ret = true;
    for (i = 0; i < r.getJoints (); i++)
    {
        ret = ret & positionMoveRaw(i, refs[i]);
    }

    return ret;
}

bool CanBusMotionControl::relativeMoveRaw(int j, double delta)
{
    return NOT_YET_IMPLEMENTED("positionRelative");
}

bool CanBusMotionControl::relativeMoveRaw(const double *deltas)
{
    return NOT_YET_IMPLEMENTED("positionRelative");
}

/// check motion done, single axis.
bool CanBusMotionControl::checkMotionDoneRaw(int axis, bool *ret)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    short value;

    if (!_readWord16 (ICUBCANPROTO_POL_MC_CMD__MOTION_DONE, axis, value))
    {
        *ret=false;
        return false;
    }

    *ret= (value!=0);

    return true;
}

/// ret is a pointer to a bool
bool CanBusMotionControl::checkMotionDoneRaw (bool *val)
{
    CanBusResources& r = RES(system_resources);
    int i;
    short value;

    _mutex.wait();

    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    for (i = 0; i < r.getJoints(); i++)
    {
        if (ENABLED(i))
        {
            r.addMessage (id, i, ICUBCANPROTO_POL_MC_CMD__MOTION_DONE);
        }
    }

    if (r._writeMessages < 1)
    {
        _mutex.post();
        return false;
    }

    r.writePacket(); //write immediatly

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus())
        return false;

    int j;
    for (i = 0, j = 0; i < r.getJoints(); i++)
    {
        if (ENABLED(i))
        {
            CanMessage *m = t->getByJoint(i, r._destInv);
            if ( (m!=0) && (m->getId() != 0xffff) )
            {
                value = *((short *)(m->getData()+1));
                if (!value)
                {
                    *val=false;
                    return true;
                }
            }
            j++;
        }
    }

    t->clear();

    *val=true;
    return true;
}

bool CanBusMotionControl::calibrateRaw(int axis, unsigned int type, double p1, double p2, double p3)
{
    bool b = _writeByteWords16(ICUBCANPROTO_POL_MC_CMD__CALIBRATE_ENCODER, axis, type, S_16(p1), S_16(p2), S_16(p3));
    return b;
}

bool CanBusMotionControl::setCalibrationParametersRaw(int j, const CalibrationParameters& params)
{
    return calibrate2Raw(j, params.type, params.param1, params.param2, params.param3);
}

bool CanBusMotionControl::setRefSpeedRaw(int axis, double sp)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    sp /= 10.0; // encoder ticks per ms
    _ref_speeds[axis] = sp;
    return true;
}

bool CanBusMotionControl::setRefSpeedsRaw(const double *spds)
{
    CanBusResources& r = RES(system_resources);
    memcpy(_ref_speeds, spds, sizeof(double)* r.getJoints());
    int i;
    for (i = 0; i < r.getJoints(); i++)
        _ref_speeds[i] /= 10.0;

    return true;
}

bool CanBusMotionControl::setRefAccelerationRaw(int axis, double acc)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    /*
    *Acceleration is expressed in imp/s^2. 
    *We divided it by 1000^2 to express 
    *it in imp/ms^2
    */
    acc /= 1000.0;
    acc /= 1000.0;
    _ref_accs[axis] = acc;
    const short s = S_16(_ref_accs[axis]);

    return _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_ACCELER, axis, s);
}

bool CanBusMotionControl::setRefAccelerationsRaw(const double *accs)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        /*
        *Acceleration is expressed in imp/s^2. 
        *We divided it by 1000^2 to express 
        *it in imp/ms^2
        */
        double acc;
        acc = accs[i]/1000.0;
        acc /= 1000.0;
        _ref_accs[i] = acc;

        if (!_writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_DESIRED_ACCELER, i, S_16(_ref_accs[i])))
            return false;
    }

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefSpeedsRaw (double *spds)
{
    CanBusResources& r = RES(system_resources);

    memcpy(spds, _ref_speeds, sizeof(double)* r.getJoints());
    int i;
    for (i = 0; i < r.getJoints(); i++)
        spds[i] *= 10.0;

    return true;
}

bool CanBusMotionControl::getRefSpeedRaw (int axis, double *spd)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;
    *spd = _ref_speeds[axis] * 10.0;

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefAccelerationsRaw (double *accs)
{
    CanBusResources& r = RES(system_resources);
    int i;
    short value = 0;

    for(i = 0; i < r.getJoints(); i++)
    {
        if (_readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_ACCELER, i, value) == true) {
            _ref_accs[i] = accs[i] = double (value);
            accs[i] *= 1000.0;
            accs[i] *= 1000.0;
        }
        else
            return false;
    }

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefAccelerationRaw (int axis, double *accs)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    short value = 0;

    if (_readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_ACCELER, axis, value) == true)
    {
        _ref_accs[axis] = double (value);
        *accs = double(value) * 1000.0 * 1000.0;
    }
    else
        return false;

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefTorquesRaw (double *ref_trqs)
{
    CanBusResources& r = RES(system_resources);
    int i;
    short value = 0;

    for(i = 0; i < r.getJoints(); i++)
    {
        if (_readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_TORQUE, i, value) == true) {
            _ref_torques[i] = ref_trqs[i] = double (value);
        }
        else
            return false;
    }

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefTorqueRaw (int axis, double *ref_trq)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    short value = 0;

    if (_readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_DESIRED_TORQUE, axis, value))
    {
        _ref_torques[axis] = double (value);
        *ref_trq = double (value);
    }
    else
        return false;

    return true;
}

bool CanBusMotionControl::getMotorTorqueParamsRaw (int axis, MotorTorqueParameters *param)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    MotorTorqueParameters pzero;
    *param = pzero;

    if (!ENABLED(axis))
    {
        return true;
    }

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, ICUBCANPROTO_POL_MC_CMD__GET_MOTOR_PARAMS);

    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("getMotorTorqueParamsRaw: message timed out\n");
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        return false;
    }

    param->bemf = *((short *)(m->getData()+1)); //1&2
    param->bemf_scale = 0;
    param->ktau = *((short *)(m->getData()+4)); //4&5 
    param->ktau_scale = 0;
    //7 dummy
    return true;
}

bool CanBusMotionControl::getFilterTypeRaw(int j, int* type)
{
    const int axis = j;

    /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS - 1) * 2))
        return false;

    int val = 0;
    if (!_readByte8(ICUBCANPROTO_POL_MC_CMD__GET_TCFILTER_TYPE, axis, val))
    {
        return false;
    }
    *type = val;
    return true;
}

bool CanBusMotionControl::setFilterTypeRaw (int j, int type)
{
    const int axis = j;

     /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_TCFILTER_TYPE, axis);
        *((short *)(r._writeBuffer[0].getData()+1)) = S_16(type);
        r._writeBuffer[0].setLen(2);
        r.writePacket();
    _mutex.post();

    return true;
}

bool CanBusMotionControl::setMotorTorqueParamsRaw (int j, MotorTorqueParameters param)
{
    const int axis = j;

     /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_MOTOR_PARAMS, axis);
        *((short *)(r._writeBuffer[0].getData()+1)) = S_16(param.bemf);
        *((unsigned char  *)(r._writeBuffer[0].getData()+3)) = (unsigned char) (param.bemf_scale);
        *((short *)(r._writeBuffer[0].getData()+4)) = S_16(param.ktau);
        *((unsigned char  *)(r._writeBuffer[0].getData()+6)) = (unsigned char) (param.ktau_scale);
        *((unsigned char  *)(r._writeBuffer[0].getData()+7)) = (unsigned char) (0);
        r._writeBuffer[0].setLen(8);
        r.writePacket();
    _mutex.post();

    return true;
}

bool CanBusMotionControl::stopRaw(int j)
{
    bool ret = true;
    ret &= _writeNone  (ICUBCANPROTO_POL_MC_CMD__STOP_TRAJECTORY, j);
    return ret;
}

bool CanBusMotionControl::stopRaw()
{
    CanBusResources& r = RES(system_resources);
    const int n=r.getJoints();
    bool ret = true;

    for (int j=0; j<n; j++)
    {
       ret &= _writeNone  (ICUBCANPROTO_POL_MC_CMD__STOP_TRAJECTORY, j);
    }
    
    return ret;
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool CanBusMotionControl::velocityMoveRaw (int axis, double sp)
{
    /// prepare can message.
    CanBusResources& r = RES(system_resources);

    int mode = 0;
    getControlModeRaw(axis, &mode);
    if (mode != VOCAB_CM_VELOCITY &&
        mode != VOCAB_CM_MIXED    &&
        mode != VOCAB_CM_IMPEDANCE_VEL && 
        mode != VOCAB_CM_IDLE)
    {
        yError() << "velocityMoveRaw: skipping command because " << networkName.c_str() << " joint " << axis << "is not in VOCAB_CM_VELOCITY mode";
        return true;
    }

    _mutex.wait();

    r.startPacket();

    if (ENABLED (axis))
    {
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__VELOCITY_MOVE, axis);
        const int j = r._writeMessages - 1;
        _ref_command_speeds[axis] = sp / 1000.0;

        *((short*)(r._writeBuffer[j].getData() + 1)) = S_16(r._velShifts[axis] * _ref_command_speeds[axis]);/// speed

        if (r._velShifts[axis]*_ref_accs[axis]>1)
            *((short*)(r._writeBuffer[j].getData()+3)) = S_16(r._velShifts[axis]*_ref_accs[axis]);/// accel
        else
            *((short*)(r._writeBuffer[j].getData()+3)) = S_16(1);

        r._writeBuffer[j].setLen(5);
    }
    else
    {
        _ref_command_speeds[axis] = sp / 1000.0;
    }

    r.writePacket();

    _mutex.post();

    return true;
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool CanBusMotionControl::velocityMoveRaw (const double *sp)
{
    if (sp==0) return false;
    CanBusResources& r = RES(system_resources);
    bool ret = true;
    int j=0;
    for(j=0; j< r.getJoints(); j++)
    {
        ret = ret && velocityMoveRaw(j, sp[j]);
    }
    return ret;
}

bool CanBusMotionControl::setEncoderRaw(int j, double val)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_ENCODER_POSITION, axis, S_32(val));
}

bool CanBusMotionControl::setEncodersRaw(const double *vals)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        if (_writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_ENCODER_POSITION, i, S_32(vals[i])) != true)
            return false;
    }

    return true;
}

bool CanBusMotionControl::resetEncoderRaw(int j)
{
    return setEncoderRaw(j, 0);
}

bool CanBusMotionControl::resetEncodersRaw()
{
    int n=RES(system_resources).getJoints();
    double *tmp = new double [n];
    ACE_ASSERT (tmp != NULL);

    for(int i=0;i<n;i++)
        tmp[i]=0;

    bool ret= setEncodersRaw(tmp);

    delete [] tmp;

    return ret;
}

bool CanBusMotionControl::getEncodersRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    
    double stamp=0;
    for (i = 0; i < r.getJoints(); i++) {
        v[i] = double(r._bcastRecvBuffer[i]._position_joint._value);

        if (stamp<r._bcastRecvBuffer[i]._position_joint._stamp)
            stamp=r._bcastRecvBuffer[i]._position_joint._stamp;
    }

    stampEncoders.update(stamp);

    _mutex.post();
    return true;
}

Stamp CanBusMotionControl::getLastInputStamp()
{
    _mutex.wait();
    Stamp ret=stampEncoders;
    _mutex.post();
    return ret;
}

bool CanBusMotionControl::getEncoderRaw(int axis, double *v)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))return false;

    _mutex.wait();
    *v = double(r._bcastRecvBuffer[axis]._position_joint._value);
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getEncoderSpeedsRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++) {
        int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(i).jnt_Vel_estimator_shift));
        v[i] = (double(r._bcastRecvBuffer[i]._speed_joint)*1000.0)/vel_factor;
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getEncoderSpeedRaw(int j, double *v)
{
    CanBusResources& r = RES(system_resources);
    //ACE_ASSERT (j >= 0 && j <= r.getJoints());
    if (!(j >= 0 && j <= r.getJoints()))
        return false;

    _mutex.wait();
    int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(j).jnt_Vel_estimator_shift));
    *v = (double(r._bcastRecvBuffer[j]._speed_joint)*1000.0)/vel_factor;
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getEncoderAccelerationsRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++) {
        int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(i).jnt_Vel_estimator_shift));
        int acc_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(i).jnt_Acc_estimator_shift));
        v[i] = (double(r._bcastRecvBuffer[i]._accel_joint)*1000000.0)/(vel_factor*acc_factor);
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getEncoderAccelerationRaw(int j, double *v)
{
    CanBusResources& r = RES(system_resources);
    //ACE_ASSERT (j >= 0 && j <= r.getJoints());
    if (!(j >= 0 && j <= r.getJoints()))
        return false;

    _mutex.wait();
    int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(j).jnt_Vel_estimator_shift));
    int acc_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(j).jnt_Acc_estimator_shift));
    *v = (double(r._bcastRecvBuffer[j]._accel_joint)*1000000.0)/(vel_factor*acc_factor);
    _mutex.post();

    return true;
}


bool CanBusMotionControl::setMotorEncoderRaw(int m, const double val)
{
    return NOT_YET_IMPLEMENTED("setMotorEncoderRaw");
}

bool CanBusMotionControl::setMotorEncodersRaw(const double *vals)
{
    return NOT_YET_IMPLEMENTED("setMotorEncodersRaw");
}

bool CanBusMotionControl::resetMotorEncoderRaw(int m)
{
    return NOT_YET_IMPLEMENTED("resetMotorEncoderRaw");
}

bool CanBusMotionControl::resetMotorEncodersRaw()
{
    return NOT_YET_IMPLEMENTED("resetMotorEncodersRaw");
}

bool CanBusMotionControl::getMotorEncodersRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    
    double stamp=0;
    for (i = 0; i < r.getJoints(); i++) {
        v[i] = double(r._bcastRecvBuffer[i]._position_rotor._value);

        if (stamp<r._bcastRecvBuffer[i]._position_rotor._stamp)
            stamp=r._bcastRecvBuffer[i]._position_rotor._stamp;
    }

    stampEncoders.update(stamp);

    _mutex.post();
    return true;
}

bool CanBusMotionControl::getMotorEncoderRaw(int m, double *v)
{
    CanBusResources& r = RES(system_resources);
    if (!(m >= 0 && m <= r.getJoints()))return false;

    _mutex.wait();
    *v = double(r._bcastRecvBuffer[m]._position_rotor._value);
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getMotorEncodersTimedRaw(double *v, double *t)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    
    double stamp=0;
    for (i = 0; i < r.getJoints(); i++) {
        v[i] = double(r._bcastRecvBuffer[i]._position_rotor._value);
        t[i] = r._bcastRecvBuffer[i]._position_rotor._stamp;

        if (stamp<r._bcastRecvBuffer[i]._position_rotor._stamp)
            stamp=r._bcastRecvBuffer[i]._position_rotor._stamp;
    }

    stampEncoders.update(stamp);

    _mutex.post();
    return true;
}

bool CanBusMotionControl::getMotorEncoderTimedRaw(int m, double *v, double *t)
{
    CanBusResources& r = RES(system_resources);
    if (!(m >= 0 && m <= r.getJoints()))return false;

    _mutex.wait();
    *v = double(r._bcastRecvBuffer[m]._position_rotor._value);
    *t = r._bcastRecvBuffer[m]._position_rotor._stamp;
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getMotorEncoderCountsPerRevolutionRaw(int m, double *cpr)
{
    return NOT_YET_IMPLEMENTED("getMotorEncodersCountsPerRevolutionRaw");
}

bool CanBusMotionControl::setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr)
{
    return NOT_YET_IMPLEMENTED("setMotorEncodersCountsPerRevolutionRaw");
}

bool CanBusMotionControl::getNumberOfMotorEncodersRaw(int* m)
{
    CanBusResources& r = RES(system_resources);
    *m=r.getJoints();
    return true;
}

bool CanBusMotionControl::getNumberOfMotorsRaw(int* m)
{
    CanBusResources& r = RES(system_resources);
    *m=r.getJoints();
    return true;
}

bool CanBusMotionControl::getMotorEncoderSpeedsRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++) {
        int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(i).mot_Vel_estimator_shift));
        v[i] = (double(r._bcastRecvBuffer[i]._speed_rotor._value)*1000.0)/vel_factor;
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getMotorEncoderSpeedRaw(int m, double *v)
{
    CanBusResources& r = RES(system_resources);
    if (!(m >= 0 && m <= r.getJoints()))return false;

    _mutex.wait();
    int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(m).mot_Vel_estimator_shift));
    *v = (double(r._bcastRecvBuffer[m]._speed_rotor._value)*1000.0)/vel_factor;
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getMotorEncoderAccelerationsRaw(double *accs)
{
    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++) {
        int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(i).mot_Vel_estimator_shift));
        int acc_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(i).mot_Acc_estimator_shift));
        accs[i] = (double(r._bcastRecvBuffer[i]._accel_rotor._value)*1000000.0)/(vel_factor*acc_factor);
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getMotorEncoderAccelerationRaw(int m, double *acc)
{
    CanBusResources& r = RES(system_resources);
    if (!(m >= 0 && m <= r.getJoints()))return false;

    _mutex.wait();
    int vel_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(m).mot_Vel_estimator_shift));
    int acc_factor = (1 << int(_speedEstimationHelper->getEstimationParameters(m).mot_Acc_estimator_shift));
    *acc = (double(r._bcastRecvBuffer[m]._accel_rotor._value)*1000000.0)/(vel_factor*acc_factor);
    _mutex.post();
    return true;
}

bool CanBusMotionControl::disableAmpRaw(int axis)
{
    return DEPRECATED("disableAmpRaw");
}

bool CanBusMotionControl::enableAmpRaw(int axis)
{
    return DEPRECATED("enableAmpRaw");
}

// bcast
bool CanBusMotionControl::getCurrentsRaw(double *cs)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++)
    {
        cs[i] = double(r._bcastRecvBuffer[i]._current);
    }

    _mutex.post();
    return true;
}

// bcast currents
bool CanBusMotionControl::getCurrentRaw(int axis, double *c)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;

    _mutex.wait();
    *c = double(r._bcastRecvBuffer[axis]._current);
    _mutex.post();

    return true;
}

bool CanBusMotionControl::setMaxCurrentRaw(int axis, double v)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_CURRENT_LIMIT, axis, S_32(v));
}

bool CanBusMotionControl::getMaxCurrentRaw(int axis, double* v)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return NOT_YET_IMPLEMENTED("getMaxCurrentRaw");
    //int tmp=0;
    //v=0;
    //bool ret = _readDWord (ICUBCANPROTO_POL_MC_CMD__GET_CURRENT_LIMIT, axis, tmp);
    //if (ret) *v=tmp; 
    //return ret;
}


bool CanBusMotionControl::setVelocityShiftRaw(int axis, double shift)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_VEL_SHIFT, axis, S_16(shift));
}

bool CanBusMotionControl::getTemperatureRaw(int m, double* val)
{
    return NOT_YET_IMPLEMENTED("getTemperatureRaw");
}

bool CanBusMotionControl::getTemperaturesRaw(double *vals)
{
    return NOT_YET_IMPLEMENTED("getTemperaturesRaw");
}

bool CanBusMotionControl::getTemperatureLimitRaw(int m, double *temp)
{
    return NOT_YET_IMPLEMENTED("getTemperatureLimitRaw");
}

bool CanBusMotionControl::setTemperatureLimitRaw(int m, const double temp)
{
    return NOT_YET_IMPLEMENTED("setTemperatureLimitRaw");
}

bool CanBusMotionControl::getPeakCurrentRaw(int m, double *val)
{
    return NOT_YET_IMPLEMENTED("getPeakCurrentRaw");
}

bool CanBusMotionControl::setPeakCurrentRaw(int m, const double val)
{
    return NOT_YET_IMPLEMENTED("setPeakCurrentRaw");
}

bool CanBusMotionControl::getNominalCurrentRaw(int m, double *val)
{
    return NOT_YET_IMPLEMENTED("getNominalCurrentRaw");
}

bool CanBusMotionControl::setNominalCurrentRaw(int m, const double val)
{
    return NOT_YET_IMPLEMENTED("setNominalCurrentRaw");
}

bool CanBusMotionControl::getPWMRaw(int j, double* val)
{
    return NOT_YET_IMPLEMENTED("getPWMRaw");
}

bool CanBusMotionControl::getPWMLimitRaw(int j, double* val)
{
    if (!(j >= 0 && j <= (CAN_MAX_CARDS - 1) * 2))
        return false;

    short s = 0;
    bool ret = _readWord16(ICUBCANPROTO_POL_MC_CMD__GET_PWM_LIMIT, j, s);
    *val = s;
    return ret;
}

bool CanBusMotionControl::setPWMLimitRaw(int j, const double val)
{
    if (!(j >= 0 && j <= (CAN_MAX_CARDS - 1) * 2))
        return false;

    return _writeWord16(ICUBCANPROTO_POL_MC_CMD__SET_PWM_LIMIT, j, S_16(val));
}

bool CanBusMotionControl::getPowerSupplyVoltageRaw(int j, double* val)
{
    return NOT_YET_IMPLEMENTED("getPowerSupplyVoltageRaw");
}

bool CanBusMotionControl::setSpeedEstimatorShiftRaw(int axis, double jnt_speed, double jnt_acc, double mot_speed, double mot_acc)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;
    
    CanBusResources& r = RES(system_resources);
    _mutex.wait();
        r.startPacket();
        r.addMessage (ICUBCANPROTO_POL_MC_CMD__SET_SPEED_ESTIM_SHIFT, axis);
        *((unsigned char *)(r._writeBuffer[0].getData()+1)) = (unsigned char)(jnt_speed) & 0xFF;
        *((unsigned char *)(r._writeBuffer[0].getData()+2)) = (unsigned char)(jnt_acc)   & 0xFF;
        *((unsigned char *)(r._writeBuffer[0].getData()+3)) = (unsigned char)(mot_speed) & 0xFF;
        *((unsigned char *)(r._writeBuffer[0].getData()+4)) = (unsigned char)(mot_acc)   & 0xFF;
        r._writeBuffer[0].setLen(5);
        r.writePacket();
    _mutex.post();

    return true;
}

bool CanBusMotionControl::setVelocityTimeoutRaw(int axis, double timeout)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (ICUBCANPROTO_POL_MC_CMD__SET_VEL_TIMEOUT, axis, S_16(timeout));
}


bool CanBusMotionControl::calibrateRaw(int axis, double p)
{
    return _writeWord16 (ICUBCANPROTO_POL_MC_CMD__CALIBRATE_ENCODER, axis, S_16(p));
}

bool CanBusMotionControl::doneRaw(int axis)
{
    short value = 0;
    DEBUG_FUNC("Calling doneRaw for joint %d\n", axis);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!_readWord16 (ICUBCANPROTO_POL_MC_CMD__GET_CONTROL_MODE, axis, value))
    {
        return false;
    } 

    if (!(value & 0xf0))
    {
        return true;
    }

    return false;
}

bool CanBusMotionControl::setPrintFunction(int (*f) (const char *fmt, ...))
{
    yError("Calling obsolete function setPrintFunction\n");
    return true;
}

bool CanBusMotionControl::getAmpStatusRaw(int *st)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++)
    {
    //  WARNING
    //    Line changed with no idea about what it should do
    //    st[i] = short(r._bcastRecvBuffer[i]._fault);  
        st[i] = short(r._bcastRecvBuffer[i]._axisStatus);  
   
    }
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getAmpStatusRaw(int j, int *st)
{
    CanBusResources& r = RES(system_resources);

    _mutex.wait();
    st[j] = short(r._bcastRecvBuffer[j]._axisStatus);  
    _mutex.post();

    return true;
}

bool CanBusMotionControl::setLimitsRaw(int axis, double min, double max)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    bool ret=true;

    ret = ret && _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_MIN_POSITION, axis, S_32(min));
    ret = ret && _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_MAX_POSITION, axis, S_32(max));

    return ret;
}

bool CanBusMotionControl::getLimitsRaw(int axis, double *min, double *max)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;
    int iMin=0;
    int iMax=0;
    bool ret=true;

    ret = ret && _readDWord (ICUBCANPROTO_POL_MC_CMD__GET_MIN_POSITION, axis, iMin);
    ret = ret && _readDWord (ICUBCANPROTO_POL_MC_CMD__GET_MAX_POSITION, axis, iMax);

    *min=iMin;
    *max=iMax;

    return ret;
}


////////////////////////////////////////
//     Position control2 interface    //
////////////////////////////////////////

bool CanBusMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && positionMoveRaw(joints[j], refs[j]);
    }
    return ret;
}

bool CanBusMotionControl::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && relativeMoveRaw(joints[j], deltas[j]);
    }
    return ret;
}

bool CanBusMotionControl::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flag)
{
    bool ret = true;
    bool value = true;
    bool tot_value = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && checkMotionDoneRaw(joints[j], &value);
        tot_value &= value;
    }
    *flag = tot_value;
    return ret;
}

bool CanBusMotionControl::setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && setRefSpeedRaw(joints[j], spds[j]);
    }
    return ret;
}

bool CanBusMotionControl::setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && setRefAccelerationRaw(joints[j], accs[j]);
    }
    return ret;
}

bool CanBusMotionControl::getRefSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getRefSpeedRaw(joints[j], &spds[j]);
    }
    return ret;
}

bool CanBusMotionControl::getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getRefAccelerationRaw(joints[j], &accs[j]);
    }
    return ret;
}

bool CanBusMotionControl::stopRaw(const int n_joint, const int *joints)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && stopRaw(joints[j]);
    }
    return ret;
}

///////////// END Position Control 2 INTERFACE  //////////////////

////////////////////////////////////////
//     Velocity control2 interface    //
////////////////////////////////////////

bool CanBusMotionControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        ret = ret && velocityMoveRaw(joints[j], spds[j]);
    }
    return ret;
}

bool CanBusMotionControl::helper_setCurPidRaw(int j, const Pid &pid)
{
    // Our boards do not have a Velocity Pid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Current Pid");
}
bool CanBusMotionControl::helper_getCurPidRaw(int j, Pid *pid)
{
    // Our boards do not have a Velocity Pid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Current Pid");
}

bool CanBusMotionControl::helper_setVelPidRaw(int j, const Pid &pid)
{
    // Our boards do not have a Velocity Pid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}
bool CanBusMotionControl::helper_getVelPidRaw(int j, Pid *pid)
{
    // Our boards do not have a Velocity Pid
    return NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

///////////// END Velocity Control 2 INTERFACE  //////////////////

// IControlLimits
bool CanBusMotionControl::setVelLimitsRaw(int axis, double min, double max)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS - 1) * 2))
        return false;
    _mutex.wait();
    _max_vel_jnt_cmd[axis]=max;
    //min not implemented
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS - 1) * 2))
        return false;
    _mutex.wait();
    *max = _max_vel_jnt_cmd[axis];
    *min = 0;
    _mutex.post();
    return true;
}

// PositionDirect Interface
bool CanBusMotionControl::setPositionRaw(int j, double ref)
{
    CanBusResources& r = RES(system_resources);

    if (1/*fabs(ref-r._bcastRecvBuffer[j]._position_joint._value) < _axisPositionDirectHelper->getMaxHwStep(j)*/)
    {

        int mode = 0;
        getControlModeRaw(j, &mode);
        if (mode != VOCAB_CM_POSITION_DIRECT &&
            mode != VOCAB_CM_IDLE)
        {
            yError() << "setPositionRaw: skipping command because " << networkName.c_str() << " joint " << j << "is not in VOCAB_CM_POSITION_DIRECT mode";
            return true;
        }
        return _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_COMMAND_POSITION, j, S_32(ref));
    }
    else
    { 
        yWarning("skipping setPosition() on %s, joint %d (req: %.1f curr %.1f) \n", networkName.c_str() , j,
        _axisPositionDirectHelper->posE2A(ref, j),
        _axisPositionDirectHelper->posE2A(r._bcastRecvBuffer[j]._position_joint._value, j));
        //double saturated_cmd = _axisPositionDirectHelper->getSaturatedValue(j,r._bcastRecvBuffer[j]._position_joint._value,ref);
        //_writeDWord (CAN_SET_COMMAND_POSITION, j, S_32(saturated_cmd));
        return false;
    }
}

bool CanBusMotionControl::setPositionsRaw(const int n_joint, const int *joints, const double *refs)
{
    if (refs == 0) return false;
    if (joints == 0) return false;
    bool ret = true;

    for (int j = 0; j < n_joint; j++)
    {
        ret = ret & setPositionRaw(joints[j],refs[j]);
    }
    return ret;
}

bool CanBusMotionControl::setPositionsRaw(const double *refs)
{
    if (refs == 0) return false;
    CanBusResources& r = RES(system_resources);
    bool ret = true;

    for (int j = 0; j < r.getJoints(); j++)
    {
        ret = ret & setPositionRaw(j,refs[j]);
    }
    return ret;
}

bool CanBusMotionControl::isPidEnabledRaw(const PidControlTypeEnum& pidtype, int j, bool* enabled)
{
    return NOT_YET_IMPLEMENTED("isPidEnabled");
}

bool CanBusMotionControl::loadBootMemory()
{
    CanBusResources& r = RES(system_resources);

    bool ret=true;
    for(int j=0; j<r.getJoints(); j++)
    {
        ret=_writeNone(ICUBCANPROTO_POL_MC_CMD__READ_FLASH_MEM, j);
        if (!ret)
            return false;
    }

    return true;
}

bool CanBusMotionControl::saveBootMemory ()
{
    CanBusResources& r = RES(system_resources);

    bool ret=true;
    for(int j=0; j<r.getJoints(); j++)
    {
        ret=_writeNone(ICUBCANPROTO_POL_MC_CMD__WRITE_FLASH_MEM, j);
        if (!ret)
            return false;
    }

    return true;
}

/// sets the broadcast policy for a given board (don't need to be called twice).
/// the parameter is a 32-bit integer: bit X = 1 -> message X = active
/// e.g. 0x02 activates the broadcast of position information
/// 0x04 activates the broadcast of velocity ...
///
bool CanBusMotionControl::setBCastMessages (int axis, unsigned int v)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    // why S_32??
    return _writeDWord (ICUBCANPROTO_POL_MC_CMD__SET_BCAST_POLICY, axis, S_32(v));
}

inline bool CanBusMotionControl::ENABLED (int axis)
{
    CanBusResources& r = RES(system_resources);
    return ((r._destinations[axis/2] & CAN_SKIP_ADDR) == 0) ? true : false;
}

/// WRITE functions
/// sends a message without parameters
bool CanBusMotionControl::_writeNone (int msg, int axis)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        return true;
    }

    DEBUG_FUNC("Write None msg:%d axis:%d\n", msg, axis);
    _mutex.wait();

    r.startPacket();
    r.addMessage (msg, axis);

    // send immediatly
    r.writePacket();
    _mutex.post();

    return true;
}


/// to send a Word16.
bool CanBusMotionControl::_writeWord16 (int msg, int axis, short s)
{
    /// prepare Can message.
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("Writing Word16 msg:%d axis:%d\n", msg, axis);
    if (!ENABLED(axis))
        return true;

    _mutex.wait();

    r.startPacket();
    r.addMessage (msg, axis);

    *((short *)(r._writeBuffer[0].getData()+1)) = s;
    r._writeBuffer[0].setLen(3);

    r.writePacket();

    _mutex.post();

    /// hopefully ok...
    return true;
}

/// write a byte
bool CanBusMotionControl::_writeByte8 (int msg, int axis, int value)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("Writing byte msg:%d axis:%d\n", msg, axis);

    if (!ENABLED(axis))
        return true;

    _mutex.wait();

    r.startPacket();
    r.addMessage (msg, axis);

    *((int*)(r._writeBuffer[0].getData()+1)) = (value & 0xFF);
    r._writeBuffer[0].setLen(2);

    r.writePacket();

    _mutex.post();

    return true;
}

/// write a DWord
bool CanBusMotionControl::_writeDWord (int msg, int axis, int value)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("Writing DWord msg:%d axis:%d\n", msg, axis);

    if (!ENABLED(axis))
        return true;

    _mutex.wait();

    r.startPacket();
    r.addMessage (msg, axis);

    *((int*)(r._writeBuffer[0].getData()+1)) = value;
    r._writeBuffer[0].setLen(5);

    r.writePacket();

    _mutex.post();

    return true;
}

/// two shorts in a single Can message (both must belong to the same control card).
bool CanBusMotionControl::_writeWord16Ex (int msg, int axis, short s1, short s2, bool checkAxisEven)
{
    /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (checkAxisEven)
        ACE_ASSERT ((axis % 2) == 0);/// axis is even.

    DEBUG_FUNC("Write Word16Ex  msg:%d axis:%d\n", msg, axis);

    if (!ENABLED(axis))
        return true;

    _mutex.wait();

    r.startPacket();
    r.addMessage (msg, axis);

    *((short *)(r._writeBuffer[0].getData()+1)) = s1;
    *((short *)(r._writeBuffer[0].getData()+3)) = s2;
    r._writeBuffer[0].setLen(5);

    r.writePacket();

    _mutex.post();

    /// hopefully ok...
    return true;
}


bool CanBusMotionControl::_writeByteWords16 (int msg, int axis, unsigned char value, short s1, short s2, short s3)
{
    /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
        return true;

    _mutex.wait();

    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();

    r.addMessage (msg, axis);

    *((unsigned char *)(r._writeBuffer[0].getData()+1)) = value;
    *((short *)(r._writeBuffer[0].getData()+2)) = s1;
    *((short *)(r._writeBuffer[0].getData()+4)) = s2;
    *((short *)(r._writeBuffer[0].getData()+6)) = s3;
    r._writeBuffer[0].setLen(8);

    if (r._writeMessages < 1)
    {
        _mutex.post();
        return false;
    }

    r.writePacket(); //write now
    _mutex.post();
    return true;
}

/// READ functions
/// sends a message and gets a dword back.
/// 
bool CanBusMotionControl::_readDWord (int msg, int axis, int& value)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        value = 0;
        return true;
    }

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, msg);

    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("readDWord: message timed out\n");
        value = 0;
        return false;
    }

    CanMessage *m=t->get(0);
    if (m==0)
    {
        value=0;
        return false;
    }

    value = *((int *)(m->getData()+1));
    return true;
}

/// reads an array of double words.
bool CanBusMotionControl::_readDWordArray (int msg, double *out)
{
    CanBusResources& r = RES(system_resources);
    int i = 0;

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();

    for (i = 0; i < r.getJoints(); i++)
    {
        if (ENABLED(i))
        {
            r.addMessage (id, msg, i);
            //r.addMessage (msg, i);
        }
        else
            out[i] = 0;
    }

    if (r._writeMessages < 1)
    {
        _mutex.post();
        return false;
    }

    r.writePacket(); //write now

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus() || t->timedOut())
    {
        yError("readDWordArray: at least one message timed out\n");
        memset (out, 0, sizeof(double) * r.getJoints());
        return false;
    }

    int j;
    for (i = 0, j = 0; i < r.getJoints(); i++)
    {
        if (ENABLED(i))
        {
            CanMessage *m = t->getByJoint(i, r._destInv);
            if ( (m!=0) && (m->getId() != 0xffff) )
            {
                out[i] = *((int *)(m->getData()+1));
            }
            else
            {
                out[i]=0;
            }
            j++;
        }
    }
    t->clear();
    return true;
}

bool CanBusMotionControl::_readWord16 (int msg, int axis, short& value)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        value = short(0);
        return true;
    }

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    DEBUG_FUNC("readWord16: called from thread %d, axis %d msg %d\n", id, axis, msg);
    r.startPacket();
    r.addMessage (id, axis, msg);
    r.writePacket(); //write immediatly

    ThreadTable2 *t=threadPool->getThreadTable(id);
    DEBUG_FUNC("readWord16: going to wait for packet %d\n", id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();
    DEBUG_FUNC("readWord16: ok, wait done %d\n",id);

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("readWord16: message timed out\n");
        value = 0;
        return false;
    }

    CanMessage *m=t->get(0);

    if (m==0)
    {
        return false;
    }

    value = *((short *)(m->getData()+1));
    t->clear();
    return true;
}

bool CanBusMotionControl::_readByte8(int msg, int axis, int& value)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS - 1) * 2))
        return false;

    if (!ENABLED(axis))
    {
        value = short(0);
        return true;
    }

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    DEBUG_FUNC("_readByte8: called from thread %d, axis %d msg %d\n", id, axis, msg);
    r.startPacket();
    r.addMessage(id, axis, msg);
    r.writePacket(); //write immediatly

    ThreadTable2 *t = threadPool->getThreadTable(id);
    DEBUG_FUNC("_readByte8: going to wait for packet %d\n", id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();
    DEBUG_FUNC("_readByte8: ok, wait done %d\n", id);

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("_readByte8: message timed out\n");
        value = 0;
        return false;
    }

    CanMessage *m = t->get(0);

    if (m == 0)
    {
        return false;
    }

    value = *((char *)(m->getData() + 1));
    t->clear();
    return true;
}

bool CanBusMotionControl::_readWord16Ex (int msg, int axis, short& value1, short& value2)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!ENABLED(axis))
    {
        value1 = 0;
        value2 = 0;
        return true;
    }

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        value1 = 0;
        value2 = 0;
        return false;
    }

    DEBUG_FUNC("readWord16Ex: called from thread %d, axis %d msg %d\n", id, axis, msg);
    r.startPacket();
    r.addMessage (id, axis, msg);
    r.writePacket(); //write immediatly

    ThreadTable2 *t=threadPool->getThreadTable(id);
    DEBUG_FUNC("readWord16Ex: going to wait for packet %d\n", id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();
    DEBUG_FUNC("readWord16Ex: ok, wait done %d\n",id);

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        yError("readWord16: message timed out\n");
        value1 = 0;
        value2 = 0;
        return false;
    }

    CanMessage *m=t->get(0);

    if (m==0)
    {
        return false;
    }

    value1 = *((short *)(m->getData()+1));
    value2 = *((short *)(m->getData()+3));
    t->clear();
    return true;
}

/// reads an array.
bool CanBusMotionControl::_readWord16Array (int msg, double *out)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        yError("More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket ();

    for(i = 0; i < r.getJoints(); i++)
    {
        if (ENABLED(i))
        {
            r.addMessage (id, i, msg);
            //            r.addMessage (msg, i);
        }
        else
            out[i] = 0;
    }

    if (r._writeMessages < 1)
    {
        _mutex.post();
        return false;
    }


    r.writePacket();

    ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

    if (!r.getErrorStatus()||(t->timedOut()))
    {
        yError("readWord16Array: at least one message timed out\n");
        memset (out, 0, sizeof(double) * r.getJoints());
        return false;
    }

    int j;
    for (i = 0, j = 0; i < r.getJoints(); i++)
    {
        if (ENABLED(i))
        {
            CanMessage *m = t->getByJoint(i, r._destInv);
            if ( (m!=0) && (m->getId() != 0xffff) )
                out[i] = *((short *)(m->getData()+1));
            else
                out[i]=0;
            j++;
        }
    }

    t->clear();
    return true;
}

yarp::dev::DeviceDriver *CanBusMotionControl::createDevice(yarp::os::Searchable& config)
{
    //analogSensor
    std::string deviceType=config.find("device").asString().c_str();
    std::string deviceId=config.find("deviceid").asString().c_str();

    
    yDebug() <<"CanBusMotionControl::createDevice() looking for device " << deviceType << " with id " << deviceId;

    if (deviceType=="analog")
    {
        std::list<TBR_AnalogSensor *>::iterator it=analogSensors.begin();
        while(it!=analogSensors.end())
        {
            yDebug() <<"CanBusMotionControl::createDevice() inspecting "<< (*it)->getDeviceId(); 

            if ((*it)->getDeviceId()==deviceId)
            {
                yDebug() <<"CanBusMotionControl::createDevice() found valid device"; 
                return (*it);
            }
            it++;
        }
    }
    else
    {
        yDebug() <<"CanBusMotionControl::createDevice() only works for analog kind of devices\n";
    }

    return 0;
}

bool CanBusMotionControl::getEncodersTimedRaw(double *v, double *t)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    
    double stamp=0;
    for (i = 0; i < r.getJoints(); i++) {
        v[i] = double(r._bcastRecvBuffer[i]._position_joint._value);
        t[i] = r._bcastRecvBuffer[i]._position_joint._stamp;

        if (stamp<r._bcastRecvBuffer[i]._position_joint._stamp)
            stamp=r._bcastRecvBuffer[i]._position_joint._stamp;
    }

    stampEncoders.update(stamp);

    _mutex.post();
    return true;
}

bool CanBusMotionControl::getEncoderTimedRaw(int axis, double *v, double *t)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))return false;

    _mutex.wait();
    *v = double(r._bcastRecvBuffer[axis]._position_joint._value);
    *t = r._bcastRecvBuffer[axis]._position_joint._stamp;
    _mutex.post();

    return true;
}


// IInteractionMode
bool CanBusMotionControl::getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode)
{
    DEBUG_FUNC("Calling GET_INTERACTION_MODE SINGLE JOINT\n");
    CanBusResources& r = RES(system_resources);
    int temp;
    _mutex.wait();

    temp = int(r._bcastRecvBuffer[axis]._interactionmodeStatus);
    *mode=(yarp::dev::InteractionModeEnum)from_interactionint_to_interactionvocab(temp);
    
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    DEBUG_FUNC("Calling GET_INTERACTION_MODE MULTIPLE JOINTS \n");
    if (joints==0) return false;
    if (modes==0) return false;

    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < n_joints; i++)
    {
        getInteractionModeRaw(joints[i], &modes[i]);
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    DEBUG_FUNC("Calling GET_INTERACTION_MODE ALL JOINTS \n");
    CanBusResources& r = RES(system_resources);
    int i;
    int temp;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++)
    {
        temp = int(r._bcastRecvBuffer[i]._interactionmodeStatus);
        modes[i]=(yarp::dev::InteractionModeEnum)from_interactionint_to_interactionvocab(temp);
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum mode)
{
    if (!(j >= 0 && j <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG_FUNC("Calling SET_INTERACTION_MODE RAW\n");

    if (mode == VOCAB_IM_COMPLIANT && _MCtorqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; return false;}
    int v = from_interactionvocab_to_interactionint(mode);
    if (v==icubCanProto_interactionmode_unknownError) return false;
    _writeByte8(CAN_SET_INTERACTION_MODE,j,v);

    return true;
}

bool CanBusMotionControl::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
    DEBUG_FUNC("Calling SET_INTERACTION_MODE_RAW MULTIPLE JOINTS\n");
    if (n_joints==0) return false;
    if (joints==0) return false;
    bool ret = true;
    for (int i=0;i<n_joints; i++)
    {
        ret = ret && setInteractionModeRaw(joints[i],modes[i]);
    }
    return ret;
}

bool CanBusMotionControl::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
    DEBUG_FUNC("Calling SET_CONTROL_MODE_RAW ALL JOINT\n");
    CanBusResources& r = RES(system_resources);

    for (int i = 0; i < r.getJoints(); i++)
    {
       if (modes[i] == VOCAB_IM_COMPLIANT && _MCtorqueControlEnabled == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

       int v = from_interactionvocab_to_interactionint(modes[i]);
       if (v==icubCanProto_interactionmode_unknownError) return false;
       _writeByte8(CAN_SET_INTERACTION_MODE,i,v);
    }

    return true;
}

//PWM interface
bool CanBusMotionControl::setRefDutyCycleRaw(int j, double v)
{
    if (!(j >= 0 && j <= (CAN_MAX_CARDS - 1) * 2))
        return false;

    return _writeWord16(ICUBCANPROTO_POL_MC_CMD__SET_OPENLOOP_PARAMS, j, S_16(v));
}

bool CanBusMotionControl::setRefDutyCyclesRaw(const double *v)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        setRefDutyCycleRaw(i, v[i]);
    }

    return true;
}

bool CanBusMotionControl::getRefDutyCycleRaw(int j, double *v)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS - 1) * 2))
        return false;

    short int value = 0;
    if (_readWord16(ICUBCANPROTO_POL_MC_CMD__GET_OPENLOOP_PARAMS, axis, value) == true)
        *v = double(value);
    else
        return false;

    return true;
}

bool CanBusMotionControl::getRefDutyCyclesRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    if (v == 0) return false;
    int i = 0;
    bool ret = true;
    for (i = 0; i < r.getJoints(); i++)
    {
        ret = ret & getRefDutyCycleRaw(i, &v[i]);
    }
    return ret;
}

bool CanBusMotionControl::getDutyCycleRaw(int j, double *v)
{
    CanBusResources& r = RES(system_resources);
    if (!(j >= 0 && j <= r.getJoints()))
        return false;
    _mutex.wait();
    *(v) = double(r._bcastRecvBuffer[j]._pid_value);
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getDutyCyclesRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++)
    {
        v[i] = double(r._bcastRecvBuffer[i]._pid_value);
    }

    _mutex.post();
    return true;
}

// Current interface
/*bool CanBusMotionControl::getCurrentRaw(int j, double *t)
{
return NOT_YET_IMPLEMENTED("getCurrentRaw");
}

bool CanBusMotionControl::getCurrentsRaw(double *t)
{
return NOT_YET_IMPLEMENTED("getCurrentsRaw");
}
*/

bool CanBusMotionControl::getCurrentRangeRaw(int j, double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getCurrentRangeRaw");
}

bool CanBusMotionControl::getCurrentRangesRaw(double *min, double *max)
{
    return NOT_YET_IMPLEMENTED("getCurrentRangesRaw");
}

bool CanBusMotionControl::setRefCurrentsRaw(const double *t)
{
    return NOT_YET_IMPLEMENTED("setRefCurrentsRaw");
}

bool CanBusMotionControl::setRefCurrentRaw(int j, double t)
{
    return NOT_YET_IMPLEMENTED("setRefCurrentRaw");
}

bool CanBusMotionControl::setRefCurrentsRaw(const int n_joint, const int *joints, const double *t)
{
    return NOT_YET_IMPLEMENTED("setRefCurrentsRaw");
}

bool CanBusMotionControl::getRefCurrentsRaw(double *t)
{
    return NOT_YET_IMPLEMENTED("getRefCurrentsRaw");
}

bool CanBusMotionControl::getRefCurrentRaw(int j, double *t)
{
    return NOT_YET_IMPLEMENTED("getRefCurrentRaw");
}

