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
#include <yarp/String.h>

//the following activates the DEBUG macro in canControlUtil.h
//#define CAN_DEBUG
//#define CANBUSMC_DEBUG

//#define USE_CANBACKDOOR //enable/disable access to canbus from a port

#include "ThreadTable2.h"
#include "ThreadPool2.h"

#include <string>
#include <iostream>
#include <string.h>

/// specific to this device driver.
#include "CanBusMotionControl.h"

#include "can_string_generic.h"
/// get the message types from the DSP code.
#include "messages.h"

#include "../src/ControlBoardInterfacesImpl.inl"

#include "canControlConstants.h"
#include "canControlUtils.h"

const int REPORT_PERIOD=6; //seconds
const double BCAST_STATUS_TIMEOUT=6; //seconds

//windows size for the circular buffers which are used in DSP-controller for velocity and acceleration estimation
#define WINDOW_SIZE_VEL 35.0
#define WINDOW_SIZE_ACC 55.0

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

#define __ENABLE_TORQUE__

inline void PRINT_CAN_MESSAGE(const char *str, CanMessage &m)
{
#ifdef CANBUSMC_DEBUG
    fprintf(stderr, "%s", str);
    fprintf(stderr, " S:%d R:%d Ch:%d M:%d\n", getSender(m), getRcp(m), m.getData()[0]&0x80, m.getData()[0]&0x7F);
#endif
}

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    fprintf(stderr, "%s not yet implemented for CanBusMotionControl\n", txt);

    return false;
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
    BCastElement _position;

    // msg 2
    short _pid_value;
    double _update_v;

    // msg 3
	short _axisStatus;
	char  _canStatus;
	char  _boardStatus;
	short  _controlmodeStatus;
    double _update_e;

    // msg 4
    short _current;
    short _position_error;
    double _update_c;

    // msg 5
    short _speed;
    short _accel;
    double _update_s;

    short _torque;
    double _update_t;
	
	// msg 6
	unsigned int _canTxError;
	// msg 7
    unsigned int _canRxError;

	int _address;

    BCastBufferElement () { zero (); }
    bool isFaultOk			 () { return (!_axisStatus ) ; }
    bool isFaultUndervoltage () { return _axisStatus & 0x01; }
    bool isFaultOverload     ()	{ return _axisStatus & 0x02; }
	bool isOverCurrent       ()	{ return _axisStatus & 0x08; }
    bool isFaultExternal	 () { return _axisStatus & 0x04; }
	bool isHallSensorError	 () { return _axisStatus & 0x10; }
	bool isAbsEncoderError	 () { return _axisStatus & 0x20; }
	bool isCanTxOverflow	 () { return _canStatus & 0x01; }
	bool isCanBusOff	     () { return _canStatus & 0x02; }
	bool isCanTxError	     () { return _canStatus & 0x04; }
	bool isCanRxError	     () { return _canStatus & 0x08; }
	bool isCanRxWarning	     () { return _canStatus & 0x10; }
	bool isCanRxOverrun 	 () { return _canStatus & 0x20; }
	bool isMainLoopOverflow  () { return _boardStatus & 0x01; }
	bool isOverTempCh1       () { return _boardStatus & 0x02; }
	bool isOverTempCh2       () { return _boardStatus & 0x04; } 
	bool isTempErrorCh1      () { return _boardStatus & 0x08; }
	bool isTempErrorCh2      () { return _boardStatus & 0x10; } 
	void ControlStatus       (int net, short controlmode,short addr) 
	{
		switch (controlmode)
		{
		case 	MODE_IDLE:
			printf ("[%d] board  %d MODE_IDLE \r\n", net, addr);
			break;
		case 	MODE_CONTROLLED:
			printf ("[%d] board %d MODE_CONTROLLED \r\n", net, addr);
			break;
		case 	MODE_CALIB:
			printf ("[%d] board  %d MODE_CALIB \r\n", net, addr);
			break;
		default:
			break;
		}
	}


    void zero (void)
    {
        _position._value = 0;
        _pid_value = 0;
        _current = 0;
	    _axisStatus=0;
	    _canStatus=0;
	    _boardStatus=0;
		_controlmodeStatus=0;
        _position_error = 0;
        _speed = 0;
        _accel = 0;

        _torque=42;

        _update_v = .0;
        _update_e = .0;
        _update_c = .0;
        _update_s = .0;

        _address=-1;
        _canTxError=0;
        _canRxError=0;
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
        fprintf(stderr, "%s", buffer);
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

    CanBuffer _readBuffer;/// read buffer.
    CanBuffer _writeBuffer;/// write buffer.
    CanBuffer _replyBuffer;/// reply buffer.

    BCastBufferElement *_bcastRecvBuffer;/// local storage for bcast messages.

    unsigned char _my_address;/// 
    unsigned char _destinations[CAN_MAX_CARDS];/// list of connected cards (and their addresses).
    int _velShifts[CAN_MAX_CARDS];/// list of velocity shift
    unsigned char *_destInv;
    int _njoints;/// number of joints (ncards * 2).

    bool _error_status;/// error status of the last packet.

    /// used to spy on can messages.
    int _filter;/// don't print filtered messages.

    char _printBuffer[16384];                   /// might be better with dynamic allocation.
    RequestsQueue *requestsQueue;
};
inline CanBusResources& RES(void *res) { return *(CanBusResources *)res; }

class CanBackDoor: public BufferedPort<yarp::sig::Vector>
{
    CanBusResources *bus;
    Semaphore *semaphore;
public:
    CanBackDoor()
    {
        bus=0;
    }

    void setUp(CanBusResources *p, Semaphore *sema)
    {
        semaphore=sema;
        bus=p;
    }

    void onRead(Bottle &b)
    {
        if (!semaphore)
            return;

        semaphore->wait();
        //RANDAZ_TODO: parse vector b

        if (!bus)
        {
           bus->startPacket();
           //RANDAZ_TODO: prepare message
           bus->_writeBuffer[0].setId(0x00);
           bus->_writeBuffer[0].getData()[0]=0x00;
           bus->_writeBuffer[0].getData()[1]=0x00;
           bus->_writeBuffer[0].setLen(2);
           bus->_writeMessages++;
           bus->writePacket();
        }
        semaphore->post();
    }
} backDoor;

AnalogSensor::AnalogSensor():
data(0)
{
	timeStamp=0;
	status=IAnalogSensor::AS_OK;
	useCalibration=0;
	scaleFactor=0;

    counterSat=0;
    counterError=0;
    counterTimeout=0;
}

AnalogSensor::~AnalogSensor()
{
    if (!data)
        delete data;
	if (!scaleFactor)
		delete scaleFactor;
}

int AnalogSensor::getState(int ch)
{
    return status;
}

bool AnalogSensor::open(int channels, AnalogDataFormat f, short bId, short useCalib)
{
    if (data)
        return false;
	if (scaleFactor)
		return false;

    data=new AnalogData(channels, channels+1);
	scaleFactor=new double[channels];
	int i=0;
	for (i=0; i<channels; i++) scaleFactor[i]=1;
    dataFormat=f;
    boardId=bId;
	useCalibration=useCalib;
	if (useCalibration==1 && dataFormat==AnalogSensor::ANALOG_FORMAT_16)
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


int AnalogSensor::getChannels()
{
    return data->size();
}

int AnalogSensor::read(yarp::sig::Vector &out)
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
 
bool AnalogSensor::calibrate(int ch, double v)
{
    return true;
}

bool AnalogSensor::decode16(const unsigned char *msg, int id, double *data)
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
            fprintf(stderr, "Warning, got unexpected class 0x3 msg(s)\n");
            return false;
            break;
        }
		//@@@DEBUG ONLY
		//fprintf(stderr, "   %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    }

    return true;
}

bool AnalogSensor::decode8(const unsigned char *msg, int id, double *data)
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
            fprintf(stderr, "Warning, got unexpected class 0x3 msg(s): groupId 0x%x\n", groupId);
            return false;
            break;
        }
    }
    return true;
}


bool AnalogSensor::handleAnalog(void *canbus)
{
    CanBusResources& r = RES (canbus);

    unsigned int i=0;
    const int _networkN=r._networkN;

    bool ret=true; //return true by default

    mutex.wait();

	double timeNow=Time::now();
    for (i = 0; i < r._readMessages; i++)
    {
        unsigned int len=0;
        unsigned int msgid=0;
        unsigned char *buff=0;
        CanMessage& m = r._readBuffer[i];
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

bool CanBusMotionControlParameters::fromConfig(yarp::os::Searchable &p)
{
    if (!p.check("GENERAL","section for general motor control parameters")) {
        fprintf(stderr, "Cannot understand configuration parameters\n");
        return false;
    }

    int i;
    int nj = p.findGroup("GENERAL").check("Joints",Value(1),
        "Number of degrees of freedom").asInt();
    alloc(nj);

    ///// CAN PARAMETERS
    Bottle& can = p.findGroup("CAN");

    if (can.check("CanForcedDeviceNum"))
    {
        _networkN=can.find("CanForcedDeviceNum").asInt();
    }
    else
        _networkN=can.check("CanDeviceNum",Value(-1),
        "numeric identifier of device").asInt();

    //    std::cout<<can.toString();
    if (_networkN<0)
    {
        fprintf(stderr, "Error: can network id not valid, skipping device\n");
        return false;
    }

    _my_address=can.check("CanMyAddress",Value(0),
                          "numeric identifier of my address").asInt();

    _polling_interval=can.check("CanPollingInterval",Value(20),
                                "polling period").asInt();
    
    _timeout=can.check("CanTimeout",Value(20),"timeout period").asInt();

    _txTimeout=can.check("CanTxTimeout", Value(20), "tx timeout").asInt();
    _rxTimeout=can.check("CanRxTimeout", Value(20), "rx timeout").asInt();

    // default values for CanTxQueueSize/CanRxQueueSize should be the 
    // maximum, difficult to pick a correct value, let the driver 
    // decide on this
    if (can.check("CanTxQueueSize"))
        _txQueueSize=can.find("CanTxQueueSize").asInt();
    else
        _txQueueSize=-1;

    if (can.check("CanRxQueueSize"))
        _rxQueueSize=can.find("CanRxQueueSize").asInt();
    else
        _rxQueueSize=-1;

    Bottle xtmp = can.findGroup("CanAddresses",
        "a list of numeric identifiers");
    for (i = 1; i < xtmp.size(); i++) {
        _destinations[i-1] = (unsigned char)(xtmp.get(i).asInt());
    }

    ////// GENERAL
    xtmp = p.findGroup("GENERAL").findGroup("AxisMap","a list of reordered indices for the axes");
    if (xtmp.size() != nj+1) {
        printf("AxisMap does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) _axisMap[i-1] = xtmp.get(i).asInt();
    xtmp = p.findGroup("GENERAL").findGroup("Encoder","a list of scales for the encoders");
    if (xtmp.size() != nj+1) {
        printf("Encoder does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) _angleToEncoder[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("GENERAL").findGroup("Zeros","a list of offsets for the zero point");
    if (xtmp.size() != nj+1) {
        printf("Zeros does not have the right number of entries\n");
        return false;
    }
    for (i = 1; i < xtmp.size(); i++) _zeros[i-1] = xtmp.get(i).asDouble();

    ////// PIDS
    int j=0;
    for(j=0;j<nj;j++)
    {
        char tmp[80];
        sprintf(tmp, "Pid%d", j); 
        xtmp = p.findGroup("PIDS","PID parameters").findGroup(tmp);
        _pids[j].kp = xtmp.get(1).asDouble();
        _pids[j].kd = xtmp.get(2).asDouble();
        _pids[j].ki = xtmp.get(3).asDouble();

        _pids[j].max_int = xtmp.get(4).asDouble();
        _pids[j].max_output = xtmp.get(5).asDouble();

        _pids[j].scale = xtmp.get(6).asDouble();
        _pids[j].offset = xtmp.get(7).asDouble();
    }

    /////// LIMITS
    xtmp = p.findGroup("LIMITS").findGroup("Currents",
        "a list of current limits");
    if (xtmp.size() != nj+1) {
        printf("Currents does not have the right number of entries\n");
        return false;
    }
    for(i=1;i<xtmp.size(); i++) _currentLimits[i-1]=xtmp.get(i).asDouble();

    xtmp = p.findGroup("LIMITS").findGroup("Max","a list of maximum angles (in degrees)");
    if (xtmp.size() != nj+1) {
        printf("Max does not have the right number of entries\n");
        return false;
    }
    for(i=1;i<xtmp.size(); i++) _limitsMax[i-1]=xtmp.get(i).asDouble();

    xtmp = p.findGroup("LIMITS").findGroup("Min","a list of minimum angles (in degrees)");
    if (xtmp.size() != nj+1) {
        printf("Min does not have the right number of entries\n");
        return false;
    }
    for(i=1;i<xtmp.size(); i++) _limitsMin[i-1]=xtmp.get(i).asDouble();

    /////// [VELOCITY]
    if (p.check("VELOCITY"))
        {
            /////// Shifts
            xtmp = p.findGroup("VELOCITY").findGroup("Shifts",
                                                     "a list of shifts to be used in the vmo control");

            if (xtmp.size() != nj+1) {
                fprintf(stderr, "[VELOCITY] Shifts do not have the right number of entries. Using default Shifts=4\n");
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
            xtmp = p.findGroup("VELOCITY").findGroup("Timeout",
                                                     "a list of timeout to be used in the vmo control");
            
            if (xtmp.size() != nj+1) 
                {
                    fprintf(stderr, "[VELOCITY] Timeout do not have the right number of entries. Using default Timeout=1000, i.e 1s\n");
                    for(i=1;i<nj+1; i++)
                        _velocityTimeout[i-1] = 1000;   //Default value
                }
            else
                {
                    for(i=1;i<xtmp.size(); i++) 
                        _velocityTimeout[i-1]=xtmp.get(i).asInt();
                }
        }
    else
    {
        fprintf(stderr, "A suitable value for [VELOCITY] Shifts was not found. Using default Shifts=4\n");
        for(i=1;i<nj+1; i++)
            _velocityShifts[i-1] = 4;   //Default value

        fprintf(stderr, "A suitable value for [VELOCITY] Timeout was not found. Using default Timeout=1000, i.e 1s.\n");
        for(i=1;i<nj+1; i++)
            _velocityTimeout[i-1] = 1000;   //Default value
    }

    xtmp=p.findGroup("CAN").findGroup("broadcast_pos");
    bool ret=setBroadCastMask(xtmp, CAN_BCAST_POSITION);
    xtmp=p.findGroup("CAN").findGroup("broadcast_pid");
    ret=ret&&setBroadCastMask(xtmp, CAN_BCAST_PID_VAL);
    xtmp=p.findGroup("CAN").findGroup("broadcast_fault");
    ret=ret&&setBroadCastMask(xtmp, CAN_BCAST_STATUS);
    xtmp=p.findGroup("CAN").findGroup("broadcast_current");
    ret=ret&&setBroadCastMask(xtmp, CAN_BCAST_CURRENT);
    xtmp=p.findGroup("CAN").findGroup("broadcast_overflow");
    ret=ret&&setBroadCastMask(xtmp, CAN_BCAST_OVERFLOW);
    xtmp=p.findGroup("CAN").findGroup("broadcast_canprint");
    ret=ret&&setBroadCastMask(xtmp, CAN_BCAST_PRINT);
    xtmp=p.findGroup("CAN").findGroup("broadcast_vel_acc");
    ret=ret&&setBroadCastMask(xtmp, CAN_BCAST_VELOCITY);

    if (!ret)
        fprintf(stderr, "Invalid configuration file, check broadcast_* parameters\n");
    
    //if (p.findGroup("CAN").find("broadcast_pos").asInt() == 1) broadcast_mask |= (1<<(CAN_BCAST_POSITION-1));
    //if (p.findGroup("CAN").find("broadcast_pid").asInt() == 1) broadcast_mask |= (1<<(CAN_BCAST_PID_VAL-1));
    //if (p.findGroup("CAN").find("broadcast_fault").asInt() == 1) broadcast_mask |= (1<<(CAN_BCAST_STATUS-1));
    //if (p.findGroup("CAN").find("broadcast_current").asInt() == 1) broadcast_mask |= (1<<(CAN_BCAST_CURRENT-1));
    //if (p.findGroup("CAN").find("broadcast_overflow").asInt() == 1) broadcast_mask |= (1<<(CAN_BCAST_OVERFLOW-1));
   // if (p.findGroup("CAN").find("broadcast_canprint").asInt() == 1) broadcast_mask |= (1<<(CAN_BCAST_PRINT-1));
    //if (p.findGroup("CAN").find("broadcast_vel_acc").asInt() == 1) broadcast_mask |= (1<<(CAN_BCAST_VELOCITY-1));

    // _broadcast_mask=broadcast_mask;



    return ret;
}

CanBusMotionControlParameters::CanBusMotionControlParameters()
{
    _networkN=0;
    _destinations=0;
    _axisMap=0;
    _angleToEncoder=0;
    _zeros=0;
    _pids=0;
    _limitsMax=0;
    _limitsMin=0;
    _currentLimits=0;
    _velocityShifts=0;
    _velocityTimeout=0;

    _my_address = 0;
    _polling_interval = 10;
    _timeout = 20;
    _njoints = 0;

    _txQueueSize = 2047;/** max len of the buffer for the esd driver */
    _rxQueueSize = 2047;
    _txTimeout = 20;/** 20ms timeout */
    _rxTimeout = 20;
    _broadcast_mask=0;
}

bool CanBusMotionControlParameters::alloc(int nj)
{
    _networkN = 0;
    _destinations = allocAndCheck<unsigned char> (CAN_MAX_CARDS);
    _axisMap = allocAndCheck<int>(nj);
    _angleToEncoder = allocAndCheck<double>(nj);
    _zeros = allocAndCheck<double>(nj);

    _pids=allocAndCheck<Pid>(nj);
    _limitsMax=allocAndCheck<double>(nj);
    _limitsMin=allocAndCheck<double>(nj);
    _currentLimits=allocAndCheck<double>(nj);
    _velocityShifts=allocAndCheck<int>(CAN_MAX_CARDS);
    _velocityTimeout=allocAndCheck<int>(CAN_MAX_CARDS);
    memset(_limitsMin, 0, sizeof(double)*nj);
    memset(_limitsMax, 0, sizeof(double)*nj);
    memset(_currentLimits, 0, sizeof(double)*nj);
    memset(_velocityShifts, 0, sizeof(int)*nj);
    memset(_velocityTimeout, 0, sizeof(int)*nj);

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
    checkAndDestroy<int>(_axisMap);
    checkAndDestroy<unsigned char>(_destinations);
    checkAndDestroy<int>(_velocityShifts);
    checkAndDestroy<int>(_velocityTimeout);


    checkAndDestroy<Pid>(_pids);
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
    _initialized=false;

    memset (_destinations, 0, sizeof(unsigned char) * CAN_MAX_CARDS);
    memset (_velShifts, 0, sizeof(int) * CAN_MAX_CARDS);

    _my_address = 0;
    _njoints = 0;

    _readMessages = 0;
    _writeMessages = 0;
    _bcastRecvBuffer = NULL;

    _error_status = true;
    _destInv=0;
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
        fprintf(stderr, "Warning could not instantiate can device\n");
        return false;
    }

    polyDriver.view(iCanBus);
    polyDriver.view(iBufferFactory);
    polyDriver.view(iCanErrors);

    if ((iCanBus==0) || (iBufferFactory==0))
    {
        fprintf(stderr, "CanBusResources::initialize() could not get ICanBus or ICanBufferFactory interface");
        return false;
    }

    ret=initialize(params);
    return ret;
}

bool CanBusResources::initialize (const CanBusMotionControlParameters& parms)
{
    printf("Calling CanBusResources::initialize\n");
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
    _txQueueSize=parms._txQueueSize;
    _rxQueueSize=parms._rxQueueSize;
    _filter = -1;

    for(int id=0;id<CAN_MAX_CARDS;id++)
        _destInv[id]=-1;

    //now fill inverted map
    for(int jj=0;jj<_njoints;jj+=2)
    {
        _destInv[_destinations[jj/2]]=jj;

     }

    _bcastRecvBuffer = allocAndCheck<BCastBufferElement> (_njoints);
    for (int j=0; j<_njoints ;j++)
        {
            _bcastRecvBuffer[j]._update_e=Time::now();
            _bcastRecvBuffer[j]._position.resetStats();
        }

    //previously initialized
    iCanBus->canSetBaudRate(_speed);
    // sets all message ID's for class 0 and 1.
    unsigned int i;
    for (i = 0; i < 0xff; i++)
        iCanBus->canIdAdd(i);

    for (i = 0x100; i < 0x1ff; i++)
        iCanBus->canIdAdd(i);

	// set all message ID's for class 2
    for (i = 0x200; i < 0x2ff; i++)
        iCanBus->canIdAdd(i);

    // set all message ID's for class 3
    for (i = 0x300; i < 0x3ff; i++)
        iCanBus->canIdAdd(i);

    _readBuffer=iBufferFactory->createBuffer(BUF_SIZE);
    _writeBuffer=iBufferFactory->createBuffer(BUF_SIZE);
    _replyBuffer=iBufferFactory->createBuffer(BUF_SIZE);

    requestsQueue = new RequestsQueue(_njoints, NUM_OF_MESSAGES);

    _initialized=true;

    DEBUG("CanBusResources::initialized correctly\n");
    return true;
}


bool CanBusResources::uninitialize ()
{
    // remember uninitialize might be called more than once
    // need to check for already destroyed pointers.
    if (!_initialized)
        return true;

    //fprintf(stderr, "CanBusResources::uninitialize\n");
    checkAndDestroy<BCastBufferElement> (_bcastRecvBuffer);

    if (_initialized)
    {
        iBufferFactory->destroyBuffer(_readBuffer);
        iBufferFactory->destroyBuffer(_writeBuffer);
        iBufferFactory->destroyBuffer(_replyBuffer);
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

    DEBUG("Sending buffer:\n");
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
    printf("CAN: write buffer\n");
    for (j = 0; j < _writeMessages; j++)
        printMessage (_writeBuffer[j]);

    printf("CAN: reply buffer\n");
    for (j = 0; j < _writeMessages; j++)
        printMessage (_replyBuffer[j]);

    printf("CAN: read buffer\n");
    for (j = 0; j < _readMessages; j++)
        printMessage (_readBuffer[j]);
    printf("CAN: -------------\n");

    return true;
}


CanBusMotionControl::CanBusMotionControl() : 
RateThread(10),
ImplementPositionControl<CanBusMotionControl, IPositionControl>(this),
ImplementVelocityControl<CanBusMotionControl, IVelocityControl>(this),
ImplementPidControl<CanBusMotionControl, IPidControl>(this),
ImplementEncoders<CanBusMotionControl, IEncoders>(this),
ImplementControlCalibration<CanBusMotionControl, IControlCalibration>(this),
ImplementControlCalibration2<CanBusMotionControl, IControlCalibration2>(this),
ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>(this),
ImplementControlLimits<CanBusMotionControl, IControlLimits>(this),
ImplementTorqueControl(this),
ImplementOpenLoopControl(this),
ImplementControlMode(this),
_mutex(1),
_done(0)
{
    system_resources = (void *) new CanBusResources;
    ACE_ASSERT (system_resources != NULL);
    _opened = false;
}


CanBusMotionControl::~CanBusMotionControl ()
{
    if (system_resources != NULL)
        delete (CanBusResources *)system_resources;
    system_resources = NULL;
}

bool CanBusMotionControl::open (Searchable &config)
{
    printf("Opening CanBusMotionControl Control\n");
    CanBusResources& res = RES (system_resources);
    CanBusMotionControlParameters p;
    bool ret=false;
    _mutex.wait();

    if(!p.fromConfig(config))
    {
        _mutex.post();
        return false;
    }

    String str=config.toString().c_str();
    Property prop;
    prop.fromString(str.c_str());
    canDevName=config.find("canbusdevice").asString();
    prop.unput("device");
    prop.unput("subdevice");
    prop.put("device", canDevName.c_str());
    prop.put("CanDeviceNum", p._networkN);
    prop.put("CanTxTimeout", p._txTimeout);
    prop.put("CanRxTimeout", p._rxTimeout);
    if (p._txQueueSize!=-1)
        prop.put("CanTxQueueSize", p._txQueueSize);
    if (p._rxQueueSize!=-1)
        prop.put("CanRxQueueSize", p._rxQueueSize);

    ret=res.initialize(prop);

    if (!ret)
    {
        _mutex.post();
        return false;
    }

    // used for printing debug messages.
    _filter = -1;
    _writerequested = false;
    _noreply = false;

    ImplementPositionControl<CanBusMotionControl, IPositionControl>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementVelocityControl<CanBusMotionControl, IVelocityControl>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementPidControl<CanBusMotionControl, IPidControl>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementEncoders<CanBusMotionControl, IEncoders>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementControlCalibration<CanBusMotionControl, IControlCalibration>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementControlCalibration2<CanBusMotionControl, IControlCalibration2>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementControlLimits<CanBusMotionControl, IControlLimits>::
        initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);

    ImplementControlMode::initialize(p._njoints, p._axisMap);
	ImplementTorqueControl::initialize(p._njoints, p._axisMap, p._angleToEncoder, p._zeros);
    ImplementOpenLoopControl::initialize(p._njoints, p._axisMap);
	
    // temporary variables used by the ddriver.
    _ref_positions = allocAndCheck<double>(p._njoints);
    _command_speeds = allocAndCheck<double>(p._njoints);
    _ref_speeds = allocAndCheck<double>(p._njoints);
    _ref_accs = allocAndCheck<double>(p._njoints);
    _ref_torques = allocAndCheck<double>(p._njoints);
    _mutex.post ();

    // default initialization for this device driver.
    setPids(p._pids);

    int i;
    for(i = 0; i < p._njoints; i++)
        setBCastMessages(i, p._broadcast_mask[i]);

    // set limits, on encoders and max current
    for(i = 0; i < p._njoints; i++) {
        setLimits(i, p._limitsMin[i], p._limitsMax[i]);
        setMaxCurrent(i, p._currentLimits[i]);
    }

    // set limits, on encoders and max current
    for(i = 0; i < p._njoints; i++) {
        setVelocityShift(i, p._velocityShifts[i]);
        setVelocityTimeout(i, p._velocityTimeout[i]);
    }

    // disable the controller, cards will start with the pid controller & pwm off
    for (i = 0; i < p._njoints; i++) {
        disablePid(i);
        disableAmp(i);
    }

    Bottle analogList=config.findGroup("analog").tail();
    //    if (analogList!=0)
        if (analogList.size()>0)
        {
            for(int k=0;k<analogList.size();k++)
            {
                std::string analogId=analogList.get(k).asString().c_str();;

                AnalogSensor *as=instantiateAnalog(config, analogId);
                if (as!=0)
                    analogSensors.push_back(as);
            }

        }

    threadPool = new ThreadPool2(res.iBufferFactory);

    RateThread::setRate(p._polling_interval);
    RateThread::start();

    _opened = true;

    DEBUG("CanBusMotionControl::open returned true\n");
    return true;
}

AnalogSensor *CanBusMotionControl::instantiateAnalog(yarp::os::Searchable& config, std::string deviceid)
{
    CanBusResources& res = RES (system_resources);
    AnalogSensor *analogSensor=0;

    //std::string groupName=std::string("ANALOG-");
    //groupName+=deviceid;
    //Bottle analogConfig=config.findGroup(groupName.c_str());
    Bottle analogConfig=config.findGroup(deviceid.c_str());
    if (analogConfig.size()>0)
    {
        fprintf(stderr, "--> Initializing analog device %s\n", deviceid.c_str());
        
        analogSensor=new AnalogSensor;
#ifdef USE_CANBACKDOOR
        backDoor.setUp(&res, &_mutex);
        backDoor.open("/portname"); //RANDAZ_TODO set portname based on analogConfig parameters
        //RANDAZ_TODO if needed set other parameters to backDoor
#endif
        analogSensor->setDeviceId(deviceid);

        char analogId=analogConfig.find("CanAddress").asInt();
        char analogFormat=analogConfig.find("Format").asInt();
        int analogChannels=analogConfig.find("Channels").asInt();
		int analogCalibration=analogConfig.find("UseCalibration").asInt();

        switch (analogFormat)
        {
            case 8:
                analogSensor->open(analogChannels, AnalogSensor::ANALOG_FORMAT_8, analogId, analogCalibration);
                break;
            case 16:
                analogSensor->open(analogChannels, AnalogSensor::ANALOG_FORMAT_16, analogId, analogCalibration);
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
        else if (analogChannels==6 && analogFormat==16)
        {
                //calibrated astrain board
                if (analogCalibration==1)
                {
                    //get the full scale values from the strain board
            		for (int ch=0; ch<6; ch++)
			        {
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
						        if (m.getId()==0x2D0 ||
							        m.getId()==0x2E0)
							        if (m.getLen()==4 &&
								        m.getData()[0]==0x18 &&
								        m.getData()[1]==ch)
								        {
									        analogSensor->getScaleFactor()[ch]=m.getData()[2]<<8 | m.getData()[3];
									        full_scale_read=true;
									        break;
								        }
					        }
					        yarp::os::Time::delay(0.001);
					        timeout++;
				        }
				        while(timeout<32 && full_scale_read==false);

				        if (full_scale_read==false) fprintf(stderr, "Trying to get fullscale data from sensor: no answer recieved or message lost (ch:%d)\n", ch);
			        }

                    // debug messages
		            #if 1
			             fprintf(stderr, "Sensor Fullscale: ");
			             fprintf(stderr, " %f ", analogSensor->getScaleFactor()[0]);
			             fprintf(stderr, " %f ", analogSensor->getScaleFactor()[1]);
			             fprintf(stderr, " %f ", analogSensor->getScaleFactor()[2]);
			             fprintf(stderr, " %f ", analogSensor->getScaleFactor()[3]);
			             fprintf(stderr, " %f ", analogSensor->getScaleFactor()[4]);
			             fprintf(stderr, " %f ", analogSensor->getScaleFactor()[5]);
			             fprintf(stderr, " \n ");
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
    }
    return analogSensor;
}

void CanBusMotionControl::finiAnalog(AnalogSensor *analogSensor)
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
                fprintf(stderr, "---> Len:%d %x %x %x\n", 
                    res._writeBuffer[0].getLen(),
                    res._writeBuffer[0].getId(),
                    res._writeBuffer[0].getData()[0],
                    res._writeBuffer[0].getData()[1]);
#endif
                res.writePacket();

                //shut down backdoor
#ifdef USE_CANBACKDOOR
                backDoor.close();
#endif
            }
}

bool CanBusMotionControl::close (void)
{
    CanBusResources& res = RES(system_resources);

    //fprintf(stderr, "CanBusMotionControl::close\n");

    if (_opened) {
        // disable the controller, pid controller & pwm off
        int i;
        for (i = 0; i < res._njoints; i++) {
            disablePid(i);
            disableAmp(i);
        }

        if (isRunning())
        {
            /// default initialization for this device driver.
            int i;
            for(i = 0; i < res.getJoints(); i++)
                setBCastMessages(i, 0x00);
        }

        RateThread::stop ();/// stops the thread first (joins too).
        ImplementPositionControl<CanBusMotionControl, IPositionControl>::uninitialize ();

        ImplementVelocityControl<CanBusMotionControl, IVelocityControl>::uninitialize();
        ImplementPidControl<CanBusMotionControl, IPidControl>::uninitialize();
        ImplementEncoders<CanBusMotionControl, IEncoders>::uninitialize();
        ImplementControlCalibration<CanBusMotionControl, IControlCalibration>::uninitialize();
        ImplementControlCalibration2<CanBusMotionControl, IControlCalibration2>::uninitialize();
        ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>::uninitialize();
        ImplementControlLimits<CanBusMotionControl, IControlLimits>::uninitialize();

        ImplementControlMode::uninitialize();
        ImplementTorqueControl::uninitialize();
        ImplementOpenLoopControl::uninitialize();

        
        //stop analog sensors
        std::list<AnalogSensor *>::iterator it=analogSensors.begin();
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
        delete threadPool;

    checkAndDestroy<double> (_ref_positions);
    checkAndDestroy<double> (_command_speeds);
    checkAndDestroy<double> (_ref_speeds);
    checkAndDestroy<double> (_ref_accs);
	checkAndDestroy<double> (_ref_torques);

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

    for (i = 0; i < r._readMessages; i++)
    {
        unsigned int len=0;
        unsigned int id=0;
        unsigned char *data=0;
        CanMessage& m = r._readBuffer[i];
        data=m.getData();
        id=m.getId();
        len=m.getLen();

        if ((id & 0x700) == 0x100) // class = 1.
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
                    fprintf(stderr, "%s [%d] Warning, got unexpected broadcast msg(s), last one from address %d, (original) id  0x%x, len %d\n", canDevName.c_str(), _networkN, addr, id, len);
                count++;
                j=-1; //error
            }
            else
            {
                j *= 2;

                /* less sign nibble specifies msg type */
                switch (id & 0x00f)
                {
                case CAN_BCAST_OVERFLOW:

                    printf ("ERROR: CAN PACKET LOSS, board %d buffer full\r\n", (((id & 0x0f0) >> 4)-1));

                    break;

                case CAN_BCAST_PRINT:

                    if (data[0] == CAN_BCAST_PRINT	||
                        data[0] == CAN_BCAST_PRINT + 128)
                    {	
                        int addr = (((id & 0x0f0) >> 4)-1);

                        int string_id = cstring[addr].add_string(&r._readBuffer[i]);
                        if (string_id != -1) 
                        {
                            cstring[addr].print(string_id);
                            cstring[addr].clear_string(string_id);
                        }
                    }
                    break;

                case CAN_BCAST_POSITION:
                    {
                        // r._bcastRecvBuffer[j]._position = *((int *)(data));
                        // r._bcastRecvBuffer[j]._update_p = before;
                        int tmp=*((int *)(data));
                        r._bcastRecvBuffer[j]._position.update(tmp, before);

                        j++;
                        if (j < r.getJoints())
                            {
                                tmp =*((int *)(data+4));
                                //r._bcastRecvBuffer[j]._position = *((int *)(data+4));
                                //r._bcastRecvBuffer[j]._update_p = before;
                                r._bcastRecvBuffer[j]._position.update(tmp, before);
                            }
                    }
                    break;

#if 0
                case CAN_BCAST_TORQUE:
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

                case CAN_BCAST_PID_VAL:
                    r._bcastRecvBuffer[j]._pid_value = *((short *)(data));
                    r._bcastRecvBuffer[j]._update_v = before;

                    j++;
                    if (j < r.getJoints())
                    {
                        r._bcastRecvBuffer[j]._pid_value = *((short *)(data+2));
                        r._bcastRecvBuffer[j]._update_v = before;
                    }
                    break;

                case CAN_BCAST_STATUS:
                    // fault signals.
                    r._bcastRecvBuffer[j]._axisStatus= *((short *)(data));
					r._bcastRecvBuffer[j]._canStatus= *((char *)(data+4));
					r._bcastRecvBuffer[j]._boardStatus= *((char *)(data+5));
                    r._bcastRecvBuffer[j]._update_e = before;
					r._bcastRecvBuffer[j]._controlmodeStatus=*((short *)(data+1));
                    r._bcastRecvBuffer[j]._address=addr;
                    r._bcastRecvBuffer[j]._canTxError+=*((char *) (data+6));
                    r._bcastRecvBuffer[j]._canRxError+=*((char *) (data+7));                                    
#if 0                    
                    if (_networkN==1)
                        {
                            for(int m=0;m<8;m++)
                                fprintf(stderr, "%.2x ", data[m]);
                            fprintf(stderr, "\n");
                        }
#endif

                    if (r._bcastRecvBuffer[j].isOverCurrent()) printf ("%s [%d] board %d OVERCURRENT AXIS 0\n", canDevName.c_str(), _networkN, addr);
                    //r._bcastRecvBuffer[j].ControlStatus(r._networkN, r._bcastRecvBuffer[j]._controlmodeStatus,addr); 
                    //if (r._bcastRecvBuffer[j].isFaultOk()) ACE_OS::printf ("Board %d OK\n", addr);
                    if (r._bcastRecvBuffer[j].isFaultUndervoltage()) printf ("%s [%d] board %d FAULT UNDERVOLTAGE AXIS 0\n", canDevName.c_str(), _networkN, addr);
                    if (r._bcastRecvBuffer[j].isFaultExternal()) printf ("%s [%d] board %d FAULT EXT AXIS 0\n", canDevName.c_str(), _networkN, addr);
                    if (r._bcastRecvBuffer[j].isFaultOverload()) printf ("%s [%d] board %d FAULT OVERLOAD AXIS 0\n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isHallSensorError()) printf ("%s [%d] board %d HALL SENSOR ERROR AXIS 0\n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isAbsEncoderError()) printf ("%s [%d] board %d ABS ENCODER ERROR AXIS 0\n", canDevName.c_str(), _networkN, addr);		
					if (r._bcastRecvBuffer[j].isCanTxOverflow()) printf ("%s [%d] board %d CAN TX OVERFLOW \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isCanBusOff()) printf ("%s [%d] board %d CAN BUS_OFF \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isCanTxError()) printf ("%s [%d] board %d CAN TX ERROR \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isCanRxError()) printf ("%s [%d] board %d CAN RX ERROR \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isCanRxWarning()) printf ("%s [%d] board %d CAN RX WARNING \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isCanRxOverrun()) printf ("%s [%d] board %d CAN RX OVERRUN \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isMainLoopOverflow()) printf ("%s [%d] board %d MAIN LOOP TIME EXCEDEED \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isOverTempCh1()) printf ("%s [%d] board %d OVER TEMPERATURE CH 1 \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isOverTempCh2()) printf ("%s [%d] board %d OVER TEMPERATURE CH 2 \n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isTempErrorCh1()) printf ("%s [%d] board %d ERROR TEMPERATURE CH 1\n", canDevName.c_str(), _networkN, addr);
					if (r._bcastRecvBuffer[j].isTempErrorCh2()) printf ("%s [%d] board %d ERROR TEMPERATURE CH 2\n", canDevName.c_str(), _networkN, addr);
                   
                    j++;

                    if (j < r.getJoints())
                    {
                        r._bcastRecvBuffer[j]._address=addr;
						r._bcastRecvBuffer[j]._axisStatus= *((short *)(data+2));
                        r._bcastRecvBuffer[j]._update_e = before;	
						r._bcastRecvBuffer[j]._controlmodeStatus=*((short *)(data+3));
                        // r._bcastRecvBuffer[j].ControlStatus(r._networkN, r._bcastRecvBuffer[j]._controlmodeStatus,addr); 
					    if (r._bcastRecvBuffer[j].isOverCurrent()) printf ("%s [%d] board %d OVERCURRENT AXIS 1\n", canDevName.c_str(), _networkN, addr);
                        if (r._bcastRecvBuffer[j].isFaultUndervoltage()) printf ("%s [%d] board %d FAULT UNDERVOLTAGE AXIS 1\n", canDevName.c_str(), _networkN, addr);
                        if (r._bcastRecvBuffer[j].isFaultExternal()) printf ("%s [%d] board %d FAULT EXT AXIS 1\n", canDevName.c_str(), _networkN, addr);
						if (r._bcastRecvBuffer[j].isFaultOverload()) printf ("%s [%d] board %d FAULT OVERLOAD AXIS 1\n", canDevName.c_str(), _networkN, addr);
					    if (r._bcastRecvBuffer[j].isHallSensorError()) printf ("%s [%d] board %d HALL SENSOR ERROR AXIS 1\n", canDevName.c_str(), _networkN, addr);
						if (r._bcastRecvBuffer[j].isAbsEncoderError()) printf ("%s [%d] board %d ABS ENCODER ERROR AXIS 1\n", canDevName.c_str(), _networkN, addr);
                    }	

                    break;

                case CAN_BCAST_CURRENT:
                    // also receives the control values.
                    r._bcastRecvBuffer[j]._current = *((short *)(data));

                    r._bcastRecvBuffer[j]._position_error = *((short *)(data+4));
                    r._bcastRecvBuffer[j]._update_c = before;
                    j++;
                    if (j < r.getJoints())
                    {
                        r._bcastRecvBuffer[j]._current = *((short *)(data+2));

                        r._bcastRecvBuffer[j]._position_error = *((short *)(data+6));
                        r._bcastRecvBuffer[j]._update_c = before;
                    }
                    break;

                case CAN_BCAST_VELOCITY:
                    // also receives the acceleration values.

                    r._bcastRecvBuffer[j]._speed = *((short *)(data));
                    r._bcastRecvBuffer[j]._accel = *((short *)(data+4));
                    r._bcastRecvBuffer[j]._update_s = before;
                    j++;
                    if (j < r.getJoints())
                    {
                        r._bcastRecvBuffer[j]._speed = *((short *)(data+2));
                        r._bcastRecvBuffer[j]._accel = *((short *)(data+6));
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
                                fprintf(stderr, "%s [%d] thread:%d msg:%d joint:%d timed out\n", 
                                        canDevName.c_str(),
                                        r._networkN,
                                        rq.threadId, rq.msg, rq.joint);
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
            fprintf(stderr, "%s [%d] thread ran %d times, req.dT:%d[ms], av.dT:%.2lf[ms] av.loopT :%.2lf[ms]\n", 
                    canDevName.c_str(),
                    r._networkN, 
                    it, 
                    r._polling_interval,
                    avPeriod,
                    avThTime);

            if (r.iCanErrors)
                {
                    CanErrors errors;
                    r.iCanErrors->canGetErrors(errors);
                    fprintf(stderr, " Can Errors --  Device Tx:%u Rx:%u TxOvf: %u RxOvf BusOff: %d -- Driver Fifo Tx:%u Rx:%u\n", 
                            errors.rxCanErrors,
                            errors.txCanErrors,
                            errors.rxCanFifoOvr,
                            errors.txCanFifoOvr,
                            errors.busoff,
                            errors.rxBufferOvr,
                            errors.txBufferOvr);
                }
            else
                {
                    fprintf(stderr, "     Device has no error interface\n");
                }
                

            int j=0;
            /// reports board errors
            char tmp[255];
            char message[255];
            sprintf(message, "%s [%d] printing boards errors:\n", canDevName.c_str(), r._networkN);
            
            bool errorF=false;
            for (j=0; j<r._njoints ;j+=2)
                {
                    int addr=r._destinations[j/2];
                    if ( (r._bcastRecvBuffer[j]._canTxError>0)||(r._bcastRecvBuffer[j]._canRxError>0))
                        {
                            errorF=true;
                            sprintf(tmp, "Id:%d T:%u R:%u ", addr, r._bcastRecvBuffer[j]._canTxError, r._bcastRecvBuffer[j]._canRxError);
                            sprintf(message, "%s%s", message, tmp);
                        }
                }
            if (!errorF)
                {
                    sprintf(tmp, "None");
                    sprintf(message, "%s%s", message, tmp);
                }

            fprintf(stderr, "%s\n", message);

            //Check statistics on boards
            for (j=0; j<r._njoints ;j++)
                {
                    double lastRecv = r._bcastRecvBuffer[j]._update_e;

                    if ( (currentRun-lastRecv)>BCAST_STATUS_TIMEOUT)
                        {
                            int ch=j%2;
                            int addr=r._destinations[j/2];
                            fprintf(stderr, "%s [%d] have not heard from board %d (channel %d) since %.2lf seconds\n", 
                                    canDevName.c_str(),
                                    r._networkN, 
                                    addr, ch, currentRun-lastRecv);
                        }
                }

            //Check position update frequency
            for(j=0; j<r._njoints; j++)
                {
                    double dT;
                    double max;
                    double min;
                    int it;
                    r._bcastRecvBuffer[j]._position.getStats(it, dT, min, max);
                    r._bcastRecvBuffer[j]._position.resetStats();

                    double POS_LATENCY_WARN_THR=averagePeriod*1.5;
                    if (max>POS_LATENCY_WARN_THR)
                        fprintf(stderr, "%s [%d] jnt %d, warning encoder latency above threshold (lat: %.2lf [ms], received %d msgs)\n",
                                canDevName.c_str(),
                                r._networkN,
                                j, 
                                max,
                                it);

                    if (it<1)
                        fprintf(stderr, "%s [%d] joint %d, warning not enough encoder messages (received %d msgs)\n",
                                canDevName.c_str(),
                                r._networkN,
                                j, 
                                it);

                }

            ///////////////////check analog
            std::list<AnalogSensor *>::iterator analogIt=analogSensors.begin();
            while(analogIt!=analogSensors.end())
            {
                AnalogSensor *pAnalog=(*analogIt);
                if (pAnalog)
                {
                    unsigned int sat;
                    unsigned int err;
                    unsigned int tout; 
                    pAnalog->getCounters(sat, err, tout);
                    if (sat+err+tout!=0)
                    {
                        fprintf(stderr, "%s [%d] analog %s saturated:%u errors: %u timeout:%u\n",
                                canDevName.c_str(),
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

    //DEBUG("CanBusMotionControl::thread running [%d]: wait\n", mycount);
    _mutex.wait ();
    //DEBUG("posted\n");

    if (r.read () != true)
        r.printMessage("%s [%d] CAN: read failed\n", canDevName.c_str(), r._networkN);

    // handle all broadcast messages.
    // (class 1, 8 bits of the ID used to define the message type and source address).

    handleBroadcasts();

    std::list<AnalogSensor *>::iterator analogIt=analogSensors.begin();
    while(analogIt!=analogSensors.end())
    {
        AnalogSensor *pAnalog=(*analogIt);
        if (pAnalog)
        {
            //fprintf(stderr, "Passing messages to analog device %s\n", pAnalog->getDeviceId().c_str());
            if (!pAnalog->handleAnalog(system_resources))
            {
                fprintf(stderr, "%s [%d] analog sensor received unexpected class 0x03 messages\n", canDevName.c_str(), r._networkN);
            }
        }
        else
            fprintf(stderr, "Warning: got null pointer this is unusual\n");
            
        analogIt++;
    }
 
    //
    // handle class 0 messages - polling messages.
    // (class 0, 8 bits of the ID used to represent the source and destination).
    // the first byte of the message is the message type and motor number (0 or 1).
    //
    if (r.requestsQueue->getPending()>0)
        {
            DEBUG("There are %d pending messages, read msgs: %d\n", 
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
                                    fprintf(stderr, "Received message but no threads waiting for it\n");
                                    continue;
                                }
                            ThreadTable2 *t=threadPool->getThreadTable(id);
                            if (t==0)
                                {
                                    fprintf(stderr, "Asked a bad thread id, this is probably a bug, check threadPool\n");
                                    continue;
                                }
                            DEBUG("Pushing reply\n");
                            //push reply to thread's list of replies
                            if (!t->push(m))
                                DEBUG("Warning, error while pushing a reply, this ir probably an error\n");
                        }
                }
        }
    else
        {
            //DEBUG("Thread loop: no pending messages\n");
        }

    //    counter ++;
    /*if (counter > r._timeout)
      {
      /// complains.
      r.printMessage("CAN: timeout - still %d messages unacknowledged\n", remainingMsgs);
      r._error_status = false;
      }*/

    _mutex.post ();

    double now = Time::now();
    averageThreadTime+=(now-before)*1000;
    previousRun=before; //save last run time
}


    // ControlMode
bool CanBusMotionControl::setPositionModeRaw(int j)
{
	if (!(j >= 0 && j <= (CAN_MAX_CARDS-1)*2))
        return false;

	DEBUG("Calling SET_CONTROL_MODE (position)\n");
	return _writeByte8(CAN_SET_CONTROL_MODE,j,1);
}

bool CanBusMotionControl::setOpenLoopModeRaw(int j)
{
	if (!(j >= 0 && j <= (CAN_MAX_CARDS-1)*2))
        return false;

	DEBUG("Calling SET_CONTROL_MODE (position)\n");
	//@@@ I'M SETTING HERE THE 0x50 CAN MESSAGE
	return _writeByte8(CAN_SET_CONTROL_MODE,j,MODE_OPENLOOP);
}

bool CanBusMotionControl::setVelocityModeRaw(int j)
{
    fprintf(stderr, "setVelocityModeRaw(): not yet implemented\n");
    return false;
}

bool CanBusMotionControl::setTorqueModeRaw(int j)
{
	if (!(j >= 0 && j <= (CAN_MAX_CARDS-1)*2))
        return false;

	DEBUG("Calling SET_CONTROL_MODE (torque)\n");
	return _writeByte8(CAN_SET_CONTROL_MODE,j,3);
}

bool CanBusMotionControl::getControlModeRaw(int j, int *v)
{
	if (!(j >= 0 && j <= (CAN_MAX_CARDS-1)*2))
    return false;

    short s;

    DEBUG("Calling GET_CONTROL_MODE\n");
    _readWord16 (CAN_GET_CONTROL_MODE, j, s); 
  
    switch (s)
    {
    case MODE_IDLE:
        *v=VOCAB_CM_IDLE;
        break;
    case MODE_POSITION:
        *v=VOCAB_CM_POSITION;
        break;				
    case MODE_VELOCITY:
        *v=VOCAB_CM_VELOCITY;
        break;
    case MODE_TORQUE:
        *v=VOCAB_CM_TORQUE;
        break;
    case MODE_OPENLOOP:
        *v=VOCAB_CM_OPENLOOP;
        break;
    default:
        *v=VOCAB_CM_UNKNOWN;
        break;
    }
	return true;
}

// return the number of controlled axes.
bool CanBusMotionControl::getAxes(int *ax)
{
    CanBusResources& r = RES(system_resources);
    *ax = r.getJoints();

    return true;
}

// LATER: can be optimized.
bool CanBusMotionControl::setPidRaw (int axis, const Pid &pid)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    _writeWord16 (CAN_SET_P_GAIN, axis, S_16(pid.kp));
    _writeWord16 (CAN_SET_D_GAIN, axis, S_16(pid.kd));
    _writeWord16 (CAN_SET_I_GAIN, axis, S_16(pid.ki));
    _writeWord16 (CAN_SET_ILIM_GAIN, axis, S_16(pid.max_int));
    _writeWord16 (CAN_SET_OFFSET, axis, S_16(pid.offset));
    _writeWord16 (CAN_SET_SCALE, axis, S_16(pid.scale));
    _writeWord16 (CAN_SET_TLIM, axis, S_16(pid.max_output));

    return true;
}

bool CanBusMotionControl::getPidRaw (int axis, Pid *out)
{
    //    ACE_ASSERT (axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    short s;

    DEBUG("Calling GET_P_GAIN\n");
    _readWord16 (CAN_GET_P_GAIN, axis, s); out->kp = double(s);
    DEBUG("Calling CAN_GET_D_GAIN\n");
    _readWord16 (CAN_GET_D_GAIN, axis, s); out->kd = double(s);
    DEBUG("Calling CAN_GET_I_GAIN\n");
    _readWord16 (CAN_GET_I_GAIN, axis, s); out->ki = double(s);
    DEBUG("Calling CAN_GET_ILIM_GAIN\n");
    _readWord16 (CAN_GET_ILIM_GAIN, axis, s); out->max_int = double(s);
    DEBUG("Calling CAN_GET_OFFSET\n");
    _readWord16 (CAN_GET_OFFSET, axis, s); out->offset= double(s);
    DEBUG("Calling CAN_GET_SCALE\n");
    _readWord16 (CAN_GET_SCALE, axis, s); out->scale = double(s);
    DEBUG("Calling CAN_GET_TLIM\n");
    _readWord16 (CAN_GET_TLIM, axis, s); out->max_output = double(s);
    DEBUG("Get PID done!\n");

    return true;
}

bool CanBusMotionControl::getPidsRaw (Pid *out)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        short s;
        _readWord16 (CAN_GET_P_GAIN, i, s); out[i].kp = double(s);
        _readWord16 (CAN_GET_D_GAIN, i, s); out[i].kd = double(s);
        _readWord16 (CAN_GET_I_GAIN, i, s); out[i].ki = double(s);
        _readWord16 (CAN_GET_ILIM_GAIN, i, s); out[i].max_int = double(s);
        _readWord16 (CAN_GET_OFFSET, i, s); out[i].offset= double(s);
        _readWord16 (CAN_GET_SCALE, i, s); out[i].scale = double(s);
        _readWord16 (CAN_GET_TLIM, i, s); out[i].max_output = double(s);
    }

    return true;
}

bool CanBusMotionControl::setTorquePidsRaw(const Pid *pids)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++) {
		setTorquePidRaw(i,pids[i]);
    }

    return true;
}
						  
bool CanBusMotionControl::setTorquePidRaw(int axis, const Pid &pid)
{
	 /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    DEBUG("setTorquePidRaw\n");

    if (!ENABLED(axis))
        return true;

    _mutex.wait();
		r.startPacket();
		r.addMessage (CAN_SET_TORQUE_PID, axis);
		*((short *)(r._writeBuffer[0].getData()+1)) = S_16(pid.kp);
		*((short *)(r._writeBuffer[0].getData()+3)) = S_16(pid.ki);
		*((short *)(r._writeBuffer[0].getData()+5)) = S_16(pid.kd);
		*((short *)(r._writeBuffer[0].getData()+7)) = S_16(pid.scale);
		r._writeBuffer[0].setLen(8);
		r.writePacket();
    _mutex.post();
    _mutex.wait();
		r.startPacket();
		r.addMessage (CAN_SET_TORQUE_PIDLIMITS, axis);
		*((short *)(r._writeBuffer[0].getData()+1)) = S_16(pid.offset);
		*((short *)(r._writeBuffer[0].getData()+3)) = S_16(pid.max_output);
		*((short *)(r._writeBuffer[0].getData()+5)) = S_16(pid.max_int);
		*((short *)(r._writeBuffer[0].getData()+7)) = S_16(0);
		r._writeBuffer[0].setLen(8);
		r.writePacket();
    _mutex.post();


    return true;
}

bool CanBusMotionControl::getTorquePidOutputRaw(int j, double *b)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    fprintf(stderr, "getTorquePidOutputRaw(): not yet implemented\n");
    return false;
}

bool CanBusMotionControl::getTorquePidOutputsRaw(double *b)
{
    fprintf(stderr, "getTorquePidOutputsRaw(): not yet implemented\n");
    return false;
}

bool CanBusMotionControl::getTorquePidRaw (int axis, Pid *out)
{
	DEBUG("Calling CAN_GET_TORQUE_PID \n");

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
        fprintf(stderr, "More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    r.addMessage (id, axis, CAN_GET_TORQUE_PID);
    r.writePacket();

	ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

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
	
	DEBUG("Calling CAN_GET_TORQUE_PIDLIMITS\n");
   
	r.startPacket();
    r.addMessage (id, axis, CAN_GET_TORQUE_PIDLIMITS);
    r.writePacket();

	// ThreadTable2 *t=threadPool->getThreadTable(id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();

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
	return true;
}

bool CanBusMotionControl::getTorquePidsRaw (Pid *out)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
		getTorquePidRaw(i, &(out[i]));
    }

    return true;
}

bool CanBusMotionControl::setPidsRaw(const Pid *pids)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++) {
        _writeWord16 (CAN_SET_P_GAIN, i, S_16(pids[i].kp));
        _writeWord16 (CAN_SET_D_GAIN, i, S_16(pids[i].kd));
        _writeWord16 (CAN_SET_I_GAIN, i, S_16(pids[i].ki));
        _writeWord16 (CAN_SET_ILIM_GAIN, i, S_16(pids[i].max_int));
        _writeWord16 (CAN_SET_OFFSET, i, S_16(pids[i].offset));
        _writeWord16 (CAN_SET_SCALE, i, S_16(pids[i].scale));
        _writeWord16 (CAN_SET_TLIM, i, S_16(pids[i].max_output));
    }

    return true;
}

/// cmd is a SingleAxis poitner with 1 double arg
bool CanBusMotionControl::setReferenceRaw (int j, double ref)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeDWord (CAN_SET_COMMAND_POSITION, axis, S_32(ref));
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::setReferencesRaw (const double *refs)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        if (_writeDWord (CAN_SET_COMMAND_POSITION, i, S_32(refs[i])) != true)
            return false;
    }

    return true;
}

/// cmd is a SingleAxis poitner with 1 double arg
bool CanBusMotionControl::setTorqueRaw (int j, double ref)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeDWord (CAN_SET_DESIRED_TORQUE, axis, S_32(ref));
}

/// cmd is a SingleAxis pointer with 1 double arg
bool CanBusMotionControl::getTorqueRaw (int j, double *t)
{
    CanBusResources& r = RES(system_resources);
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    _mutex.wait();
    *t=double(r._bcastRecvBuffer[axis]._torque);
    _mutex.post();
    return false;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::setTorquesRaw (const double *refs)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        if (_writeDWord (CAN_SET_DESIRED_TORQUE, i, S_32(refs[i])) != true)
            return false;
    }

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getTorquesRaw (double *t)
{
    CanBusResources& r = RES(system_resources);

    int i;
    _mutex.wait();
    
    double stamp=0;
    for (i = 0; i < r.getJoints(); i++)
    {
        t[i] = double(r._bcastRecvBuffer[i]._torque);

        if (stamp<r._bcastRecvBuffer[i]._update_t)
            stamp=r._bcastRecvBuffer[i]._update_t;
    }
    stampEncoders.update(stamp);

    _mutex.post();
    return true;
}

bool CanBusMotionControl::disableTorquePidRaw(int j)
{
	return NOT_YET_IMPLEMENTED("isableTorquePidRaw");
}

bool CanBusMotionControl::enableTorquePidRaw(int j)
{
	return NOT_YET_IMPLEMENTED("enableTorquePidRaw");
}

bool CanBusMotionControl::setErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimit");
}

bool CanBusMotionControl::setErrorLimitsRaw(const double *limit)
{
    return NOT_YET_IMPLEMENTED("setErrorLimits");
}

bool CanBusMotionControl::setTorqueErrorLimitRaw(int j, double limit)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimit");
}

bool CanBusMotionControl::setTorqueErrorLimitsRaw(const double *limit)
{
    return NOT_YET_IMPLEMENTED("setTorqueErrorLimits");
}

bool CanBusMotionControl::getTorqueErrorRaw(int axis, double *err)
{
	return NOT_YET_IMPLEMENTED("getErrorRaw");
}

bool CanBusMotionControl::getTorqueErrorsRaw(double *errs)
{
	return NOT_YET_IMPLEMENTED("getErrorsRaw");
}

bool CanBusMotionControl::getErrorRaw(int axis, double *err)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;
    _mutex.wait();
    *(err) = double(r._bcastRecvBuffer[axis]._position_error);
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getErrorsRaw(double *errs)
{
    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++)
    {
        errs[i] = double(r._bcastRecvBuffer[i]._position_error);
    }
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getOutputRaw(int axis, double *out)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))
        return false;
    _mutex.wait();
    *(out) = double(r._bcastRecvBuffer[axis]._pid_value);
    _mutex.post();
    return true;
}

bool CanBusMotionControl::getOutputsRaw(double *outs)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++)
    {
        outs[i] = double(r._bcastRecvBuffer[i]._pid_value);
    }

    _mutex.post();
    return true;
}

bool CanBusMotionControl::getReferenceRaw(int j, double *ref)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    int value = 0;
    if (_readDWord (CAN_GET_DESIRED_POSITION, axis, value) == true)
        *ref = double (value);
    else
        return false;

    return true;
}

bool CanBusMotionControl::getReferencesRaw(double *ref)
{
    return _readDWordArray(CAN_GET_DESIRED_POSITION, ref);
}

bool CanBusMotionControl::getErrorLimitRaw(int j, double *err)
{
    return NOT_YET_IMPLEMENTED("getErrorLimit");
}

bool CanBusMotionControl::getErrorLimitsRaw(double *errs)
{
    return NOT_YET_IMPLEMENTED("getErrorLimits");
}

bool CanBusMotionControl::getTorqueErrorLimitRaw(int j, double *err)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimit");
}

bool CanBusMotionControl::getTorqueErrorLimitsRaw(double *errs)
{
    return NOT_YET_IMPLEMENTED("getTorqueErrorLimits");
}

bool CanBusMotionControl::resetPidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetPid");
}

bool CanBusMotionControl::resetTorquePidRaw(int j)
{
    return NOT_YET_IMPLEMENTED("resetTorquePid");
}

bool CanBusMotionControl::enablePidRaw(int axis)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeNone (CAN_CONTROLLER_RUN, axis);

}

bool CanBusMotionControl::setOffsetRaw(int axis, double v)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (CAN_SET_OFFSET, axis, S_16(v));

}

bool CanBusMotionControl::setOutputsRaw(const double *v)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++) {
		setOutputRaw(i,v[i]);
    }

    return true;
}

bool CanBusMotionControl::setOutputRaw(int axis, double v)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (CAN_SET_OFFSET, axis, S_16(v));

}

bool CanBusMotionControl::setTorqueOffsetRaw(int axis, double v)
{
    return NOT_YET_IMPLEMENTED("setTorqueOffsetRaw");
}

bool CanBusMotionControl::disablePidRaw(int axis)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeNone (CAN_CONTROLLER_IDLE, axis);
}

bool CanBusMotionControl::setPositionMode()
{
    return NOT_YET_IMPLEMENTED("setPositionMode");
}

/*bool CanBusMotionControl::setOpenLoopModeRaw()
{
    return NOT_YET_IMPLEMENTED("setOpenLoopModeRaw");
}*/

bool CanBusMotionControl::setOpenLoopMode(int axis)
{
    return NOT_YET_IMPLEMENTED("setOpenLoopMode");
}

bool CanBusMotionControl::setTorqueModeRaw()
{
    return NOT_YET_IMPLEMENTED("setTorqueModeRaw");
}

bool CanBusMotionControl::setVelocityMode()
{
    return NOT_YET_IMPLEMENTED("setVelocityModeRaw");
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
        _ref_positions[axis] = ref;
        return true;
    }

    _mutex.wait();

    r.startPacket();
    r.addMessage (CAN_POSITION_MOVE, axis);

    _ref_positions[axis] = ref;
    *((int*)(r._writeBuffer[0].getData()+1)) = S_32(_ref_positions[axis]);/// pos
    *((short*)(r._writeBuffer[0].getData()+5)) = S_16(_ref_speeds[axis]);/// speed
    r._writeBuffer[0].setLen(7);

    r.writePacket();

    _mutex.post();

    return true;
}

bool CanBusMotionControl::positionMoveRaw(const double *refs)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    r.startPacket();

    for (i = 0; i < r.getJoints (); i++)
    {
        if (ENABLED(i))
        {
            r.addMessage (CAN_POSITION_MOVE, i);
            const int j = r._writeMessages - 1;
            _ref_positions[i] = refs[i];
            *((int*)(r._writeBuffer[j].getData()+1)) = S_32(_ref_positions[i]);/// pos
            *((short*)(r._writeBuffer[j].getData()+5)) = S_16(_ref_speeds[i]);/// speed
            r._writeBuffer[j].setLen(7);
        }
        else
        {
            _ref_positions[i] = refs[i];
        }
    }

    r.writePacket();

    _mutex.post();

    return true;   
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

    if (!_readWord16 (CAN_MOTION_DONE, axis, value))
    {
        *ret=false;
        return false;
    }

    *ret= (value!=0);

    return true;
}

/// cmd is a pointer to a bool
bool CanBusMotionControl::checkMotionDoneRaw (bool *ret)
{
    CanBusResources& r = RES(system_resources);
    int i;
    short value;

    _mutex.wait();

    int id;
    if (!threadPool->getId(id))
    {
        fprintf(stderr, "More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    r.startPacket();
    for (i = 0; i < r.getJoints(); i++)
    {
        if (ENABLED(i))
        {
            r.addMessage (id, i, CAN_MOTION_DONE);
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
                    *ret=false;
                    return true;
                }
            }
            j++;
        }
    }

    t->clear();

    *ret=true;
    return true;
}

bool CanBusMotionControl::calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3)
{
    return _writeByteWords16 (CAN_CALIBRATE_ENCODER, axis, type, S_16(p1), S_16(p2), S_16(p3));
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
    memcpy(_ref_speeds, spds, sizeof(double) * r.getJoints());
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

    return _writeWord16 (CAN_SET_DESIRED_ACCELER, axis, s);
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

        if (!_writeWord16 (CAN_SET_DESIRED_ACCELER, i, S_16(_ref_accs[i])))
            return false;
    }

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefSpeedsRaw (double *spds)
{
    CanBusResources& r = RES(system_resources);

    memcpy(spds, _ref_speeds, sizeof(double) * r.getJoints());
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
        if (_readWord16 (CAN_GET_DESIRED_ACCELER, i, value) == true) {
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

    if (_readWord16 (CAN_GET_DESIRED_ACCELER, axis, value) == true)
    {
        _ref_accs[axis] = double (value);
        *accs = double(value) * 1000.0 * 1000.0;
    }
    else
        return false;

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefTorquesRaw (double *t)
{
    CanBusResources& r = RES(system_resources);
    int i;
    short value = 0;

    for(i = 0; i < r.getJoints(); i++)
    {
        if (_readWord16 (CAN_GET_DESIRED_TORQUE, i, value) == true) {
            _ref_torques[i] = t[i] = double (value);
        }
        else
            return false;
    }

    return true;
}

/// cmd is an array of double (LATER: to be optimized).
bool CanBusMotionControl::getRefTorqueRaw (int axis, double *t)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    short value = 0;

    if (_readWord16 (CAN_GET_DESIRED_TORQUE, axis, value))
    {
        _ref_torques[axis] = double (value);
		*t = double (value);
    }
    else
        return false;

    return true;
}

bool CanBusMotionControl::stopRaw(int j)
{
    return velocityMoveRaw(j, 0);
}

bool CanBusMotionControl::stopRaw()
{
    CanBusResources& r = RES(system_resources);
    const int n=r.getJoints();

    double *tmp = new double [n];
    memset(tmp, 0, sizeof(double)*n);
    bool ret=velocityMoveRaw(tmp);
    
    delete [] tmp;
    return ret;
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool CanBusMotionControl::velocityMoveRaw (int axis, double sp)
{
    /// prepare can message.
    CanBusResources& r = RES(system_resources);

    _mutex.wait();
    r.startPacket();

    if (ENABLED (axis))
    {
        r.addMessage (CAN_VELOCITY_MOVE, axis);
        const int j = r._writeMessages - 1;
        _command_speeds[axis] = sp / 1000.0;

        *((short*)(r._writeBuffer[j].getData()+1)) = S_16(r._velShifts[axis]*_command_speeds[axis]);/// speed

        if (r._velShifts[axis]*_ref_accs[axis]>1)
            *((short*)(r._writeBuffer[j].getData()+3)) = S_16(r._velShifts[axis]*_ref_accs[axis]);/// accel
        else
            *((short*)(r._writeBuffer[j].getData()+3)) = S_16(1);

        r._writeBuffer[j].setLen(5);
    }
    else
    {
        _command_speeds[axis] = sp / 1000.0;
    }

    r.writePacket();

    _mutex.post();

    return true;
}

/// cmd is an array of double of length njoints specifying speed 
/// for each axis
bool CanBusMotionControl::velocityMoveRaw (const double *sp)
{
    /// prepare can message.
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    r.startPacket();

    for (i = 0; i < r.getJoints(); i++)
    {
        if (ENABLED (i))
        {
            r.addMessage (CAN_VELOCITY_MOVE, i);
            const int j = r._writeMessages - 1;
            _command_speeds[i] = sp[i] / 1000.0;

            *((short*)(r._writeBuffer[j].getData()+1)) = S_16(r._velShifts[i]*_command_speeds[i]);/// speed

            if (r._velShifts[i]*_ref_accs[i]>1)
                *((short*)(r._writeBuffer[j].getData()+3)) = S_16(r._velShifts[i]*_ref_accs[i]);/// accel
            else
                *((short*)(r._writeBuffer[j].getData()+3)) = S_16(1);

            r._writeBuffer[j].setLen(5);
        }
        else
        {
            _command_speeds[i] = sp[i] / 1000.0;
        }
    }

    r.writePacket();
    _mutex.post();
    return true;
}

bool CanBusMotionControl::setEncoderRaw(int j, double val)
{
    const int axis = j;
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeDWord (CAN_SET_ENCODER_POSITION, axis, S_32(val));
}

bool CanBusMotionControl::setEncodersRaw(const double *vals)
{
    CanBusResources& r = RES(system_resources);

    int i;
    for (i = 0; i < r.getJoints(); i++)
    {
        if (_writeDWord (CAN_SET_ENCODER_POSITION, i, S_32(vals[i])) != true)
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
        v[i] = double(r._bcastRecvBuffer[i]._position._value);

        if (stamp<r._bcastRecvBuffer[i]._position._stamp)
            stamp=r._bcastRecvBuffer[i]._position._stamp;
    }

    stampEncoders.update(stamp);

    _mutex.post();
    return true;
}

bool CanBusMotionControl::getEncoderRaw(int axis, double *v)
{
    CanBusResources& r = RES(system_resources);
    if (!(axis >= 0 && axis <= r.getJoints()))return false;

    _mutex.wait();
    *v = double(r._bcastRecvBuffer[axis]._position._value);
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getEncoderSpeedsRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++) {
        v[i] = (double(r._bcastRecvBuffer[i]._speed)*1000.0)/WINDOW_SIZE_VEL;
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
    *v = (double(r._bcastRecvBuffer[j]._speed)*1000.0)/WINDOW_SIZE_VEL;
    _mutex.post();

    return true;
}

bool CanBusMotionControl::getEncoderAccelerationsRaw(double *v)
{
    CanBusResources& r = RES(system_resources);
    int i;
    _mutex.wait();
    for (i = 0; i < r.getJoints(); i++) {
        v[i] = (double(r._bcastRecvBuffer[i]._accel)*1000000.0)/(WINDOW_SIZE_VEL*WINDOW_SIZE_ACC);
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
    *v = (double(r._bcastRecvBuffer[j]._accel)*1000000.0)/(WINDOW_SIZE_VEL*WINDOW_SIZE_ACC);
    _mutex.post();

    return true;
}

bool CanBusMotionControl::disableAmpRaw(int axis)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeNone (CAN_DISABLE_PWM_PAD, axis);
}

bool CanBusMotionControl::enableAmpRaw(int axis)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeNone (CAN_ENABLE_PWM_PAD, axis);
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

    return _writeDWord (CAN_SET_CURRENT_LIMIT, axis, S_32(v));
}

bool CanBusMotionControl::setVelocityShift(int axis, double shift)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (CAN_SET_VEL_SHIFT, axis, S_16(shift));
}

bool CanBusMotionControl::setVelocityTimeout(int axis, double timeout)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    return _writeWord16 (CAN_SET_VEL_TIMEOUT, axis, S_16(timeout));
}


bool CanBusMotionControl::calibrateRaw(int axis, double p)
{
    return _writeWord16 (CAN_CALIBRATE_ENCODER, axis, S_16(p));
}

bool CanBusMotionControl::doneRaw(int axis)
{
    short value = 0;
    DEBUG("Calling doneRaw for joint %d\n", axis);
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (!_readWord16 (CAN_GET_CONTROL_MODE, axis, value))
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
    printf("Calling obsolete funzion setPrintFunction\n");
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
	//	Line changed with no idea about what it should do
	//    st[i] = short(r._bcastRecvBuffer[i]._fault);  
		st[i] = short(r._bcastRecvBuffer[i]._axisStatus);  
   
	}
    _mutex.post();

    return true;
}

bool CanBusMotionControl::setLimitsRaw(int axis, double min, double max)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    bool ret=true;

    ret = ret && _writeDWord (CAN_SET_MIN_POSITION, axis, S_32(min));
    ret = ret && _writeDWord (CAN_SET_MAX_POSITION, axis, S_32(max));

    return ret;
}

bool CanBusMotionControl::getLimitsRaw(int axis, double *min, double *max)
{
    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;
    int iMin=0;
    int iMax=0;
    bool ret=true;

    ret = ret && _readDWord (CAN_GET_MIN_POSITION, axis, iMin);
    ret = ret && _readDWord (CAN_GET_MAX_POSITION, axis, iMax);

    *min=iMin;
    *max=iMax;

    return ret;
}

bool CanBusMotionControl::loadBootMemory()
{
    CanBusResources& r = RES(system_resources);

    bool ret=true;
    for(int j=0; j<r.getJoints(); j++)
    {
        ret=_writeNone(CAN_READ_FLASH_MEM, j);
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
        ret=_writeNone(CAN_WRITE_FLASH_MEM, j);
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
    return _writeDWord (CAN_SET_BCAST_POLICY, axis, S_32(v));
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

    DEBUG("Write None msg:%d axis:%d\n", msg, axis);
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

    DEBUG("Writing Word16 msg:%d axis:%d\n", msg, axis);
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

    DEBUG("Writing byte msg:%d axis:%d\n", msg, axis);

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

    DEBUG("Writing DWord msg:%d axis:%d\n", msg, axis);

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
bool CanBusMotionControl::_writeWord16Ex (int msg, int axis, short s1, short s2, bool checkAxisEven=true)
{
    /// prepare Can message.
    CanBusResources& r = RES(system_resources);

    if (!(axis >= 0 && axis <= (CAN_MAX_CARDS-1)*2))
        return false;

    if (checkAxisEven)
        ACE_ASSERT ((axis % 2) == 0);/// axis is even.

    DEBUG("Write Word16Ex  msg:%d axis:%d\n", msg, axis);

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
        fprintf(stderr, "More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
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
        fprintf(stderr, "More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
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
        DEBUG("readDWord: message timed out\n");
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
        fprintf(stderr, "More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
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
        DEBUG("readDWordArray: at least one message timed out\n");
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
        fprintf(stderr, "More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
        _mutex.post();
        return false;
    }

    DEBUG("readWord16: called from thread %d, axis %d msg %d\n", id, axis, msg);
    r.startPacket();
    r.addMessage (id, axis, msg);
    r.writePacket(); //write immediatly

    ThreadTable2 *t=threadPool->getThreadTable(id);
    DEBUG("readWord16: going to wait for packet %d\n", id);
    t->setPending(r._writeMessages);
    _mutex.post();
    t->synch();
    DEBUG("readWord16: ok, wait done %d\n",id);

    if (!r.getErrorStatus() || (t->timedOut()))
    {
        DEBUG("readWord16: message timed out\n");
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

/// reads an array.
bool CanBusMotionControl::_readWord16Array (int msg, double *out)
{
    CanBusResources& r = RES(system_resources);
    int i;

    _mutex.wait();
    int id;
    if (!threadPool->getId(id))
    {
        fprintf(stderr, "More than %d threads, cannot allow more\n", CANCONTROL_MAX_THREADS);
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
        DEBUG("readWord16Array: at least one message timed out\n");
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

    
    std::cout<<"CanBusMotionControl::createDevice() looking for device " << deviceType;
    std::cout<< " with id " << deviceId<<std::endl;

    if (deviceType=="analog")
    {
        std::list<AnalogSensor *>::iterator it=analogSensors.begin();
        while(it!=analogSensors.end())
        {
            std::cout<<"CanBusMotionControl::createDevice() inspecting "<< (*it)->getDeviceId() << std::endl; 

            if ((*it)->getDeviceId()==deviceId)
            {
                std::cout<<"CanBusMotionControl::createDevice() found valid device"<<std::endl; 
                return (*it);
            }
            it++;
        }
    }
    else
    {
        std::cout<<"CanBusMotionControl::createDevice() only works for analog kind of devices\n";
    }

    return 0;
}
