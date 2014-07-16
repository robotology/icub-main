// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup canbusmotioncontrol canbusmotioncontrol
 *
 * This device contains code which handles communication to 
 * the motor control boards on a CAN bus. It 
 * converts requests from function calls into CAN bus messages for
 * the motor control boards. A thread monitors the bus for incoming
 * messages and dispatches replies to calling threads.
 *
 * Comunication with the CAN bus is done through the standard
 * YARP ICanBus interface.
 *
 * Copyright (C) 2010 RobotCub Consortium.
 *
 * Author: Lorenzo Natale
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

//
// $Id: CanBusMotionControl.h,v 1.16 2009/07/29 13:12:29 nat Exp $
//
//

#ifndef __CanBusMotionControlh__
#define __CanBusMotionControlh__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardHelper.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/dev/PreciselyTimed.h>
#include <iCub/DebugInterfaces.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>

#include <iCub/FactoryInterface.h>
#include <iCub/LoggerInterfaces.h>
#include <messages.h>

namespace yarp{
    namespace dev{
        class CanBusMotionControl;
        class CanBusMotionControlParameters;
    }
}

class ThreadPool2;
class RequestsQueue;
struct SpeedEstimationParameters
{
    double jnt_Vel_estimator_shift;
    double jnt_Acc_estimator_shift;
    double mot_Vel_estimator_shift;
    double mot_Acc_estimator_shift;

    SpeedEstimationParameters()
    {
        jnt_Vel_estimator_shift=0;
        jnt_Acc_estimator_shift=0;
        mot_Vel_estimator_shift=0;
        mot_Acc_estimator_shift=0;
    }
};

struct ImpedanceLimits
{
    double min_stiff;
    double max_stiff;
    double min_damp;
    double max_damp;
    double param_a;
    double param_b;
    double param_c;

    public:
    ImpedanceLimits()
    {
        min_stiff=0; max_stiff=0;
        min_damp=0;  max_damp=0;
        param_a=0; param_a=0; param_c=0;
    }

    double get_min_stiff() {return min_stiff;}
    double get_max_stiff() {return max_stiff;}
    double get_min_damp()  {return min_damp;}
    double get_max_damp()  {return max_damp;}
};
/**
* \file CanBusMotionControl.h 
* class for interfacing with a generic can device driver.
*/

/**
* \include UserDoc_dev_motorcontrol.dox
*/

/**
* @ingroup dev_impl_motor
*
* The PlxCan motion controller device driver.
* Contains a thread that takes care of polling the can bus for incoming messages.
*/

/**
* The open parameter class containing the initialization values.
*/
class yarp::dev::CanBusMotionControlParameters
{
private:
    CanBusMotionControlParameters (const CanBusMotionControlParameters&);
    void operator= (const CanBusMotionControlParameters&);



public:
    /**
    * Constructor (please make sure you use the constructor to allocate
    * memory).
    * @param nj is the number of controlled joints/axes.
    */
    CanBusMotionControlParameters ();
    
    /**
    * Destructor, with memory deallocation.
    */
    ~CanBusMotionControlParameters ();

    struct DebugParameters
    {
        double data[8];
        bool   enabled;
        DebugParameters() {for (int i=0; i<8; i++) data[i]=0; enabled=false;}
    };

    struct ImpedanceParameters
    {
        double stiffness;
        double damping;
        bool   enabled;
        ImpedanceLimits limits;
        ImpedanceParameters() {stiffness=0; damping=0; enabled=false;}
    };

    bool parsePosPidsGroup_OldFormat(yarp::os::Bottle& pidsGroup, int nj, Pid myPid[]);
    bool parseTrqPidsGroup_OldFormat(yarp::os::Bottle& pidsGroup, int nj, Pid myPid[]);
    bool parsePidsGroup_NewFormat(yarp::os::Bottle& pidsGroup, Pid myPid[]);
    bool parseImpedanceGroup_NewFormat(yarp::os::Bottle& pidsGroup, ImpedanceParameters vals[]);
    bool parseDebugGroup_NewFormat(yarp::os::Bottle& pidsGroup, DebugParameters vals[]);

    bool setBroadCastMask(yarp::os::Bottle &list, int MASK);

    bool fromConfig(yarp::os::Searchable &config);
    bool alloc(int nj);

    int _txQueueSize;
    int _rxQueueSize;
    int _txTimeout;
    int _rxTimeout;
    int *_broadcast_mask;

    int _networkN;                              /** network number */
    int _njoints;                               /** number of joints/axes/controlled motors */
    unsigned char *_destinations;               /** destination addresses */
    unsigned char _my_address;                  /** my address */
    int _polling_interval;                      /** thread polling interval [ms] */
    int _timeout;                               /** number of cycles before timing out */

    int *_axisMap;                              /** axis remapping lookup-table */
    double *_angleToEncoder;                    /** angle to encoder conversion factors */
    double *_rotToEncoder;                      /** angle to rotor conversion factors */
    double *_zeros;                             /** encoder zeros */
    Pid *_pids;                                 /** initial gains */
    Pid *_tpids;                                /** initial torque gains */
    bool _pwmIsLimited;                         /** set to true if pwm is limited */
    bool _tpidsEnabled;                         /** abilitation for torque gains */
    SpeedEstimationParameters *_estim_params;   /** parameters for speed/acceleration estimation */
    DebugParameters *_debug_params;             /** debug parameters */
    ImpedanceParameters *_impedance_params;     /** impedance parameters */
    ImpedanceLimits     *_impedance_limits;     /** impedancel imits */
    double *_bemfGain;                          /** bemf compensation gain */
    double *_limitsMin;                         /** joint limits, max*/
    double *_limitsMax;                         /** joint limits, min*/
    double *_currentLimits;                     /** current limits */
    int *_velocityShifts;                       /** velocity shifts */
    int *_velocityTimeout;                      /** velocity shifts */
    double *_maxStep;                           /** max size of a positionDirect step */
    double *_optical_factor;                    /** reduction ratio of the optical encoder on motor axis */ 
    int *_torqueSensorId;                       /** Id of associated Joint Torque Sensor */
    int *_torqueSensorChan;                     /** Channel of associated Joint Torque Sensor */
    double *_maxTorque;                         /** Max torque of a joint */
    double *_newtonsToSensor;                   /** Newtons to force sensor units conversion factors */
};

class TBR_AnalogData
{
private:
    double *_data;
    int _size;
    int _bufferSize;
public:
    TBR_AnalogData(int ch, int buffsize): _data(0), _size(ch), _bufferSize(buffsize)
    {
        _data=new double[_bufferSize];
        for(int k=0;k<_bufferSize;k++)
            _data[k]=0;
    }
    ~TBR_AnalogData()
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

#include <yarp/os/Semaphore.h>
typedef int AnalogDataFormat;

class TBR_CanBackDoor;

class TBR_AnalogSensor: public yarp::dev::IAnalogSensor,
                    public yarp::dev::DeviceDriver
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
    // debug messages
    unsigned int counterSat;
    unsigned int counterError;
    unsigned int counterTimeout;
    int rate;

    ////////////////////
    TBR_AnalogData *data;
    short status;
    double timeStamp;
    double* scaleFactor;
    yarp::os::Semaphore mutex;
    AnalogDataFormat dataFormat;
    yarp::os::Bottle initMsg;
    yarp::os::Bottle speedMsg;
    yarp::os::Bottle closeMsg;
    std::string deviceIdentifier;
    short boardId;
    short useCalibration;
    bool  isVirtualSensor; //RANDAZ

    bool decode8(const unsigned char *msg, int id, double *data);
    bool decode16(const unsigned char *msg, int id, double *data);

public:
    TBR_CanBackDoor* backDoor; //RANDAZ

    TBR_AnalogSensor();
    ~TBR_AnalogSensor();
    bool handleAnalog(void *);

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

    bool open(int channels, AnalogDataFormat f, short bId, short useCalib, bool isVirtualSensor);

    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value)
    {
        return calibrateSensor();
    }

    virtual int calibrateChannel(int ch)
    {
        return calibrateChannel(ch, 0);
    }
    /////////////////////////////////
};

class speedEstimationHelper
{
    int  jointsNum;
    SpeedEstimationParameters *estim_params;
    
    public:
    speedEstimationHelper (int njoints, SpeedEstimationParameters* estim_parameters );
    inline int getNumberOfJoints ()
    { 
        return jointsNum;
    }
    inline SpeedEstimationParameters getEstimationParameters (int jnt)
    {
        if (jnt>=0 && jnt<jointsNum) return estim_params[jnt];
        SpeedEstimationParameters empty_params;
        return empty_params;
    }
    inline ~speedEstimationHelper()
    {
        delete [] estim_params;
    }
};

#define BOARD_TYPE_4DC    0x03
#define BOARD_TYPE_BLL    0x04 // Note: Also BLL2DC firmware is identified BLL. This is intentional.

struct can_protocol_info
{
    int major;
    int minor;
};

struct firmware_info
{
    int joint;
    int network_number;
    std::string network_name;
    int board_can_id;
    int board_type;
    int fw_major;
    int fw_version;
    int fw_build;
    can_protocol_info can_protocol;
    int ack;

    inline void print_info()
    {
        fprintf(stderr,"%s [%d] joint: %d can_address: %2d ",network_name.c_str(),network_number,joint,board_can_id);
        
        if (board_type==0)
        {
            fprintf(stderr,"Unable to detect firmware version. Old firmware running?");
        }
        else
        {
           if (board_type==BOARD_TYPE_4DC) fprintf(stderr,"board type: 3 (4DC) ");
           if (board_type==BOARD_TYPE_BLL) fprintf(stderr,"board type: 4 (BLL) ");
           fprintf(stderr,"version:%2x.%2x build:%3d CAN_protocol:%d.%d", fw_major, fw_version, fw_build,can_protocol.major,can_protocol.minor);
        }
        fprintf(stderr,"\n");
    }

};

class firmwareVersionHelper
{
    int  jointsNum;
    
    public:
    firmware_info* infos;
    can_protocol_info icub_protocol;

    firmwareVersionHelper(int joints, firmware_info* f_infos, can_protocol_info& protocol)
    {
        icub_protocol = protocol;
        jointsNum=joints;
        infos = new firmware_info [jointsNum];
        for (int i=0; i<jointsNum; i++)
        {
            infos[i] = f_infos[i];
        }
    }
    inline int getNumberOfJoints ()
    { 
        return jointsNum;
    }
    bool checkFirmwareVersions()
    {
        bool printed = false;
        for (int j=0; j<jointsNum; j++)
        {
            if (infos[j].board_type==0)
            {
                printMessageSevereError();
                return false;
            }

            if (infos[j].ack==0)
            {
                printMessageSevereError();
                return false;
            }

            if (infos[j].board_type==BOARD_TYPE_BLL)
            {
                // Note: Also BLL2DC firmware is identified BLL. This is intentional.
                if (infos[j].fw_build<LAST_BLL_BUILD) 
                {
                    if (!printed) printMessagePleaseUpgradeFirmware();
                    printed = true;
                }
                if (infos[j].fw_build>LAST_BLL_BUILD)
                {
                    if (!printed) printMessagePleaseUpgradeiCub();
                    printed = true;
                }
            }

            if (infos[j].board_type==BOARD_TYPE_4DC)
            {
                if (infos[j].fw_build<LAST_MC4_BUILD) 
                {
                    if (!printed) printMessagePleaseUpgradeFirmware();
                    printed = true;
                }
                if (infos[j].fw_build>LAST_MC4_BUILD) 
                {
                    if (!printed) printMessagePleaseUpgradeiCub();
                    printed = true;
                }
            }
        }
        return true;
    }
    inline void printFirmwareVersions()
    {
        fprintf(stderr,"\n");
        fprintf(stderr,"**********************************\n");
        fprintf(stderr,"iCubInterface CAN protocol: %d.%d\n",icub_protocol.major,icub_protocol.minor);
        fprintf(stderr,"Firmware report:\n");
        for (int j=0; j<jointsNum; j++)
        {
            infos[j].print_info();
        }
        fprintf(stderr,"**********************************\n");
        fprintf(stderr,"\n");
    }
    inline void printMessagePleaseUpgradeFirmware()
    {
        fprintf(stderr,"\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"\n");
        fprintf(stderr,"  iCubInterface detected that your control boards are not running the latest\n");
        fprintf(stderr,"  available firmware version, although it is still compatible with it.\n");
        fprintf(stderr,"  Upgrading your iCub firmware to build %d is highly recommended.\n", LAST_BLL_BUILD);
        fprintf(stderr,"  For further information please visit: http://wiki.icub.org/wiki/Firmware\n");
        fprintf(stderr,"\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"\n");
    }
    inline void printMessagePleaseUpgradeiCub()
    {
        fprintf(stderr,"\n");
        fprintf(stderr,"#################################################################################################\n");
        fprintf(stderr,"#################################################################################################\n");
        fprintf(stderr,"\n");
        fprintf(stderr,"  iCubInterface detected that your control boards are running a firmware version\n");
        fprintf(stderr,"  which is newer than the recommended version (build %d), although it is still compatible with it.\n", LAST_BLL_BUILD);
        fprintf(stderr,"  It may also be that you are running an experimental firmware version. \n");
        fprintf(stderr,"  An update of Yarp/iCub SW is recommended. Proceed only if you are aware of what you are doing.\n");
        fprintf(stderr,"\n");
        fprintf(stderr,"#################################################################################################\n");
        fprintf(stderr,"#################################################################################################\n");
        fprintf(stderr,"\n");
    }
    inline void printMessageSevereError()
    {
        fprintf(stderr,"\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"\n");
        fprintf(stderr,"  It has been detected that your control boards are not using the same\n");
        fprintf(stderr,"  CAN protocol used by iCubinterface. iCubInterface cannot continue.\n");
        fprintf(stderr,"  Please update your system (iCubInterface and/or your control board firmware.\n");
        fprintf(stderr,"  For further information please visit: http://wiki.icub.org/wiki/Firmware\n");
        fprintf(stderr,"\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"###################################################################################\n");
        fprintf(stderr,"\n");
    }
    inline ~firmwareVersionHelper()
    {
        delete [] infos;
        infos = 0;
    }
};

class axisImpedanceHelper
{
    int  jointsNum;
    ImpedanceLimits* impLimits;
    
    public:
    axisImpedanceHelper(int njoints, ImpedanceLimits* imped_limits );

    inline ~axisImpedanceHelper()
    {
        delete [] impLimits;
        impLimits=0;
           }
    
    inline ImpedanceLimits* getImpedanceLimits () {return impLimits;}
};

class axisPositionDirectHelper
{
    int  jointsNum;
    double* maxHwStep;
    double* maxUserStep;
    ControlBoardHelper* helper;
    
    public:
    axisPositionDirectHelper(int njoints, const int *aMap, const double *angToEncs, double* _maxStep);

    inline ~axisPositionDirectHelper()
    {
        delete [] maxHwStep;
        maxHwStep=0;
        delete [] maxUserStep;
        maxUserStep=0;
        delete helper;
        helper = 0;
    }
    
    inline double posA2E (double ang, int j) {return helper->posA2E(ang, j);}
    inline double posE2A (double ang, int j) {return helper->posE2A(ang, j);}
    inline double getMaxHwStep (int j) {return maxHwStep[j];}
    inline double getMaxUserStep (int j) {return maxUserStep[j];}
    inline double getSaturatedValue (int j, double curr_value, double ref_value); 
};

class axisTorqueHelper
{
    int  jointsNum;
    int* torqueSensorId;                        /** Id of associated Joint Torque Sensor */
    int* torqueSensorChan;                      /** Channel of associated Joint Torque Sensor */
    double* maximumTorque;
    double* newtonsToSensor;

    public:
    axisTorqueHelper(int njoints, int* id, int* chan, double* maxTrq, double* newtons2sens );
    inline int getTorqueSensorId (int jnt)
    {
        if (jnt>=0 && jnt<jointsNum) return torqueSensorId[jnt];
        return 0;
    }
    inline int getTorqueSensorChan (int jnt)
    {
        if (jnt>=0 && jnt<jointsNum) return torqueSensorChan[jnt];
        return 0;
    }
    inline double getMaximumTorque (int jnt)
    {
        if (jnt>=0 && jnt<jointsNum) return maximumTorque[jnt];
        return 0;
    }
    inline double getNewtonsToSensor (int jnt)
    {
        if (jnt>=0 && jnt<jointsNum) return newtonsToSensor[jnt];
        return 0;
    }
    inline int getNumberOfJoints ()
    {
        return jointsNum;
    }
    inline ~axisTorqueHelper()
    {
        delete [] torqueSensorId;
        delete [] torqueSensorChan;
        delete [] maximumTorque;
        delete [] newtonsToSensor;
        torqueSensorId=0;
        torqueSensorChan=0;
        maximumTorque=0;
        newtonsToSensor=0;
    }
};

class yarp::dev::CanBusMotionControl:public DeviceDriver,
            public os::RateThread, 
            public IPidControlRaw, 
            public IPositionControl2Raw,
            public IPositionDirectRaw,
            public IControlCalibration2Raw,
            public IVelocityControl2Raw,
            public IAmplifierControlRaw,
            public IControlCalibrationRaw,
            public IDebugInterfaceRaw,
            public IControlLimits2Raw,
            public ITorqueControlRaw,
            public IImpedanceControlRaw,
            public IOpenLoopControlRaw,
            public IControlMode2Raw,
            public IPreciselyTimed,
            public ImplementPositionControl2,
            public ImplementPositionDirect,
            public ImplementVelocityControl2,
            public ImplementPidControl<CanBusMotionControl, IPidControl>,
            public IEncodersTimedRaw,
            public ImplementEncodersTimed,
            public ImplementControlCalibration<CanBusMotionControl, IControlCalibration>,    
            public ImplementControlCalibration2<CanBusMotionControl, IControlCalibration2>,
            public ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>,
            public ImplementControlLimits2,
            public ImplementTorqueControl,
            public ImplementImpedanceControl,
            public ImplementOpenLoopControl,
            public ImplementControlMode2,
            public ImplementDebugInterface,
            public IInteractionModeRaw,
            public ImplementInteractionMode,
            public IFactoryInterface,
            public IClientLogger
{
private:
    CanBusMotionControl(const CanBusMotionControl&);
    void operator=(const CanBusMotionControl&);

    void handleBroadcasts();
 
    double previousRun;
    double averagePeriod;
    double averageThreadTime;
    double currentRun;
    int myCount;
    double lastReportTime;
    os::Stamp stampEncoders;

    char _buff[256];

    std::list<TBR_AnalogSensor *> analogSensors;

    yarp::os::ConstString canDevName;

    IServerLogger *mServerLogger;

    bool readFullScaleAnalog(int analog_can_address, int channel, double* fullScale);
    TBR_AnalogSensor *instantiateAnalog(yarp::os::Searchable& config, std::string id);
    void finiAnalog(TBR_AnalogSensor *s);

public:
    /**
    * Default constructor. Construction is done in two stages, first build the
    * object and then open the device driver.
    */
    CanBusMotionControl();

    /**
    * Destructor.
    */
    virtual ~CanBusMotionControl();

    /**
    * Open the device driver and start communication with the hardware.
    * @param config is a Searchable object containing the list of parameters.
    * @return true on success/failure.
    */
    virtual bool open(yarp::os::Searchable& config);

    /**
    * Open the device driver.
    * @param par is the parameter structure 
    * @return true/false on success/failure.
    */ 
    bool open(const CanBusMotionControlParameters &par);


    /**
    * Closes the device driver.
    * @return true on success.
    */
    virtual bool close(void);

    ////////////// IFactoryInterface
    yarp::dev::DeviceDriver *createDevice(yarp::os::Searchable& config);

    ////////////// IClientLogger
    void setServerLogger(const IServerLogger *server)
    { 
        mServerLogger=(IServerLogger*)server; 
    }

    #ifdef _USE_INTERFACEGUI
    
    void logNetworkData(const char *devName,int network,int index,const yarp::os::Value& data)
    {
        if (mServerLogger)
        {
            sprintf(_buff,"%s %d,network,%d",devName,network,index);
            mServerLogger->log(std::string(_buff),data);
        }
    }
    
    void logJointData(const char *devName,int network,int joint,int index,const yarp::os::Value& data)
    {
        if (mServerLogger)
        {
            sprintf(_buff,"%s %d,BLL,%d,%d",devName,network,joint,index);
            mServerLogger->log(std::string(_buff),data);
        }
    }

    void logAnalogData(const char *devName,int network,int board,int index,const yarp::os::Value& data)
    {
        if (mServerLogger)
        {
            sprintf(_buff,"%s %d,analog,%d,%d",devName,network,board,index);
            mServerLogger->log(std::string(_buff),data);
        }
    }
    #else
    #define logJointData(a,b,c,d,e)
    #define logNetworkData(a,b,c,d)
    #define logBoardData(a,b,c,d,e)
    #endif

    ///////////// PID INTERFACE
    //
    virtual bool setPidRaw(int j, const Pid &pid);
    virtual bool setPidsRaw(const Pid *pids);
    virtual bool setReferenceRaw(int j, double ref);
    virtual bool setReferencesRaw(const double *refs);
    virtual bool setErrorLimitRaw(int j, double limit);
    virtual bool setErrorLimitsRaw(const double *limits);
    virtual bool getErrorRaw(int j, double *err);
    virtual bool getErrorsRaw(double *errs);
//    virtual bool getOutputRaw(int j, double *out);    // also in the openloop interface
//    virtual bool getOutputsRaw(double *outs);         // also in the openloop interface
    virtual bool getPidRaw(int j, Pid *pid);
    virtual bool getPidsRaw(Pid *pids);
    virtual bool getReferenceRaw(int j, double *ref);
    virtual bool getReferencesRaw(double *refs);
    virtual bool getErrorLimitRaw(int j, double *limit);
    virtual bool getErrorLimitsRaw(double *limits);
    virtual bool resetPidRaw(int j);
    virtual bool disablePidRaw(int j);
    virtual bool enablePidRaw(int j);
    virtual bool setOffsetRaw(int j, double v);
    //
    /////////////////////////////// END PID INTERFACE

    //
    /// POSITION CONTROL INTERFACE RAW
    virtual bool getAxes(int *ax);
    virtual bool setPositionModeRaw();
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs);
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas);
    virtual bool checkMotionDoneRaw(bool *flag);
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds);
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs);
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds);
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs);
    virtual bool stopRaw(int j);
    virtual bool stopRaw();
    //
    /////////////////////////////// END Position Control INTERFACE

    //
    /// TORQUE CONTROL INTERFACE RAW
//    virtual bool getAxes(int *ax);
    virtual bool getRefTorqueRaw(int j, double *ref_trq);
    virtual bool getRefTorquesRaw(double *ref_trqs);
    virtual bool setRefTorqueRaw(int j, double ref_trq);
    virtual bool setRefTorquesRaw(const double *ref_trqs);
    virtual bool getTorqueRaw(int j, double *trq);
    virtual bool getTorquesRaw(double *trqs);
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max);
    virtual bool getBemfParamRaw(int j, double *trq);
    virtual bool setBemfParamRaw(int j, double trq);

    virtual bool setTorquePidRaw(int j, const Pid &pid);
    virtual bool setTorquePidsRaw( const Pid *pid);
    virtual bool getTorquePidRaw(int j, Pid *pid);
    virtual bool getTorquePidsRaw(Pid *pids);
    virtual bool resetTorquePidRaw(int j);
    virtual bool disableTorquePidRaw(int j);
    virtual bool enableTorquePidRaw(int j);
    virtual bool setTorqueOffsetRaw(int j, double v);

    virtual bool getTorqueErrorLimitRaw(int j, double *limit);
    virtual bool getTorqueErrorLimitsRaw(double *limits);
    virtual bool setTorqueErrorLimitRaw(int j, double limit);
    virtual bool setTorqueErrorLimitsRaw(const double *limits);
    virtual bool getTorqueErrorRaw(int j, double *err);
    virtual bool getTorqueErrorsRaw(double *errs);
    virtual bool setTorqueModeRaw();
    virtual bool getTorquePidOutputRaw(int j, double *v);
    virtual bool getTorquePidOutputsRaw(double *v);
 
    //
    /////////////////////////////// END Torque Control INTERFACE

    //
    /// IMPEDANCE CONTROL INTERFACE RAW
    virtual bool getImpedanceRaw(int j, double *stiff, double *damp);  
    virtual bool setImpedanceRaw(int j, double  stiff, double  damp);   
    virtual bool getImpedanceOffsetRaw(int j, double *offs);  
    virtual bool setImpedanceOffsetRaw(int j, double  offs);   
    virtual bool getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);

    //
    /////////////////////////////// END Impedance Control INTERFACE

    // ControlMode
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
    virtual bool setImpedancePositionModeRaw(int j);
    virtual bool setImpedanceVelocityModeRaw(int j);
    virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *v);
    virtual bool getControlModesRaw(int* v);

    // ControlMode 2
    virtual bool getControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModeRaw(const int j, const int mode);
    virtual bool setControlModesRaw(const int n_joint, const int *joints, int *modes);
    virtual bool setControlModesRaw(int *modes);


    ///////////// OpenLoop control interface raw
    ///
    virtual bool setOpenLoopModeRaw();
    virtual bool setRefOutputRaw(int axis, double v);
    virtual bool setRefOutputsRaw(const double *v);
    virtual bool getRefOutputRaw(int j, double *out);
    virtual bool getRefOutputsRaw(double *outs);
    virtual bool getOutputRaw(int j, double *out);
    virtual bool getOutputsRaw(double *outs);
    /////////////////////////////// END OpenLoop Control INTERFACE

    ///////////// Velocity control interface raw
    ///
    virtual bool setVelocityModeRaw();
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);
    //
    /////////////////////////////// END Velocity Control INTERFACE

    //Shift factors for velocity control
    bool setVelocityShiftRaw(int j, double val);
    //Timeout factors for velocity control
    bool setVelocityTimeoutRaw(int j, double val);
    //Shift factors for speed / acceleration estimation
    bool setSpeedEstimatorShiftRaw(int j, double jnt_speed, double jnt_acc, double mot_speed, double mot_acc);
    //Factor used to make correspond optical encoder units to joint angle degrees
    bool setOpticalRatioRaw(int axis, double ratio);


    //////////////////////// BEGIN EncoderInterface
    //
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw();
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double *vals);
    virtual bool getEncoderRaw(int j, double *v);
    virtual bool getEncodersRaw(double *encs);
    virtual bool getEncoderSpeedRaw(int j, double *sp);
    virtual bool getEncoderSpeedsRaw(double *spds);
    virtual bool getEncoderAccelerationRaw(int j, double *spds);
    virtual bool getEncoderAccelerationsRaw(double *accs);
    //
    ///////////////////////// END Encoder Interface

    virtual bool getEncodersTimedRaw(double *v, double *t);
    virtual bool getEncoderTimedRaw(int j, double *v, double *t);

    ////// Amplifier interface
    //
    virtual bool enableAmpRaw(int j);
    virtual bool disableAmpRaw(int j);
    virtual bool getCurrentsRaw(double *vals);
    virtual bool getCurrentRaw(int j, double *val);
    virtual bool setMaxCurrentRaw(int j, double val);
    virtual bool getAmpStatusRaw(int *st);
    virtual bool getAmpStatusRaw(int j, int *st);

    //
    /////////////// END AMPLIFIER INTERFACE

    ////// calibration
    virtual bool calibrateRaw(int j, double p);
    virtual bool doneRaw(int j);
    virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);

    /// IControlDebug Interface
    virtual bool setPrintFunction(int (*f) (const char *fmt, ...));
    virtual bool loadBootMemory();
    virtual bool saveBootMemory();

    // IDebug Interface
    virtual bool setParameterRaw(int j, unsigned int type, double value);
    virtual bool getParameterRaw(int j, unsigned int type, double* value);
    virtual bool setDebugParameterRaw(int j, unsigned int index, double value);
    virtual bool getDebugParameterRaw(int j, unsigned int index, double* value);
    virtual bool setDebugReferencePositionRaw(int j, double value);
    virtual bool getDebugReferencePositionRaw(int j, double *value);
    virtual bool getRotorPositionRaw(int j, double* value);
    virtual bool getRotorPositionsRaw(double *value);
    virtual bool getRotorSpeedRaw(int j, double* value);
    virtual bool getRotorSpeedsRaw(double *value);
    virtual bool getRotorAccelerationRaw(int j, double* value);
    virtual bool getRotorAccelerationsRaw(double *value);
    virtual bool getJointPositionRaw(int j, double* value);
    virtual bool getJointPositionsRaw(double *value);

    /////// Limits
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);
    // Limits 2
    bool setVelLimitsRaw(int axis, double min, double max);
    bool getVelLimitsRaw(int axis, double *min, double *max);

    /////// IPreciselyTimed interface
    virtual yarp::os::Stamp getLastInputStamp();


    // Position Control2 Interface
    virtual bool positionMoveRaw(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMoveRaw(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDoneRaw(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeedsRaw(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerationsRaw(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeedsRaw(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerationsRaw(const int n_joint, const int *joints, double *accs);
    virtual bool stopRaw(const int n_joint, const int *joints);

    // IVelocityControl2
    bool velocityMoveRaw(const int n_joint, const int *joints, const double *spds);
    bool setVelPidRaw(int j, const Pid &pid);
    bool setVelPidsRaw(const Pid *pids);
    bool getVelPidRaw(int j, Pid *pid);
    bool getVelPidsRaw(Pid *pids);

    // Firmware version
    bool getFirmwareVersionRaw (int axis, can_protocol_info const& icub_interface_protocol, firmware_info *info);
    // Torque measurement selection
    bool setTorqueSource (int axis, char board_id, char board_chan );

    // PositionDirect Interface
    bool setPositionDirectModeRaw();
    bool setPositionRaw(int j, double ref);
    bool setPositionsRaw(const int n_joint, const int *joints, double *refs);
    bool setPositionsRaw(const double *refs);

    // InteractionMode interface
    bool getInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum* mode);
    bool getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);
    bool setInteractionModeRaw(int axis, yarp::dev::InteractionModeEnum mode);
    bool setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes);
    bool setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes);

protected:
    bool setBCastMessages (int axis, unsigned int v);

protected:
    void *system_resources;
    yarp::os::Semaphore _mutex;
    yarp::os::Semaphore _done;
    ICanBus *canController;

    bool _writerequested;
    bool _noreply;
    bool _opened;
    ThreadPool2 *threadPool;

    /**
    * filter for recurrent messages.
    */
    int _filter;

    /**
    * helper function to check whether the enabled flag is on or off.
    * @param axis is the axis to check for.
    * @return true if the axis is enabled and processing of the message
    * can in fact continue.
    */
    inline bool ENABLED (int axis);

    virtual void run(void);
    virtual bool threadInit();
    virtual void threadRelease();

    // helper functions
    bool _writeWord16 (int msg, int axis, short s);
    bool _writeWord16Ex (int msg, int axis, short s1, short s2, bool check=true);
    bool _readWord16 (int msg, int axis, short& value);
    bool _readWord16Ex (int msg, int axis, short& value1, short& value2);
    bool _readWord16Array (int msg, double *out);
    bool _readDWord (int msg, int axis, int& value);
    bool _readDWordArray (int msg, double *out);
    bool _writeDWord (int msg, int axis, int value);
    bool _writeNone  (int msg, int axis);
    bool _writeByte8 (int msg, int axis, int value);
    bool _writeByteWords16(int msg, int axis, unsigned char value, short s1, short s2, short s3);
    axisTorqueHelper      *_axisTorqueHelper;
    axisImpedanceHelper   *_axisImpedanceHelper;
    firmwareVersionHelper *_firmwareVersionHelper;
    speedEstimationHelper *_speedEstimationHelper;
    axisPositionDirectHelper  *_axisPositionDirectHelper;

    inline unsigned char from_modevocab_to_modeint (int modevocab);
    inline int from_modeint_to_modevocab (unsigned char modeint);
    inline unsigned char from_interactionvocab_to_interactionint (int interactionvocab);
    inline int from_interactionint_to_interactionvocab (unsigned char interactionint);

    // internal stuff.
    double *_ref_speeds;        // used for position control.
    double *_command_speeds;    // used for velocity control.
    double *_ref_accs;            // for velocity control, in position min jerk eq is used.
    double *_ref_torques;        // for torque control.
    double *_ref_positions;        // for position control.

    enum { MAX_SHORT = 32767, MIN_SHORT = -32768, MAX_INT = 0x7fffffff, MIN_INT = 0x80000000 };
    enum { CAN_SKIP_ADDR = 0x80 };

    inline short S_16(double x) const 
    {
        if (x <= double(-(MAX_SHORT))-1)
            return MIN_SHORT;
        else
            if (x >= double(MAX_SHORT))
                return MAX_SHORT;
        else
            if  (x>0)
                return short(x + .5);
            else
                return short(x - .5);
    }

    inline int S_32(double x) const
    {
        if (x <= double(-(MAX_INT))-1.0)
            return MIN_INT;
        else
            if (x >= double(MAX_INT))
                return MAX_INT;
        else
            if  (x>0)
                return int(x + .5);
            else
                return int(x - .5);
    }
};

#endif


