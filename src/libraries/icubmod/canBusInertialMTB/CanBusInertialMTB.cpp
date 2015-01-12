// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo and Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusInertialMTB.h>

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <string.h>

#include <canProtocolLib/iCubCanProto_skinMessages.h>

const int    CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE=2047;
const int    CANBUS_INERTIAL_MTB_CHANNELS=6;
const double CANBUS_INERTIAL_MTB_TIMEOUT=0.1; //100ms


// Protocol for inertial sensor messages sent by the MTB
const unsigned int    CAN_MSG_CLASS_ACC_GYRO  = 0x500;
const unsigned int    MSG_TYPE_GYRO           = 0x000;
const unsigned int    MSG_TYPE_ACC           = 0x001;

const unsigned short    CANBUS_INERTIAL_MTB_DEFAULT_SENSOR_PERIOD=5;

const char   CANBUS_INERTIAL_MTB_INTERNAL_ACC_BIT = 0x02;
const char   CANBUS_INERTIAL_MTB_EXTERNAL_ACC_BIT = 0x08;
const char   CANBUS_INERTIAL_MTB_EXTERNAL_GYRO_BIT = 0x04;

using namespace std;


bool checkRequiredParamIsString(yarp::os::Searchable& config,
                                const yarp::os::ConstString& paramName)
{
    bool correct = config.check(paramName);
    if( correct )
    {
        correct = config.find(paramName).isString();
    }
    
    if( !correct )
    {
        yError("CanBusInertialMTB: problem loading parameter %s as string",paramName.c_str());
    }

    return correct;
}

bool checkRequiredParamIsInt(yarp::os::Searchable& config,
                                const yarp::os::ConstString& paramName)
{
    bool correct = config.check(paramName);
    if( correct )
    {
        correct = config.find(paramName).isInt();
    }

    if( !correct )
    {
        yError("CanBusInertialMTB: problem loading parameter %s as int",paramName.c_str());
    }
    
    return correct;
}

bool CanBusInertialMTB::validateConf(yarp::os::Searchable& config)
{
    bool correct=true;

    correct = correct && checkRequiredParamIsString(config,"canbusDevice");
    correct = correct && checkRequiredParamIsInt(config,"canDeviceNum");
    correct = correct && checkRequiredParamIsInt(config,"canAddress");
    correct = correct && checkRequiredParamIsString(config,"physDevice");
    correct = correct && checkRequiredParamIsString(config,"sensorType");

    return correct;
}

bool CanBusInertialMTB::open(yarp::os::Searchable& config)
{
    bool correct = this->validateConf(config);

    if (!correct)
    {
        yError("CanBusInertialMTB: Insufficient parameters to CanBusInertialMTB\n");
        return false;
    }

    //Parse sensor type
    std::string sensor_type = config.find("sensorType").asString();
    if( sensor_type == "acc" )
    {
        this->enabledSensors = CANBUS_INERTIAL_MTB_INTERNAL_ACC_BIT;
        this->enabledGyro    = false;
    }
    else if( sensor_type == "extAccAndGyro" )
    {
        this->enabledSensors = CANBUS_INERTIAL_MTB_EXTERNAL_GYRO_BIT |
                               CANBUS_INERTIAL_MTB_EXTERNAL_ACC_BIT;
        this->enabledGyro    = true;
    }
    else
    {
        yError("CanBusInertialMTB: unknown sensorType %s",sensor_type.c_str());
        return false;
    }

    if (config.check("period")==true)
    {
        int period=10;
        period=config.find("period").asInt();
        setRate(period);
    }

    if (config.check("sensorPeriod"))
    {
        int int_sensorPeriod = config.find("sensorPeriod").asInt();
        if( int_sensorPeriod < 1 || int_sensorPeriod > 255 )
        {
            yError("CanBusInertialMTB: sensorPeriod is lower than 1 or bigger then 255\n");
            return false;
        }
        this->sensorPeriod = int_sensorPeriod;
    }

    Property prop;

    prop.put("device", config.find("canbusDevice").asString().c_str());
    prop.put("physDevice", config.find("physDevice").asString().c_str());
    prop.put("canTxTimeout", 500);
    prop.put("canRxTimeout", 500);
    prop.put("canDeviceNum", config.find("canDeviceNum").asInt());
    prop.put("canMyAddress", 0);
    prop.put("canTxQueueSize", CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);
    prop.put("canRxQueueSize", CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);
    pCanBus=0;
    pCanBufferFactory=0;

    //open the can driver
    driver.open(prop);
    if (!driver.isValid())
    {
        yError("Error opening CanBusInertialMTB check parameters\n");
        return false;
    }
    driver.view(pCanBus);
    if (!pCanBus)
    {
        yError("Error opening can device not available\n");
        return false;
    }
    driver.view(pCanBufferFactory);
    outBuffer=pCanBufferFactory->createBuffer(CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);

    //select the communication speed
    pCanBus->canSetBaudRate(0); //default 1MB/s


    //set the internal configuration
    this->boardId           = config.find("canAddress").asInt();

    // open the can mask for the specific canDeviceId
    // messages class is 0x500
    if( this->enabledGyro )
    { 
        pCanBus->canIdAdd((CAN_MSG_CLASS_ACC_GYRO)+(boardId<<4)+MSG_TYPE_GYRO);
    }
    pCanBus->canIdAdd((CAN_MSG_CLASS_ACC_GYRO)+(boardId<<4)+MSG_TYPE_ACC);

    channelsNum=CANBUS_INERTIAL_MTB_CHANNELS;
    data.resize(channelsNum);
    data.zero();
    privateData.resize(channelsNum);
    privateData.zero();

    RateThread::start();
    return true;
}

bool CanBusInertialMTB::close()
{
    //stop the thread
    RateThread::stop();

    //stop the driver
    if (pCanBufferFactory)
    {
        pCanBufferFactory->destroyBuffer(inBuffer);
        pCanBufferFactory->destroyBuffer(outBuffer);
    }
    driver.close();

    return true;
}

int CanBusInertialMTB::read(yarp::sig::Vector &out)
{
    int tmp=0;
    mutex.wait();
    out=data;
    tmp=this->status;
    mutex.post();

    return tmp;
}

int CanBusInertialMTB::getState(int ch)
{
    int tmp=0;
    mutex.wait();
    tmp=this->status;
    mutex.post();
    return tmp;
}

int CanBusInertialMTB::getChannels()
{
    return channelsNum;
}

int CanBusInertialMTB::calibrateSensor()
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusInertialMTB::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusInertialMTB::calibrateSensor(const yarp::sig::Vector& v)
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusInertialMTB::calibrateChannel(int ch)
{
    //NOT YET IMPLEMENTED
    return 0;
}

bool CanBusInertialMTB::threadInit()
{    
    unsigned int canMessages=0;
    unsigned id = 0x200 + boardId;
    

    CanMessage &msg=outBuffer[0];
    msg.setId(id);
    msg.getData()[0]=ICUBCANPROTO_POL_SK_CMD__ACC_GYRO_SETUP; // message type
    msg.getData()[1]=this->enabledSensors; // = enable the desired sensors
    msg.getData()[2]=this->sensorPeriod; // period (ms)
    msg.setLen(3);
    canMessages=0;
    bool ret = pCanBus->canWrite(outBuffer, 1, &canMessages);
        
    if( !ret )
    {
        yError("CanBusInertialMTB: canWrite returned false");
        return false;
    } 

    return true;
}

void CanBusInertialMTB::run()
{
    unsigned int canMessages=0;

    bool res=pCanBus->canRead(inBuffer,CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
        yError("CanBusInertialMTB::run(): canRead failed\n");

    double timeNow=Time::now();
    
    int st=IAnalogSensor::AS_OK;   // reading 0 messages is considered ok
    //static double prev = yarp::os::Time::now();
    //fprintf(stderr, "period %f delta %f canMessages %d\n",this->getRate(), yarp::os::Time::now()-prev, canMessages );
    //prev = yarp::os::Time::now();

    if(canMessages <0)
    {
        yError("CanBusInertialMTB::run() ERROR: get %d canMessages\n", canMessages);
        st=IAnalogSensor::AS_ERROR;
    }

    if(canMessages == 0)
    {
        // reading 0 messages is considered ok, so just return;
        st=IAnalogSensor::AS_OK;
    }

    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=inBuffer[i];

        unsigned int msgid    = msg.getId();
        unsigned char *buff   = msg.getData();
        unsigned int len      = msg.getLen();
        unsigned char   id      = ((msgid & 0x00f0)>>4);
        unsigned int   msg_class  = ((msgid & 0x0700));
        unsigned char  msg_type   = ((msgid & 0x000f));

        //parse data here
        if (msg_class==CAN_MSG_CLASS_ACC_GYRO && id==boardId && msg_type == MSG_TYPE_ACC)
        {
            this->accTimeStamp=timeNow;
            privateData[0]= (signed short) ((buff[1]<<8) + buff[0]);
            privateData[1]= (signed short) ((buff[3]<<8) + buff[2]);
            privateData[2]= (signed short) ((buff[5]<<8) + buff[4]);
        }

        if (this->enabledGyro && msg_class==CAN_MSG_CLASS_ACC_GYRO && id==boardId && msg_type == MSG_TYPE_GYRO)
        {
            this->gyroTimeStamp=timeNow;
            privateData[3]= (signed short) (buff[1]<<8)+buff[0];
            privateData[4]= (signed short) (buff[3]<<8)+buff[2];
            privateData[5]= (signed short) (buff[5]<<8)+buff[4];
        }

        st=IAnalogSensor::AS_OK;
    }

    if(initted)
    {
        //if 100ms have passed since the last received message for gyro
        if (timeNow-this->gyroTimeStamp>CANBUS_INERTIAL_MTB_TIMEOUT && this->enabledGyro)
        {
            yError("CanBusInertialMTB::run(): gyroscope read timed out (last received %lf sec ago)",timeNow-this->gyroTimeStamp);
            st=IAnalogSensor::AS_TIMEOUT;
        }

        //if 100ms have passed since the last received message for acc
        if (timeNow-this->accTimeStamp>CANBUS_INERTIAL_MTB_TIMEOUT)
        {
            yError("CanBusInertialMTB::run(): accelerometer read timed out (last received %lf sec ago)",timeNow-this->accTimeStamp);
            st=IAnalogSensor::AS_TIMEOUT;
        }
    }
    else
    {
        // wait some time to have the device ready and avoid spurious timeout messages at startup
        count++;
        if(count == 10)
            initted=true;
    }

    mutex.wait();
    memcpy(data.data(), privateData.data(), sizeof(double)*privateData.size());
    this->status=st;
    mutex.post();

    return;
}

void CanBusInertialMTB::threadRelease()
{
    //Send a message for stopping the streaming
    unsigned int canMessages=0;
    unsigned id = 0x200 + boardId;

    CanMessage &msg=outBuffer[0];
    msg.setId(id);
    msg.getData()[0]=ICUBCANPROTO_POL_SK_CMD__ACC_GYRO_SETUP; // message type
    msg.getData()[1]=0; // = disable all sensors
    msg.getData()[2]=0x01; // period (ms)
    msg.setLen(3);
    canMessages=0;
    pCanBus->canWrite(outBuffer, 1, &canMessages);
    
    return;
}

