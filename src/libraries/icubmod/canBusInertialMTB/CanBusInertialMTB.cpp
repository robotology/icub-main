// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo and Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusInertialMTB.h>

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <cstring>
#include <string>

#include <iCubCanProto_skinMessages.h>

const int    CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE=2047;
const double CANBUS_INERTIAL_MTB_TIMEOUT=0.1; //100ms


// Protocol for inertial sensor messages sent by the MTB
const unsigned int    CAN_MSG_CLASS_ACC_GYRO  = 0x500;
const unsigned int    MSG_TYPE_GYRO           = 0x000;
const unsigned int    MSG_TYPE_ACC           = 0x001;

const unsigned short    CANBUS_INERTIAL_MTB_DEFAULT_SENSOR_PERIOD=10;

const char   CANBUS_INERTIAL_MTB_INTERNAL_ACC_BIT = 0x02;
const char   CANBUS_INERTIAL_MTB_EXTERNAL_ACC_BIT = 0x08;
const char   CANBUS_INERTIAL_MTB_EXTERNAL_GYRO_BIT = 0x04;

using namespace std;


bool checkRequiredParamIsString(yarp::os::Searchable& config,
                                const std::string& paramName)
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
                             const std::string& paramName)
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

bool checkRequiredParamIsVectorOfInt(yarp::os::Searchable& config,
                                     const std::string& paramName,
                                     std::vector<int> & output_vector)
{
    bool correct = !(config.findGroup(paramName).isNull());
    if( correct )
    {
        Bottle ids = config.findGroup(paramName).tail();
        output_vector.resize(ids.size());
        for(int i = 0; i < ids.size(); i++ )
        {
            output_vector[i] = ids.get(i).asInt();
        }
    }

    if( !correct )
    {
        yError("CanBusInertialMTB: problem loading parameter %s as vector of int",paramName.c_str());
    }

    return correct;
}

// \todo TODO bug ? 
bool checkRequiredParamIsVectorOfString(yarp::os::Searchable& config,
                                     const std::string& paramName,
                                     std::vector<std::string> & output_vector)
{
    bool correct = !(config.findGroup(paramName).isNull());
    if( correct )
    correct = true;
    {
        Bottle ids = config.findGroup(paramName).tail();
        std::cout << "ids : " << ids.toString() << std::endl;
        std::cout << "ids : " << config.find(paramName).toString() << std::endl;
        output_vector.resize(ids.size());
        for(int i = 0; i < ids.size(); i++ )
        {
            output_vector[i] = ids.get(i).asString().c_str();
        }
    }

    if( !correct )
    {
        yError("CanBusInertialMTB: problem loading parameter %s as vector of string",paramName.c_str());
    }

    return correct;
}


bool CanBusInertialMTB::validateConf(yarp::os::Searchable& config,
                                     std::vector<int> & canAddresses)
{
    std::cout << "CanBusInertialMTB::validateConf : " << config.toString() << std::endl;
    bool correct=true;

    correct = correct && checkRequiredParamIsString(config,"canbusDevice");
    correct = correct && checkRequiredParamIsInt(config,"canDeviceNum");
    correct = correct && checkRequiredParamIsVectorOfInt(config,"canAddress",canAddresses);
    correct = correct && checkRequiredParamIsString(config,"physDevice");
    correct = correct && checkRequiredParamIsString(config,"sensorType");

    return correct;
}

bool CanBusInertialMTB::open(yarp::os::Searchable& config)
{
    std::vector<int> canAddresses;
    bool correct = this->validateConf(config,canAddresses);

    if (!correct)
    {
        yError("CanBusInertialMTB: Insufficient parameters to CanBusInertialMTB\n");
        return false;
    }

    //Read sensor period for all sensors
    int sensorPeriod = CANBUS_INERTIAL_MTB_DEFAULT_SENSOR_PERIOD;
    if (config.check("sensorPeriod"))
    {
        int int_sensorPeriod = config.find("sensorPeriod").asInt();
        if( int_sensorPeriod < 1 || int_sensorPeriod > 255 )
        {
            yError("CanBusInertialMTB: sensorPeriod is lower than 1 or bigger then 255\n");
            return false;
        }
        sensorPeriod = int_sensorPeriod;
    }

    //Parse sensor type and address of all readed sensors
    this->boards.resize(canAddresses.size());
    this->nrOfTotalChannels = 0;
    

    for(size_t board=0; board < this->boards.size(); board++ )
    {
        this->boards[board].boardId = canAddresses[board];
        if( config.find("sensorType").asString() == "acc" )
        {
            this->boards[board].enabledSensors = CANBUS_INERTIAL_MTB_INTERNAL_ACC_BIT;
            this->boards[board].enabledGyro    = false;
            this->boards[board].nrOfChannels   = 3;
            this->boards[board].vectorOffset   = this->nrOfTotalChannels;
            this->nrOfTotalChannels += this->boards[board].nrOfChannels;
        }
        else if( config.find("sensorType").asString() == "extAccAndGyro" )
        {
            this->boards[board].enabledSensors = CANBUS_INERTIAL_MTB_EXTERNAL_GYRO_BIT |
                                                 CANBUS_INERTIAL_MTB_EXTERNAL_ACC_BIT;
            this->boards[board].enabledGyro    = true;
            this->boards[board].nrOfChannels   = 6;
            this->boards[board].vectorOffset   = this->nrOfTotalChannels;
            this->nrOfTotalChannels += this->boards[board].nrOfChannels;
        }
        else
        {
            yError("CanBusInertialMTB: unknown sensorType %s",config.find("sensorType").asString().c_str());
            return false;
        }
    }

    if (config.check("period")==true)
    {
        int period=10;
        period=config.find("period").asInt();
        setPeriod((double)period/1000.0);
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
        yError("Unable to open CanBusInertialMTB check parameters\n");
        return false;
    }
    driver.view(pCanBus);
    if (!pCanBus)
    {
        yError("Unable to open CAN device not available\n");
        return false;
    }
    driver.view(pCanBufferFactory);
    outBuffer=pCanBufferFactory->createBuffer(CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);

    //select the communication speed
    pCanBus->canSetBaudRate(0); //default 1MB/s

    // open the can mask for the desired canDeviceId
    // messages class is 0x500
    for(size_t board=0; board < this->boards.size(); board++ )
    {
        unsigned short     boardId = this->boards[board].boardId;
        if( this->boards[board].enabledGyro )
        {
            pCanBus->canIdAdd((CAN_MSG_CLASS_ACC_GYRO)+(boardId<<4)+MSG_TYPE_GYRO);
        }
        pCanBus->canIdAdd((CAN_MSG_CLASS_ACC_GYRO)+(boardId<<4)+MSG_TYPE_ACC);

    }

    data.resize(this->nrOfTotalChannels);
    data.zero();
    privateData.resize(this->nrOfTotalChannels);
    privateData.zero();
    sharedStatus.resize(this->nrOfTotalChannels,IAnalogSensor::AS_OK);
    privateStatus.resize(this->nrOfTotalChannels,IAnalogSensor::AS_OK);

    PeriodicThread::start();
    return true;
}

bool CanBusInertialMTB::close()
{
    //stop the thread
    PeriodicThread::stop();

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
    lock_guard<mutex> lck(mtx);
    out=data;
    return this->sharedGlobalStatus;
}

int CanBusInertialMTB::getState(int ch)
{
    lock_guard<mutex> lck(mtx);
    return this->sharedStatus[ch];
}

int CanBusInertialMTB::getChannels()
{
    return this->nrOfTotalChannels;
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
    for(size_t board=0; board < this->boards.size(); board++ )
    {
        unsigned int canMessages=0;
        unsigned id = 0x200 + this->boards[board].boardId;

        CanMessage &msg=outBuffer[0];
        msg.setId(id);
        msg.getData()[0]=ICUBCANPROTO_POL_SK_CMD__ACC_GYRO_SETUP; // message type
        msg.getData()[1]=this->boards[board].enabledSensors; // = enable the desired senssors
        msg.getData()[2]=this->boards[board].sensorPeriod; // period (ms)
        msg.setLen(3);
        canMessages=0;
        bool ret = pCanBus->canWrite(outBuffer, 1, &canMessages);

        if( !ret )
        {
            yError("CanBusInertialMTB: canWrite returned false for sensor with address %x",this->boards[board].boardId);
            return false;
        }
    }

    return true;
}

void CanBusInertialMTB::setPrivateBoardStatus(int boardIndex, short status)
{
    int vectorOffset = this->boards[boardIndex].vectorOffset;
    int nrOfBoardChannels = this->boards[boardIndex].nrOfChannels;
    for(int i=0; i < nrOfBoardChannels; i++ )
    {
        privateStatus[vectorOffset+i] = status;
    }
}

void CanBusInertialMTB::setPrivateBoardGyroStatus(int boardIndex, short status)
{
    int vectorOffset = this->boards[boardIndex].vectorOffset;
    int nrOfBoardChannels = this->boards[boardIndex].nrOfChannels;
    for(int i=3; i < 6; i++ )
    {
        privateStatus[vectorOffset+i] = status;
    }
}

void CanBusInertialMTB::setPrivateBoardAccStatus(int boardIndex, short status)
{
    int vectorOffset = this->boards[boardIndex].vectorOffset;
    int nrOfBoardChannels = this->boards[boardIndex].nrOfChannels;
    for(int i=0; i < 3; i++ )
    {
        privateStatus[vectorOffset+i] = status;
    }
}

void CanBusInertialMTB::run()
{
    unsigned int canMessages=0;

    bool res=pCanBus->canRead(inBuffer,CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
        yError("CanBusInertialMTB::run(): canRead failed\n");

    double timeNow=Time::now();

    if(canMessages <0)
    {
        yError("CanBusInertialMTB::run() get %d canMessages\n", canMessages);
        for(size_t board=0; board < this->boards.size(); board++ )
        {
            setPrivateBoardStatus(board,IAnalogSensor::AS_ERROR);
        }

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

        // \todo TODO substitute the double for with a boardId -> board index vector? Worth?
        for(size_t board=0; board < this->boards.size(); board++ )
        {
            unsigned short     boardId = this->boards[board].boardId;
            unsigned int       vectorOffset = this->boards[board].vectorOffset;
            //parse data here
            if (id==boardId && msg_class==CAN_MSG_CLASS_ACC_GYRO && msg_type == MSG_TYPE_ACC)
            {
                this->boards[board].accTimeStamp=timeNow;
                privateData[vectorOffset+0]= (signed short) ((buff[1]<<8) + buff[0]);
                privateData[vectorOffset+1]= (signed short) ((buff[3]<<8) + buff[2]);
                privateData[vectorOffset+2]= (signed short) ((buff[5]<<8) + buff[4]);
                setPrivateBoardAccStatus(board,IAnalogSensor::AS_OK);
            }

            if (id==boardId && this->boards[board].enabledGyro && msg_class==CAN_MSG_CLASS_ACC_GYRO && msg_type == MSG_TYPE_GYRO)
            {
                this->boards[board].gyroTimeStamp=timeNow;
                privateData[vectorOffset+3]= (signed short) (buff[1]<<8)+buff[0];
                privateData[vectorOffset+4]= (signed short) (buff[3]<<8)+buff[2];
                privateData[vectorOffset+5]= (signed short) (buff[5]<<8)+buff[4];
                setPrivateBoardGyroStatus(board,IAnalogSensor::AS_OK);
            }

        }

    }


    if(initted)
    {
        for(size_t board=0; board < this->boards.size(); board++ )
        {
            unsigned short  boardId = this->boards[board].boardId;

            //if 100ms have passed since the last received message for gyro
            double delay = timeNow-this->boards[board].gyroTimeStamp;
            if (delay > CANBUS_INERTIAL_MTB_TIMEOUT && this->boards[board].enabledGyro)
            {
                yError("CanBusInertialMTB::run(): gyroscope read for board %x timed out (last received %lf sec ago)",boardId,delay);
                setPrivateBoardGyroStatus(board,IAnalogSensor::AS_TIMEOUT);
            }

            //if 100ms have passed since the last received message for acc
            delay = timeNow-this->boards[board].accTimeStamp;
            if (delay > CANBUS_INERTIAL_MTB_TIMEOUT)
            {
                yError("CanBusInertialMTB::run(): accelerometer read for board %x timed out (last received %lf sec ago)",boardId,delay);
                setPrivateBoardAccStatus(board,IAnalogSensor::AS_TIMEOUT);
            }
        }
    }
    else
    {
        // wait some time to have the device ready and avoid spurious timeout messages at startup
        count++;
        if(count == 10)
            initted=true;
    }


    //Compute the global status
    this->privateGlobalStatus = IAnalogSensor::AS_OK;
    for(int ch=0; ch < this->nrOfTotalChannels; ch+= 3)
    {
        if( this->privateStatus[ch] == IAnalogSensor::AS_ERROR )
        {
            this->privateGlobalStatus = IAnalogSensor::AS_ERROR;
            break;
        }
        if( this->privateStatus[ch] == IAnalogSensor::AS_TIMEOUT )
        {
            this->privateGlobalStatus = IAnalogSensor::AS_TIMEOUT;
            break;
        }
    }

    // Copy the data in the output data
    lock_guard<mutex> lck(mtx);
    memcpy(data.data(), privateData.data(), sizeof(double)*privateData.size());
    for(size_t ch=0; ch < privateStatus.size(); ch++ )
    {
        this->sharedStatus[ch] =  this->privateStatus[ch];
    }
    this->sharedGlobalStatus = this->privateGlobalStatus;

    return;
}

void CanBusInertialMTB::threadRelease()
{
    for(size_t board=0; board < this->boards.size(); board++ )
    {
        unsigned int canMessages=0;
        unsigned id = 0x200 + this->boards[board].boardId;

        CanMessage &msg=outBuffer[0];
        msg.setId(id);
        msg.getData()[0]=ICUBCANPROTO_POL_SK_CMD__ACC_GYRO_SETUP; // message type
        msg.getData()[1]=0x0; // disable all the sensors
        msg.getData()[2]=0x1; // period (ms)
        msg.setLen(3);
        canMessages=0;
        bool ret = pCanBus->canWrite(outBuffer, 1, &canMessages);

        if( !ret )
        {
            yError("CanBusInertialMTB: canWrite returned false for sensor with address %x",this->boards[board].boardId);
        }
    }
    return;
}

