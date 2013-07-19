// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo and Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusInertialMTB.h>

#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

const int CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE=2047;
const int CANBUS_INERTIAL_MTB_CHANNELS=6;
const double CANBUS_INERTIAL_MTB_TIMEOUT=0.1; //100ms
const int CAN_MSG_CLASS_ACCELEROMETER=0x5;
const int CAN_MSG_CLASS_GYRO=0x6;


using namespace std;

bool CanBusInertialMTB::open(yarp::os::Searchable& config)
{
    bool correct=true;

    correct &= config.check("CanbusDevice");
    correct &= config.check("CanDeviceNum");
    correct &= config.check("CanAddress");
    correct &= config.check("Period");
    
    if (!correct)
    {
        fprintf(stderr, "Error: insufficient parameters to CanBusAnalogSensor\n"); 
        return false;
    }

    int period=config.find("Period").asInt();
    setRate(period);

    Property prop;

    prop.put("device", config.find("CanbusDevice").asString().c_str());
    prop.put("physdevice", config.find("physdevice").asString().c_str());
    prop.put("CanTxTimeout", 500);
    prop.put("CanRxTimeout", 500);
    prop.put("CanDeviceNum", config.find("CanDeviceNum").asInt());
    prop.put("CanMyAddress", 0);
    prop.put("CanTxQueueSize", CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);
    prop.put("CanRxQueueSize", CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);
    pCanBus=0;
    pCanBufferFactory=0;

    //open the can driver
    driver.open(prop);
    if (!driver.isValid())
    {
        fprintf(stderr, "Error opening CanBusInertialMTB check parameters\n");
        return false;
    }
    driver.view(pCanBus);
    if (!pCanBus)
    {
        fprintf(stderr, "Error opening can device not available\n");
        return false;
    }
    driver.view(pCanBufferFactory);
    outBuffer=pCanBufferFactory->createBuffer(CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE);

    //select the communication speed
    pCanBus->canSetBaudRate(0); //default 1MB/s


    //set the internal configuration
    this->boardId           = config.find("CanAddress").asInt();
   
    // open the can mask for the specific canDeviceId
    // messages class is 0x500
    pCanBus->canIdAdd((CAN_MSG_CLASS_ACCELEROMETER<<8)+(boardId<<4));
    pCanBus->canIdAdd((CAN_MSG_CLASS_GYRO<<8)+(boardId<<4));

    channelsNum=CANBUS_INERTIAL_MTB_CHANNELS;
    data.resize(channelsNum);
    privateData.resize(channelsNum);

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
    tmp=status;
    mutex.post();

    return status;
}

int CanBusInertialMTB::getState(int ch)
{
    int tmp=0;
    mutex.wait();
    tmp=status;
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
    msg.getData()[0]=0x4C; // message type
    msg.getData()[1]=0x01; 
    msg.getData()[2]=0x01; 
    msg.getData()[3]=0x01;
    msg.getData()[4]=0x07;    //accelerometer, gyros etc
    msg.getData()[5]=0x22;
    msg.getData()[6]=0;
    msg.getData()[7]=0;
    msg.setLen(8);
    canMessages=0;
    pCanBus->canWrite(outBuffer, 1, &canMessages);

    return true;
}

void CanBusInertialMTB::run()
{    
    unsigned int canMessages=0;
    bool ret=true; //return true by default

    bool res=pCanBus->canRead(inBuffer,CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
        fprintf(stderr, "CanBusInertialMTB::run(): canRead failed\n");

    double timeNow=Time::now();
    double stampGyro=0.0;
    double stampAcc=0.0;

    int st=IAnalogSensor::AS_ERROR;
    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=inBuffer[i];

        unsigned int msgid    = msg.getId();
        unsigned char *buff   = msg.getData();
        unsigned int len      = msg.getLen();
        unsigned int id       = (msgid & 0x00f0)>>4;
        const char   type     = ((msgid&0x700)>>8);

        //parse data here
        if (type==(CAN_MSG_CLASS_ACCELEROMETER) && id==boardId)
        {
            stampAcc=Time::now();
            privateData[0]=(buff[1]<<8)+buff[0];
            privateData[1]=(buff[3]<<8)+buff[2];
            privateData[2]=(buff[5]<<8)+buff[4];
        }
        
        if (type==CAN_MSG_CLASS_GYRO && id==boardId)
        {
            stampGyro=Time::now();
            privateData[3]=(buff[1]<<8)+buff[0];
            privateData[4]=(buff[3]<<8)+buff[2];
            privateData[5]=(buff[5]<<8)+buff[4];
        }

        st=IAnalogSensor::AS_OK;
    }

    //if 100ms have passed since the last received message
    if (timeNow-timeStamp>CANBUS_INERTIAL_MTB_TIMEOUT)
    {
        st=IAnalogSensor::AS_TIMEOUT;
    }

    mutex.wait();
    memcpy(data.data(), privateData.data(), sizeof(double)*privateData.size());
    status=st;
    mutex.post();
}

void CanBusInertialMTB::threadRelease()
{
    printf("CanBusInertialMTB Thread releasing...\n");
    printf("... done.\n");
}

