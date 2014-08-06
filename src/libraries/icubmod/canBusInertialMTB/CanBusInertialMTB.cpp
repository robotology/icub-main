// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo and Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusInertialMTB.h>

#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

const int    CANBUS_INERTIAL_MTB_CAN_DRIVER_BUFFER_SIZE=2047;
const int    CANBUS_INERTIAL_MTB_CHANNELS=6;
const double CANBUS_INERTIAL_MTB_TIMEOUT=0.1; //100ms
const int    CAN_MSG_CLASS_ACCELEROMETER=0x5;

using namespace std;

bool CanBusInertialMTB::open(yarp::os::Searchable& config)
{
    bool correct=true;

    correct &= config.check("canbusDevice");
    correct &= config.check("canDeviceNum");
    correct &= config.check("canAddress");
    
    if (!correct)
    {
        fprintf(stderr, "Error: insufficient parameters to CanBusInertialMTB\n"); 
        return false;
    }

    if (config.check("period")==true)
    {
        int period=10;
        period=config.find("period").asInt();
        setRate(period);
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
    this->boardId           = config.find("canAddress").asInt();
   
    // open the can mask for the specific canDeviceId
    // messages class is 0x500
    pCanBus->canIdAdd((CAN_MSG_CLASS_ACCELEROMETER<<8)+(boardId<<4)+0);
    pCanBus->canIdAdd((CAN_MSG_CLASS_ACCELEROMETER<<8)+(boardId<<4)+1);
    pCanBus->canIdAdd((CAN_MSG_CLASS_ACCELEROMETER<<8)+(boardId<<4)+2);

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
    /*static int initialized=-10;

    if (initialized<0)
    {
        unsigned int canMessages=0;
        unsigned id = 0x200 + boardId;

        CanMessage &msg=outBuffer[0];
        msg.setId(id);
        msg.getData()[0]=0x4E; // message type
        msg.getData()[1]=0x02; 
        msg.getData()[2]=0x22; 
        msg.getData()[3]=0xF0;
        msg.getData()[4]=0x0E;    //accelerometer, gyros etc
        msg.getData()[5]=0xFF;
        msg.getData()[6]=0xFF;
        msg.getData()[7]=0x0A;
        msg.setLen(8);
        canMessages=0;
        pCanBus->canWrite(outBuffer, 1, &canMessages);

        initialized++;

        fprintf(stderr, "CanBusInertialMTB::read1\n");
    }*/

    int tmp=0;
    mutex.wait();
    out=data;
    tmp=status;
    mutex.post();

    return tmp;
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
    msg.getData()[0]=0x4F; // message type
    msg.getData()[1]=0x02; // = enable digital accelerometer and gyroscope
    msg.getData()[2]=0x05; // period (ms)
    msg.setLen(3);
    canMessages=0;
    pCanBus->canWrite(outBuffer, 1, &canMessages);

    //this->setRate(10);
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

    int st=IAnalogSensor::AS_OK;   // reading 0 messages is considered ok
    //static double prev = yarp::os::Time::now();
    //fprintf(stderr, "period %f delta %f canMessages %d\n",this->getRate(), yarp::os::Time::now()-prev, canMessages );
    //prev = yarp::os::Time::now();
    if(canMessages <0)
    {
        fprintf(stderr, "CanBusInertialMTB::run() ERROR: get %d canMessages\n", canMessages);
        st=IAnalogSensor::AS_ERROR;
        return;
    }

    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=inBuffer[i];

        unsigned int msgid    = msg.getId();
        unsigned char *buff   = msg.getData();
        unsigned int len      = msg.getLen();
        unsigned char   id      = ((msgid & 0x00f0)>>4);
        unsigned char   mclass  = ((msgid & 0x0700)>>8);
        unsigned char   mtype   = ((msgid & 0x000f));

        //parse data here
        if (mclass==CAN_MSG_CLASS_ACCELEROMETER && id==boardId && mtype == 1)
        {
            stampAcc=Time::now();
            privateData[0]= (signed short) ((buff[1]<<8) + buff[0]);
            privateData[1]= (signed short) ((buff[3]<<8) + buff[2]);
            privateData[2]= (signed short) ((buff[5]<<8) + buff[4]);
        }
        
        if (mclass==CAN_MSG_CLASS_ACCELEROMETER && id==boardId && mtype == 2)
        {
            stampGyro=Time::now();
            privateData[3]=(buff[1]<<8)+buff[0];
            privateData[4]=(buff[3]<<8)+buff[2];
            privateData[5]=(buff[5]<<8)+buff[4];
        }

        st=IAnalogSensor::AS_OK;
    }

    //if 100ms have passed since the last received message
    /*if (timeNow-timeStamp>CANBUS_INERTIAL_MTB_TIMEOUT)
    {
        st=IAnalogSensor::AS_TIMEOUT;
    }*/

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

