// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusAnalogSensor.h>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <string.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

using namespace std;

bool CanBusAnalogSensor::open(yarp::os::Searchable& config)
{
    bool correct=true;

    //debug
    fprintf(stderr, "%s\n", config.toString().c_str());

    correct &= config.check("canbusDevice");
    correct &= config.check("canDeviceNum");
    correct &= config.check("canAddress");
    correct &= config.check("format");
    correct &= config.check("period");
    correct &= config.check("channels");
    correct &= config.check("physDevice");
    
    if (!correct)
    {
        yError() << "Insufficient parameters to CanBusAnalogSensor\n"; 
        return false;
    }

    int period=config.find("period").asInt();
    setRate(period);

    Property prop;

    prop.put("device", config.find("canbusDevice").asString().c_str());
    prop.put("physDevice", config.find("physDevice").asString().c_str());
    prop.put("canTxTimeout", 500);
    prop.put("canRxTimeout", 500);
    canDeviceNum = config.find("canDeviceNum").asInt();
    prop.put("canDeviceNum", canDeviceNum);
    prop.put("canMyAddress", 0);
    prop.put("canTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    prop.put("canRxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    pCanBus=0;
    pCanBufferFactory=0;

    //open the can driver
    driver.open(prop);
    if (!driver.isValid())
    {
        yError() << "Error opening PolyDriver check parameters";
        return false;
    }
    driver.view(pCanBus);
    if (!pCanBus)
    {
        yError() << "Error opening can device not available";
        return false;
    }
    driver.view(pCanBufferFactory);
    outBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);

    //select the communication speed
    pCanBus->canSetBaudRate(0); //default 1MB/s


    //set the internal configuration
    //this->isVirtualSensor   = false;
    this->boardId           = config.find("canAddress").asInt();
    this->useCalibration    = config.find("useCalibration").asInt();
    //this->SensorFullScale = config.find("fullScale").asInt();
    unsigned int tmpFormat  = config.find("format").asInt();
    if      (tmpFormat == 8)
        this->dataFormat = ANALOG_FORMAT_8_BIT;
    else if (tmpFormat == 16)
        this->dataFormat = ANALOG_FORMAT_16_BIT;
    else    
        this->dataFormat = ANALOG_FORMAT_ERR;

    //open the can mask for the specific canDeviceId
    for (int id=0; id<16; ++id)
    {
        pCanBus->canIdAdd(0x300+(boardId<<4)+id);
    }
    pCanBus->canIdAdd(0x200+boardId);
    pCanBus->canIdAdd(0x200+(boardId<<4));

    //create the data vector:
    this->channelsNum    = config.find("channels").asInt();
    data.resize(channelsNum);
    data.zero();
    scaleFactor.resize(channelsNum);
    scaleFactor.zero();

    //start the sensor broadcast
    sensor_start(config);

    RateThread::start();
    return true;
}

bool CanBusAnalogSensor::readFullScaleAnalog(int ch)
{
    scaleFactor[ch]=1e-20;

    unsigned int canMessages=0;
    unsigned id = 0x200 + boardId;
    CanMessage &msg=outBuffer[0];
    msg.setId(id);
    msg.getData()[0]=0x18;
    msg.getData()[1]=ch;
    msg.setLen(2);
    canMessages=0;
    pCanBus->canWrite(outBuffer, 1, &canMessages);

    long int timeout=0;
    bool full_scale_read=false;
    do 
    {
        unsigned int max_messages=CAN_DRIVER_BUFFER_SIZE;
        unsigned int read_messages = 0;
        bool b = pCanBus->canRead(inBuffer,max_messages,&read_messages,false);

        for (unsigned int i=0; i<read_messages; i++)
        {
            CanMessage& m= inBuffer[i];
            unsigned int currId=m.getId();
            if (currId==(0x0200 | boardId << 4))
                if (m.getLen()==4 &&
                    m.getData()[0]==0x18 &&
                    m.getData()[1]==ch)
                    {
                        scaleFactor[ch]=m.getData()[2]<<8 | m.getData()[3];
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
            yError("Trying to get fullscale data from sensor %d net [%d]: no answer received or message lost (ch:%d)\n", boardId, canDeviceNum, ch);
            return false;
        }

    return true;
}

bool CanBusAnalogSensor::sensor_start(yarp::os::Searchable& analogConfig)
{
    yTrace("Initializing analog device %s\n", analogConfig.find("deviceId").toString().c_str());

    unsigned int canMessages=0;
    unsigned id = 0x200 + boardId;

    if (analogConfig.check("period"))
    {
        int period=analogConfig.find("period").asInt();
        CanMessage &msg=outBuffer[0];
        msg.setId(id);
        msg.getData()[0]=0x08;
        msg.getData()[1]=period; 
        msg.setLen(2);
        canMessages=0;
        pCanBus->canWrite(outBuffer, 1, &canMessages);
        yDebug("using broadcast period %d on device %s\n", period, analogConfig.find("deviceId").toString().c_str());
    }

    //init message for mais board
    if (channelsNum==16 && dataFormat==ANALOG_FORMAT_8_BIT)
    {
        CanMessage &msg=outBuffer[0];
        msg.setId(id);
        msg.getData()[0]=0x07;
        msg.getData()[1]=0x00; 
        msg.setLen(2);
        canMessages=0;
        pCanBus->canWrite(outBuffer, 1, &canMessages);
    }

    //init message for strain board
    else if (channelsNum==6 && dataFormat==ANALOG_FORMAT_16_BIT)
    {
        //calibrated astrain board
        if (useCalibration>0)
        {
            yDebug("Using internal calibration on device %s\n", analogConfig.find("deviceId").toString().c_str());
            //get the full scale values from the strain board
            for (int ch=0; ch<6; ch++)
            {
                bool b=false;
                int attempts = 0;
                while(attempts<15) 
                {
                    b = readFullScaleAnalog(ch);
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
                    yError("Trying to get fullscale data from sensor: all attempts failed (ch:%d)\n", ch);
                }
            }

            // debug messages
            #if 1
                    fprintf(stderr, "Sensor Fullscale Id %#4X: ",boardId);
                    fprintf(stderr, " %f ", scaleFactor[0]);
                    fprintf(stderr, " %f ", scaleFactor[1]);
                    fprintf(stderr, " %f ", scaleFactor[2]);
                    fprintf(stderr, " %f ", scaleFactor[3]);
                    fprintf(stderr, " %f ", scaleFactor[4]);
                    fprintf(stderr, " %f ", scaleFactor[5]);
                    fprintf(stderr, " \n ");
            #endif

            // start the board
            CanMessage &msg=outBuffer[0];
            msg.setId(id);
            msg.getData()[0]=0x07;
            if (useCalibration == 1)  msg.getData()[1]=0x00;
            if (useCalibration == 2)  msg.getData()[1]=0x04;
            if (useCalibration == 3)  msg.getData()[1]=0x05;
            msg.setLen(2);
            canMessages=0;
            pCanBus->canWrite(outBuffer, 1, &canMessages);
        }
        //not calibrated strain board (useCalibration = 0)
        else
        {
            yInfo("Using uncalibrated raw data for device %s\n", analogConfig.find("deviceId").toString().c_str());
            CanMessage &msg=outBuffer[0];
            msg.setId(id);
            msg.getData()[0]=0x07;
            msg.getData()[1]=0x03; 
            msg.setLen(2);
            canMessages=0;
            pCanBus->canWrite(outBuffer, 1, &canMessages);
        }
    }
    return true;
}

bool CanBusAnalogSensor::sensor_stop()
{
    unsigned int canMessages=0;
    unsigned id = 0x200 + boardId;
    CanMessage &msg=outBuffer[0];
    msg.setId(id);
    msg.getData()[0]=0x07;
    msg.getData()[1]=0x01; 
    msg.setLen(2);
    canMessages=0;
    pCanBus->canWrite(outBuffer, 1, &canMessages);
    return true;
}

bool CanBusAnalogSensor::close()
{
    //stop the thread
    RateThread::stop();

    //stop the sensor
    sensor_stop();

    //stop the driver
    if (pCanBufferFactory)
    {
        pCanBufferFactory->destroyBuffer(inBuffer);
        pCanBufferFactory->destroyBuffer(outBuffer);
    }
    driver.close();

    return true;
}

int CanBusAnalogSensor::read(yarp::sig::Vector &out) 
{
    mutex.wait();
    out=data;
    mutex.post();

    return yarp::dev::IAnalogSensor::AS_OK;
}

int CanBusAnalogSensor::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

int CanBusAnalogSensor::getChannels()
{
    return channelsNum;
}

int CanBusAnalogSensor::calibrateSensor()
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusAnalogSensor::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusAnalogSensor::calibrateSensor(const yarp::sig::Vector& v)
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusAnalogSensor::calibrateChannel(int ch)
{
    //NOT YET IMPLEMENTED
    return 0;
}

bool CanBusAnalogSensor::threadInit()
{
    return true;
}

bool CanBusAnalogSensor::decode16(const unsigned char *msg, int msg_id, double *data)
{
    const char groupId=(msg_id & 0x00f);
    int baseIndex=0;
    {
        switch (groupId)
        {
        case 0xA:
            {
                for(int k=0;k<3;k++)
                    {
                        data[k]=(((unsigned short)(msg[2*k+1]))<<8)+msg[2*k]-0x8000;
                        if (useCalibration>0)
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
                        if (useCalibration>0)
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
            yWarning("Got unexpected class 0x3 msg(s): groupId 0x%x\n", groupId);
            return false;
            break;
        }
        //@@@DEBUG ONLY
        //fprintf(stderr, "   %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    }

    return true;
}

bool CanBusAnalogSensor::decode8(const unsigned char *msg, int msg_id, double *data)
{
    const char groupId=(msg_id & 0x00f);
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
            yWarning("CanBusAnalogSensor got unexpected class 0x3 msg(s): groupId 0x%x\n", groupId);
            return false;
            break;
        }
        //@@@DEBUG ONLY
        //fprintf(stderr, "   %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f %+8.1f\n",data[0],data[1],data[2],data[3],data[4],data[5],data[6]);
    }
    return true;
}

void CanBusAnalogSensor::run()
{    
    mutex.wait();

    unsigned int canMessages=0;
    bool ret=true; //return true by default

    bool res=pCanBus->canRead(inBuffer,CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
    {
        yError()<<"CanBusAnalogSensor canRead failed\n";
    }

    double timeNow=Time::now();
    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=inBuffer[i];

        unsigned int msgid    = msg.getId();
        unsigned char *buff   = msg.getData();
        unsigned int len      = msg.getLen();
        unsigned int id       = (msgid & 0x00f0)>>4;
        const char   type     = ((msgid&0x700)>>8);

        //parse data here
        status=IAnalogSensor::AS_OK;
        
        if (type==0x03) //analog data
        {
            if (id==boardId)
            {
                timeStamp=Time::now();
                switch (dataFormat)
                {
                    case ANALOG_FORMAT_8_BIT:
                        ret=decode8(buff, msgid, data.data());
                        status=IAnalogSensor::AS_OK;
                        break;
                    case ANALOG_FORMAT_16_BIT:
                        if (len==6) 
                        {
                            ret=decode16(buff, msgid, data.data());
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
                            ret=decode16(buff, msgid, data.data());
                        }
                        break;
                    default:
                        status=IAnalogSensor::AS_ERROR;
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
}

void CanBusAnalogSensor::threadRelease()
{
    yTrace("CanBusVirtualAnalogSensor Thread released\n");
}

