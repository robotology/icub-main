// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusAnalogSensor.h>

#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

using namespace std;

bool CanBusAnalogSensor::open(yarp::os::Searchable& config)
{
    bool correct=true;

    //debug
    fprintf(stderr, "%s\n", config.toString().c_str());

    correct &= config.check("CanbusDevice");
    correct &= config.check("CanDeviceNum");
    correct &= config.check("CanAddress");
    correct &= config.check("Format");
    correct &= config.check("Period");
    correct &= config.check("Channels");
    
    if (!correct)
    {
        std::cerr<<"Error: insufficient parameters to CanBusAnalogSensor\n"; 
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
    prop.put("CanTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    prop.put("CanRxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    pCanBus=0;
    pCanBufferFactory=0;

    //open the can driver
    driver.open(prop);
    if (!driver.isValid())
    {
        fprintf(stderr, "Error opening PolyDriver check parameters\n");
        return false;
    }
    driver.view(pCanBus);
    if (!pCanBus)
    {
        fprintf(stderr, "Error opening can device not available\n");
        return false;
    }
    driver.view(pCanBufferFactory);
    outBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);

    //select the communication speed
    pCanBus->canSetBaudRate(0); //default 1MB/s


    //set the internal configuration
    //this->isVirtualSensor   = false;
    this->boardId           = config.find("CanAddress").asInt();
    this->analogChannels    = config.find("Channels").asInt();
    this->useCalibration    = (config.find("UseCalibration").asInt()==1);
    //this->SensorFullScale = config.find("FullScale").asInt();
    unsigned int tmpFormat  = config.find("Format").asInt();
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
    int chan=config.find("Channels").asInt();
    sensorsNum = chan>=1?chan:1;
    data.resize(sensorsNum);
    scaleFactor.resize(sensorsNum);

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
            fprintf(stderr, "*** ERROR: Trying to get fullscale data from sensor: no answer received or message lost (ch:%d)\n", ch);
            return false;
        }

    return true;
}

bool CanBusAnalogSensor::sensor_start(yarp::os::Searchable& analogConfig)
{
    fprintf(stderr, "--> Initializing analog device %s\n", analogConfig.find("deviceId").toString().c_str());

    unsigned int canMessages=0;
    unsigned id = 0x200 + boardId;

    if (analogConfig.check("Period"))
    {
        int period=analogConfig.find("Period").asInt();
        CanMessage &msg=outBuffer[0];
        msg.setId(id);
        msg.getData()[0]=0x08;
        msg.getData()[1]=period; 
        msg.setLen(2);
        canMessages=0;
        pCanBus->canWrite(outBuffer, 1, &canMessages);
    }

    //init message for mais board
    if (analogChannels==16 && dataFormat==ANALOG_FORMAT_8_BIT)
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
    else if (analogChannels==6 && dataFormat==ANALOG_FORMAT_16_BIT)
    {
        //calibrated astrain board
        if (useCalibration)
        {
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
                            if (attempts>0)    fprintf(stderr, "*** WARNING: Trying to get fullscale data from sensor: channel recovered (ch:%d)\n", ch);
                            break;
                        }
                    attempts++;
                    yarp::os::Time::delay(0.020);
                }
                if (attempts>=15)
                {
                    fprintf(stderr, "*** ERROR: Trying to get fullscale data from sensor: all attempts failed (ch:%d)\n", ch);
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
            msg.getData()[1]=0x00; 
            msg.setLen(2);
            canMessages=0;
            pCanBus->canWrite(outBuffer, 1, &canMessages);
        }
        //not calibrated strain board
        else
        {
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
    return sensorsNum;
}

int CanBusAnalogSensor::calibrateSensor()
{
    //NOT YET IMPLEMENTED
    return calibrateSensor();
}

int CanBusAnalogSensor::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return calibrateChannel(ch, 0);
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
            fprintf(stderr, "Warning, got unexpected class 0x3 msg(s): groupId 0x%x\n", groupId);
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
            fprintf(stderr, "Warning, got unexpected class 0x3 msg(s): groupId 0x%x\n", groupId);
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
        std::cerr<<"canRead failed\n";
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
    printf("CanBusAnalogSensor Thread releasing...\n");
    printf("... done.\n");
}

