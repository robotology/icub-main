// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2013 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <CanBusDoubleFTSensor.h>

#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

#include <yarp/os/Log.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

using namespace std;

bool CanBusDoubleFTSensor::open(yarp::os::Searchable& config)
{
    bool correct=true;

    //debug
    fprintf(stderr, "%s\n", config.toString().c_str());

    correct &= config.check("canbusDevice");
    correct &= config.check("canDeviceNum");
    correct &= config.check("canAddressBackSensor");
    correct &= config.check("canAddressFrontSensor");
    correct &= config.check("period");
    correct &= config.check("physDevice");
    correct &= config.check("backFrontDistance");
    correct &= config.find("backFrontDistance").isDouble();
    correct &= config.check("backSingleDistance");
    correct &= config.find("backSingleDistance").isDouble();

    if (!correct)
    {
        std::cerr<<"[ERROR] insufficient parameters to CanBusDoubleFTSensor\n";
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
        fprintf(stderr, "[ERROR] Error opening PolyDriver check parameters\n");
        return false;
    }
    driver.view(pCanBus);
    if (!pCanBus)
    {
        fprintf(stderr, "[ERROR] Error opening can device not available\n");
        return false;
    }
    driver.view(pCanBufferFactory);
    outBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);

    //select the communication speed
    pCanBus->canSetBaudRate(0); //default 1MB/s

    //set the internal configuration
    //this->isVirtualSensor   = false;
    this->sensorBoardId[FRONT_SENSOR]  = config.find("canAddressBackSensor").asInt();
    this->sensorBoardId[BACK_SENSOR]   = config.find("canAddressFrontSensor").asInt();
    //this->SensorFullScale = config.find("fullScale").asInt();
    //ANALOG_DATA_FORMAT is 16 Bit for STRAIN Boards
    unsigned int tmpFormat  = 16;
    if      (tmpFormat == 8)
        this->dataFormat = ANALOG_FORMAT_8_BIT;
    else if (tmpFormat == 16)
        this->dataFormat = ANALOG_FORMAT_16_BIT;
    else
        this->dataFormat = ANALOG_FORMAT_ERR;

    //Load geometric information
    this->backSingleDistance = config.find("backSingleDistance").asDouble();
    this->frontSingleDistance = this->backSingleDistance - config.find("backFrontDistance").asDouble();

    //open the can mask for the specific canDeviceId
    this->openCanMask(0);
    this->openCanMask(1);


    //create the raw data vectors:
    this->sensorsChannelsNum    =  6;
    sensorData[0].resize(this->sensorsChannelsNum);
    sensorData[0].zero();
    sensorData[1].resize(this->sensorsChannelsNum);
    sensorData[1].zero();
    scaleFactor[0].resize(this->sensorsChannelsNum);
    scaleFactor[0].zero();
    scaleFactor[1].resize(this->sensorsChannelsNum);
    scaleFactor[1].zero();
    //create the output data vectors

    //As we are using calibration, the output is the sum of the
    //two wrenches passing throught the two sensors
    this->outputChannelsNum = 6;

    this->outputData.resize(this->outputChannelsNum,0.0);
    this->buffer.resize(this->outputChannelsNum,0.0);

    //start the sensor broadcast
    sensor_start(config,0);
    sensor_start(config,1);

    RateThread::start();
    return true;
}

void CanBusDoubleFTSensor::openCanMask(int sensor_number)
{
    unsigned short boardId = this->sensorBoardId[sensor_number];
    for (int id=0; id<16; ++id)
    {
        pCanBus->canIdAdd(0x300+(boardId<<4)+id);
    }
    pCanBus->canIdAdd(0x200+boardId);
    pCanBus->canIdAdd(0x200+(boardId<<4));
}

bool CanBusDoubleFTSensor::readFullScaleAnalog(int ch, int sensor_number)
{
    scaleFactor[sensor_number][ch]=1e-20;

    unsigned int canMessages=0;
    unsigned short boardId = this->sensorBoardId[sensor_number];
    unsigned id = 0x200 + this->sensorBoardId[sensor_number];
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
                        scaleFactor[sensor_number][ch]=m.getData()[2]<<8 | m.getData()[3];
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
            fprintf(stderr, "[ERROR] Trying to get fullscale data from sensor %d net [%d]: no answer received or message lost (ch:%d)\n", boardId, canDeviceNum, ch);
            return false;
        }

    return true;
}

bool CanBusDoubleFTSensor::sensor_start(yarp::os::Searchable& analogConfig, int sensor_number)
{
    fprintf(stderr, "--> Initializing analog device %s\n", analogConfig.find("deviceId").toString().c_str());

    unsigned int canMessages=0;
    unsigned id = 0x200 + this->sensorBoardId[sensor_number];

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
        fprintf(stderr, "using broadcast period %d on device %s\n", period, analogConfig.find("deviceId").toString().c_str());
    }

    //init message for strain board (checking just for safety)
    if (this->sensorsChannelsNum==6 && this->dataFormat==ANALOG_FORMAT_16_BIT)
    {
        {
            fprintf(stderr, "using internal calibration on device %s\n", analogConfig.find("deviceId").toString().c_str());
            //get the full scale values from the strain board
            for (int ch=0; ch<6; ch++)
            {
                bool b=false;
                int attempts = 0;
                while(attempts<15)
                {
                    b = readFullScaleAnalog(ch,sensor_number);
                    if (b==true)
                        {
                            if (attempts>0)    fprintf(stderr, "[WARNING] Trying to get fullscale data from sensor: channel recovered (ch:%d)\n", ch);
                            break;
                        }
                    attempts++;
                    yarp::os::Time::delay(0.020);
                }
                if (attempts>=15)
                {
                    fprintf(stderr, "[ERROR] Trying to get fullscale data from sensor: all attempts failed (ch:%d)\n", ch);
                }
            }

            // debug messages
            #if 1
                    fprintf(stderr, "Sensor Fullscale Id %#4X: Sensor Number %d",this->sensorBoardId[sensor_number],sensor_number);
                    fprintf(stderr, " %f ", scaleFactor[sensor_number][0]);
                    fprintf(stderr, " %f ", scaleFactor[sensor_number][1]);
                    fprintf(stderr, " %f ", scaleFactor[sensor_number][2]);
                    fprintf(stderr, " %f ", scaleFactor[sensor_number][3]);
                    fprintf(stderr, " %f ", scaleFactor[sensor_number][4]);
                    fprintf(stderr, " %f ", scaleFactor[sensor_number][5]);
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
    }
    return true;
}

bool CanBusDoubleFTSensor::sensor_stop(int sensor_number)
{
    unsigned int canMessages=0;
    unsigned id = 0x200 + this->sensorBoardId[sensor_number];
    CanMessage &msg=outBuffer[0];
    msg.setId(id);
    msg.getData()[0]=0x07;
    msg.getData()[1]=0x01;
    msg.setLen(2);
    canMessages=0;
    pCanBus->canWrite(outBuffer, 1, &canMessages);
    return true;
}

bool CanBusDoubleFTSensor::close()
{
    //stop the thread
    RateThread::stop();

    //stop the sensors
    sensor_stop(0);
    sensor_stop(1);

    //stop the driver
    if (pCanBufferFactory)
    {
        pCanBufferFactory->destroyBuffer(inBuffer);
        pCanBufferFactory->destroyBuffer(outBuffer);
    }
    driver.close();

    return true;
}

int CanBusDoubleFTSensor::read(yarp::sig::Vector &out)
{
    int ret_val;

    if( overallStatus == yarp::dev::IAnalogSensor::AS_OK)
    {
        mutex.wait();
        out=this->outputData;
        mutex.post();
    }

    return this->overallStatus;
}

int CanBusDoubleFTSensor::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;
}

int CanBusDoubleFTSensor::getChannels()
{
    return this->outputChannelsNum;
}

int CanBusDoubleFTSensor::calibrateSensor()
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusDoubleFTSensor::calibrateChannel(int ch, double v)
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusDoubleFTSensor::calibrateSensor(const yarp::sig::Vector& v)
{
    //NOT YET IMPLEMENTED
    return 0;
}

int CanBusDoubleFTSensor::calibrateChannel(int ch)
{
    //NOT YET IMPLEMENTED
    return 0;
}

bool CanBusDoubleFTSensor::threadInit()
{
    return true;
}

bool CanBusDoubleFTSensor::decode16(const unsigned char *msg, int msg_id, int sensor_number)
{
    const char groupId=(msg_id & 0x00f);
    if( sensor_number != 0 &&
        sensor_number != 1)
    {
        return false;
    }
    double * data = this->sensorData[sensor_number].data();
    int baseIndex=0;
    {
        switch (groupId)
        {
        case 0xA:
            {
                for(int k=0;k<3;k++)
                    {
                        data[k]=(((unsigned short)(msg[2*k+1]))<<8)+msg[2*k]-0x8000;
                        data[k]=data[k]*scaleFactor[sensor_number][k]/float(0x8000);
                    }
            }
            break;
        case 0xB:
            {
                for(int k=0;k<3;k++)
                    {
                        data[k+3]=(((unsigned short)(msg[2*k+1]))<<8)+msg[2*k]-0x8000;
                        data[k+3]=data[k+3]*scaleFactor[sensor_number][k+3]/float(0x8000);
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


void CanBusDoubleFTSensor::run()
{
    mutex.wait();

    unsigned int canMessages=0;
    bool ret=true; //return true by default

    bool res=pCanBus->canRead(inBuffer,CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
    {
        std::cerr<<"[ERROR] CanBusDoubleFTSensor canRead failed\n";
    }

    double timeNow=Time::now();
    bool updateOutput = false;
    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=inBuffer[i];

        unsigned int msgid    = msg.getId();
        unsigned char *buff   = msg.getData();
        unsigned int len      = msg.getLen();
        unsigned int id       = (msgid & 0x00f0)>>4;
        const char   type     = ((msgid&0x700)>>8);

        //parse data here
        overallStatus=IAnalogSensor::AS_OK;

        if (type==0x03) //analog data
        {
            int sensor_number = -1;
            if( id==this->sensorBoardId[0] )
            {
                sensor_number = 0;
            }
            if( id==this->sensorBoardId[1] )
            {
                sensor_number = 1;
            }
            if ( sensor_number == 0 ||
                 sensor_number == 1 )
            {
                updateOutput = true;
                timeStamp[sensor_number]=Time::now();
                switch (dataFormat)
                {
                    case ANALOG_FORMAT_16_BIT:
                        if (len==6)
                        {
                            ret=decode16(buff, msgid, sensor_number);
                            status[sensor_number]=IAnalogSensor::AS_OK;
                        }
                        else
                        {
                            if (len==7 && buff[6] == 1)
                            {
                                status[sensor_number]=IAnalogSensor::AS_OVF;
                            }
                            else
                            {
                                status[sensor_number]=IAnalogSensor::AS_ERROR;
                            }
                            ret=decode16(buff, msgid, sensor_number);
                        }
                        break;
                    default:
                        status[sensor_number]=IAnalogSensor::AS_ERROR;
                        ret=false;
                }
            }
        }
    }

    //if 100ms have passed since the last received message
    for(int sensor_number=0; sensor_number < 2; sensor_number++ )
    {
        if (timeStamp[sensor_number]+0.1<timeNow)
        {
            updateOutput = true;
            status[sensor_number]=IAnalogSensor::AS_TIMEOUT;
        }
    }

    if( updateOutput )
    {
        //If the sensor reading have been update, update also the output
        combineDoubleSensorStatus();
        combineDoubleSensorReadings();
    }

    mutex.post();
}

void CanBusDoubleFTSensor::combineDoubleSensorReadings()
{
    //First, rotate the front sensor measurement in the same
    // orientation of the back and emulated sensors
    // Buffer will hold the rotate front version reading
    buffer[0] = -sensorData[FRONT_SENSOR][0];
    buffer[1] = -sensorData[FRONT_SENSOR][1];
    buffer[2] =  sensorData[FRONT_SENSOR][2];
    buffer[3] = -sensorData[FRONT_SENSOR][3];
    buffer[4] = -sensorData[FRONT_SENSOR][4];
    buffer[5] =  sensorData[FRONT_SENSOR][5];

    //Then translate the FT measure from the
    //back sensor to the emulated single sensor
    //Here we are basically using the adjoint matrix
    outputData[0] = sensorData[BACK_SENSOR][0];
    outputData[1] = sensorData[BACK_SENSOR][1];
    outputData[2] = sensorData[BACK_SENSOR][2];
    outputData[3] = sensorData[BACK_SENSOR][3];
    outputData[4] = sensorData[BACK_SENSOR][4] + backSingleDistance*sensorData[BACK_SENSOR][2];
    outputData[5] = sensorData[BACK_SENSOR][5] - backSingleDistance*sensorData[BACK_SENSOR][1];

    //Then we translate also the FT measure from
    //front sensor to the emulate single sensor
    //summing them to one of the back sensor
    outputData[0] += buffer[0];
    outputData[1] += buffer[1];
    outputData[2] += buffer[2];
    outputData[3] += buffer[3];
    outputData[4] += buffer[4] + frontSingleDistance*buffer[2];
    outputData[5] += buffer[5] - frontSingleDistance*buffer[1];
}

void CanBusDoubleFTSensor::combineDoubleSensorStatus()
{
    //If one of the sensor has an error, report an error
    if( status[0] == IAnalogSensor::AS_ERROR ||
        status[1] == IAnalogSensor::AS_ERROR )
    {
        overallStatus = IAnalogSensor::AS_ERROR;
    }
    else if( status[0] == IAnalogSensor::AS_TIMEOUT ||
             status[1] == IAnalogSensor::AS_TIMEOUT )
    {
        overallStatus = IAnalogSensor::AS_TIMEOUT;
    }
    else if( status[0] == IAnalogSensor::AS_OVF ||
             status[1] == IAnalogSensor::AS_OVF )
    {
        overallStatus = IAnalogSensor::AS_OVF;
    }
    else if( status[0] == IAnalogSensor::AS_OK &&
             status[1] == IAnalogSensor::AS_OK )
    {
        overallStatus = IAnalogSensor::AS_OK;
    }
    else
    {
        //something went wrong because the status of the
        //individual sensor have a unexpected value, returning
        //an error
        yError("CanBusDoubleFTSensor sensor status have unexpected values %d,%d\n",status[0],status[1]);
        overallStatus = IAnalogSensor::AS_ERROR;
    }
    return;
}



void CanBusDoubleFTSensor::threadRelease()
{
    printf("CanBusDoubleFTSensor Thread releasing...\n");
    printf("... done.\n");
}

