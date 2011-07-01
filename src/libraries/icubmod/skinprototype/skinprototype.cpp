// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2010 RobotCub Consortium
// Authors: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <skinprototype.h>

#include <yarp/os/Time.h>
#include <iostream>
#include <string.h>

const int CAN_DRIVER_BUFFER_SIZE=2047;

#define SKIN_DEBUG 0

using namespace std;

bool SkinPrototype::open(yarp::os::Searchable& config)
{
    bool correct=true;
#if SKIN_DEBUG
    fprintf(stderr, "%s\n", config.toString().c_str());
#endif

    correct &= config.check("canbusdevice");
    correct &= config.check("CanDeviceNum");
    correct &= config.check("SkinCanIds");
    correct &= config.check("Period");

    if (!correct)
    {
        std::cerr<<"Error: insufficient parameters to SkinPrototypce\n"; 
        return false;
    }

    int period=config.find("Period").asInt();
    setRate(period);

    Bottle ids=config.findGroup("SkinCanIds").tail();

    if (ids.size()>1)
    {
		cerr<<"Warning: SkinPrototype id list contains more than one entry -> devices will be merged. "<<endl;
    }
	for (int i=0; i<ids.size(); i++)
	{
		int id = ids.get(i).asInt();
		cardId.push_back (id);
		#if SKIN_DEBUG
			fprintf(stderr, "Id reading from %d\n", id);
		#endif
	}

    Property prop;

    prop.put("device", config.find("canbusdevice").asString().c_str());
    prop.put("CanTxTimeout", 500);
    prop.put("CanRxTimeout", 500);
    prop.put("CanDeviceNum", config.find("CanDeviceNum").asInt());
    prop.put("CanMyAddress", 0);
    prop.put("CanTxQueueSize", CAN_DRIVER_BUFFER_SIZE);
    prop.put("CanRxQueueSize", CAN_DRIVER_BUFFER_SIZE);

    pCanBus=0;
    pCanBufferFactory=0;

    driver.open(prop);
    if (!driver.isValid())
    {
        fprintf(stderr, "Error opening PolyDriver check parameters\n");
        return false;
    }

    driver.view(pCanBus);
    
    if (!pCanBus)
    {
        fprintf(stderr, "Error opening /ecan device not available\n");
        return false;
    }

    driver.view(pCanBufferFactory);
    pCanBus->canSetBaudRate(0); //default 1MB/s

	for (int i=0; i<cardId.size(); i++)
		for (int id=0; id<16; ++id)
		{
			pCanBus->canIdAdd(0x300+(cardId[i]<<4)+id);
		}

    outBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);
    inBuffer=pCanBufferFactory->createBuffer(CAN_DRIVER_BUFFER_SIZE);

    //elements are:
    sensorsNum=16*12*cardId.size();
    data.resize(sensorsNum);

    RateThread::start();
    return true;
}

bool SkinPrototype::close()
{
    RateThread::stop();
    if (pCanBufferFactory) 
    {
        pCanBufferFactory->destroyBuffer(inBuffer);
        pCanBufferFactory->destroyBuffer(outBuffer);
    }
    driver.close();
    return true;
}

int SkinPrototype::read(yarp::sig::Vector &out) 
{
    mutex.wait();
    out=data;
    mutex.post();

    return yarp::dev::IAnalogSensor::AS_OK;
}

int SkinPrototype::getState(int ch)
{
    return yarp::dev::IAnalogSensor::AS_OK;;
}

int SkinPrototype::getChannels()
{
    return sensorsNum;
}

int SkinPrototype::calibrateSensor()
{
	for (int i=0; i<cardId.size(); i++)
	{
		#if SKIN_DEBUG
			printf("SkinPrototype:: calibrating boardId: %d\n",cardId[i]);
		#endif 

   		unsigned int canMessages=0;
		unsigned id = 0x200 + cardId[i];
	   
		CanMessage &msg=outBuffer[0];
		msg.setId(id);
		msg.getData()[0]=0x4C; // message type
		msg.getData()[1]=0x01; 
		msg.getData()[2]=0x01; 
		msg.getData()[3]=0x01;
		msg.getData()[4]=0;
		msg.getData()[5]=0x22;
		msg.getData()[6]=0;
		msg.getData()[7]=0;
		msg.setLen(8);
		canMessages=0;
		pCanBus->canWrite(outBuffer, 1, &canMessages);
	}

    return AS_OK;
}

int SkinPrototype::calibrateChannel(int ch, double v)
{
	//NOT YET IMPLEMENTED
	return calibrateSensor();
}

int SkinPrototype::calibrateSensor(const yarp::sig::Vector& v)
{
    return 0;
}

int SkinPrototype::calibrateChannel(int ch)
{
    return 0;
}


bool SkinPrototype::threadInit()
{
	for (int i=0; i<cardId.size(); i++)
	{
		#if SKIN_DEBUG
			printf("SkinPrototype:: thread initialising boardId:%d\n",cardId[i]);
		#endif 

   		unsigned int canMessages=0;
		unsigned id = 0x200 + cardId[i];
	   
		CanMessage &msg=outBuffer[0];
		msg.setId(id);
		msg.getData()[0]=0x4C; // message type
		msg.getData()[1]=0x01; 
		msg.getData()[2]=0x01; 
		msg.getData()[3]=0x01;
		msg.getData()[4]=0;
		msg.getData()[5]=0x22;
		msg.getData()[6]=0;
		msg.getData()[7]=0;
		msg.setLen(8);
		canMessages=0;
		pCanBus->canWrite(outBuffer, 1, &canMessages);
	}

    return true;
}

void SkinPrototype::run()
{	
    mutex.wait();

    unsigned int canMessages=0;

    bool res=pCanBus->canRead(inBuffer,CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
    {
        std::cerr<<"canRead failed\n";
    }

    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=inBuffer[i];

        unsigned int msgid=msg.getId();
        unsigned int id;
        unsigned int sensorId;
        id=(msgid & 0x00f0)>>4;
        sensorId=msgid&0x000f;

        unsigned int type=msg.getData()[0]&0x80;
        int len=msg.getLen();

		for (int i=0; i<cardId.size(); i++)
		{
			if (id==cardId[i])
            {
                int index=16*12*i + sensorId*12;
                
                if (type)
                    {
                        for(int k=0;k<5;k++)
                            data[index+k+7]=msg.getData()[k+1];
                    }
                else 
                    {
                        for(int k=0;k<7;k++)
                            data[index+k]=msg.getData()[k+1];
                    }
          //    else
          //        {
          //            std::cerr<<"Error: skin received malformed message\n";
          //        }
            }
		}
    }

    mutex.post();
}

void SkinPrototype::threadRelease()
{
#if SKIN_DEBUG
	printf("SkinPrototype Thread releasing...\n");	
    printf("... done.\n");
#endif
}

