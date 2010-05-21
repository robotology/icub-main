// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <skinprototype.h>

#include <yarp/os/Time.h>
#include <iostream>

const int CAN_DRIVER_BUFFER_SIZE=2047;

#define DEBUG 1

using namespace std;

bool SkinPrototype::open(yarp::os::Searchable& config)
{
    bool correct=true;
    correct &= config.check("canbusdevice");
    correct &= config.check("CanDeviceNum");
    correct &= config.check("skinCanIds");

    if (!correct)
    {
        std::cerr<<"Error: insufficient parameters to SkinPrototype\n"; 
        return false;
    }

    Bottle ids=config.findGroup("skinCanIds").tail();

    if (ids.size()>1)
    {
        cerr<<"Error: SkinPrototype id list contains more than one entry, this at the moment is unsupported"<<endl;
        return false;
    }
    cardId=ids.get(0).asInt();

#if DEBUG
    fprintf(stderr, "Id reading from %d\n", cardId);
#endif 

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

    canBuffer=pCanBufferFactory->createBuffer(4*sensorsNum);

    //elements are:
    sensorsNum=16*12;
    data.resize(sensorsNum);

    RateThread::run();
    return true;
}

bool SkinPrototype::close()
{
    RateThread::stop();
    if (pCanBufferFactory) 
    {
        pCanBufferFactory->destroyBuffer(canBuffer);
    }
    driver.close();
    return true;
}

int SkinPrototype::read(yarp::sig::Vector &out) 
{
    mutex.wait();
    out=data;
    mutex.post();

    return ANALOG_OK;
}

int SkinPrototype::getState(int ch)
{
    return ANALOG_OK;
}

int SkinPrototype::getChannels()
{
    return sensorsNum;
}

bool SkinPrototype::calibrate(int ch, double v)
{
    return true;
}


bool SkinPrototype::threadInit()
{
#if DEBUG
	printf("Image Thread initialising...\n");
    printf("... done!\n");
#endif 
    return true;
}

void SkinPrototype::run()
{	
    mutex.wait();

    unsigned int canMessages=0;

    bool res=pCanBus->canRead(canBuffer,CAN_DRIVER_BUFFER_SIZE,&canMessages);
    if (!res)
    {
        std::cerr<<"canRead failed\n";
    }

    for (unsigned int i=0; i<canMessages; i++)
    {
        CanMessage &msg=canBuffer[i];

        if ((msg.getId() & 0xFFFFFFF0) == cardId)
        {
            int sensorId=msg.getId() & 0x0F;

             if (msg.getData()[0] & 0x80)
             {
                 // last 5 bytes
                 for(int k=0;k<5;k++)
                 {
                    data(sensorId+k)=msg.getData()[k];
                 }
             }
             else
             {
                // first 7 bytes
                 for(int k=0;k<7;k++)
                 {
                     data(sensorId+k)=msg.getData()[k];
                 }
             }
        }
    }

    mutex.post();
}

void SkinPrototype::threadRelease()
{
#if DEBUG
	printf("Skin Mesh Thread releasing...\n");	
    printf("... done.\n");
#endif
}

