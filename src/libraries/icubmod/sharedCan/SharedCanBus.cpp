// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2012 RobotCub Consortium
 * Author: Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <vector>

#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/os/RateThread.h>
#include <yarp/dev/PolyDriver.h>

#include "SharedCanBus.h"

const int CAN_DRIVER_BUFFER_SIZE = 500;

class SharedCanBus : public yarp::os::RateThread
{
public:
    SharedCanBus() : RateThread(10), writeMutex(1), configMutex(1)
    {
        mBufferSize=0;
        mCanDeviceNum=-1;
        mDevice="";

        reqIdsUnion=new char[0x800];

        for (int i=0; i<0x800; ++i) reqIdsUnion[i]=UNREQ;
    }

    ~SharedCanBus()
    {
        stop();

        polyDriver.close();

        delete [] reqIdsUnion;
    }

    int getBufferSize()
    {
        return mBufferSize;
    }

    bool IloveUmom(yarp::os::Searchable &config)
    {
        if (!config.check("physDevice"))
        {
            fprintf(stderr, "[ERROR] SharedCanBus::???() could not find low level can driver specification\n");         
            return false;
        }

        if (mDevice!=config.find("physDevice").asString()) return false;

        if (!config.check("canDeviceNum")) return true;

        return mCanDeviceNum==config.find("canDeviceNum").asInt();
    }

    void attachAccessPoint(yarp::dev::CanBusAccessPoint* ap)
    {
        configMutex.wait();
        accessPoints.push_back(ap);
        configMutex.post();
    }

    void detachAccessPoint(yarp::dev::CanBusAccessPoint* ap)
    {
        if (!ap) return;

        configMutex.wait();

        int n=accessPoints.size();

        for (int i=0; i<n; ++i)
        {
            if (ap==accessPoints[i])
            {
                for (int id=0; id<0x800; ++id)
                {
                    if (ap->hasId(id)) canIdDeleteUnsafe(id);
                }
                
                accessPoints[i]=accessPoints[n-1];
                
                accessPoints.pop_back();

                break;
            }
        }

        if (accessPoints.size()==0)
        {
            // should close the driver here?
        }

        configMutex.post();
    }

    void run()
    {
        static const bool NOWAIT=false;
        unsigned int msgsNum=0;

        configMutex.wait();

        bool ret=theCanBus->canRead(readBufferUnion,mBufferSize,&msgsNum,NOWAIT);

        if (ret)
        {
            for (unsigned int i=0; i<msgsNum; ++i)
            {
                unsigned int id=readBufferUnion[i].getId();

                for (unsigned int p=0; p<accessPoints.size(); ++p)
                {
                    if (accessPoints[p]->hasId(id))
                    {
                        accessPoints[p]->pushReadMsg(readBufferUnion[i]);    
                    }
                }
            }
        } 

        configMutex.post();
    }

    bool canWrite(const yarp::dev::CanBuffer &msgs, unsigned int size, unsigned int *sent, bool wait,yarp::dev::CanBusAccessPoint* pFrom)
    {
        writeMutex.wait();
        bool ret=theCanBus->canWrite(msgs,size,sent,wait);

        //this allows other istances to read back the sent message (echo)
        yarp::dev::CanBuffer buff=msgs;
        for (unsigned int m=0; m<size; ++m)
        {
            unsigned int id=buff[m].getId();
            if (reqIdsUnion[id])
            {
                for (unsigned int p=0; p<accessPoints.size(); ++p)
                {
                    if (accessPoints[p]!=pFrom && accessPoints[p]->hasId(id))
                    {
                        accessPoints[p]->pushReadMsg(buff[m]);
                    }
                }
            }
        }

        writeMutex.post();

        return ret;
    }

    void canIdAdd(unsigned int id)
    {
        configMutex.wait();

        if (reqIdsUnion[id]==UNREQ)
        {
            reqIdsUnion[id]=REQST;
            theCanBus->canIdAdd(id);
        }

        configMutex.post();
    }

    void canIdDelete(unsigned int id)
    {
        configMutex.wait();

        canIdDeleteUnsafe(id);

        configMutex.post();
    }
    
    yarp::dev::ICanBus* getCanBus()
    {
        return theCanBus;
    }

    yarp::dev::ICanBufferFactory* getCanBufferFactory()
    {
        if (!theBufferFactory)
        {
            fprintf(stderr, "[ERROR] no buffer factory\n");
        }

        return theBufferFactory;
    }

    yarp::dev::ICanBusErrors* getCanBusErrors()
    {
        return theCanBusErrors;
    }

    bool open(yarp::os::Searchable &config)
    {
        configMutex.wait();

        //fprintf(stderr, "sharedCanBus::open() using the following configuration parameters: \n%s\n",config.toString().c_str());
        if (!config.check("physDevice"))
        {
            fprintf(stderr, "[ERROR] SharedCanBus::open() could not find low level can driver specification\n");
            configMutex.post();         
            return false;
        }

        yarp::os::ConstString device=config.find("physDevice").asString();

        yarp::os::Property prop;
        prop.fromString(config.toString().c_str());

        prop.unput("device");
        prop.unput("subdevice");
        prop.unput("physDevice");

        prop.put("device",device.c_str());

        // low level driver
        polyDriver.open(prop);
    
        if (!polyDriver.isValid())
        {
            fprintf(stderr, "[ERROR] could not instantiate can device\n");
            configMutex.post();
            return false;
        }

        polyDriver.view(theCanBus);

        if (theCanBus==NULL)
        {
            fprintf(stderr, "[ERROR] could not get ICanBus interface\n");
            configMutex.post();
            return false;
        }

        polyDriver.view(theBufferFactory);

        if (theBufferFactory==NULL)
        {
            fprintf(stderr, "[ERROR] could not get ICanBufferFactory interface\n");
            configMutex.post();
            return false;
        }

        polyDriver.view(theCanBusErrors);

        mBufferSize=CAN_DRIVER_BUFFER_SIZE;

        if (config.check("canRxQueueSize"))
        {
            mBufferSize=config.find("canRxQueueSize").asInt();
        }

        readBufferUnion=theBufferFactory->createBuffer(mBufferSize);

        bool started=start();

        mDevice=device;

        if (config.check("canDeviceNum"))
        {
            mCanDeviceNum=config.find("canDeviceNum").asInt();
        }

        configMutex.post();

        return started;
    }

private:
    void canIdDeleteUnsafe(unsigned int id)
    {
        if (reqIdsUnion[id]==REQST)
        {
            for (int i=0; i<(int)accessPoints.size(); ++i)
            {
                if (accessPoints[i]->hasId(id))
                {
                    return;
                }
            }

            reqIdsUnion[id]=UNREQ;

            theCanBus->canIdDelete(id);
        }
    }

    int mBufferSize;

    yarp::os::Semaphore writeMutex;
    yarp::os::Semaphore configMutex;

    yarp::os::ConstString mDevice;
    int mCanDeviceNum;

    yarp::dev::PolyDriver polyDriver;

    yarp::dev::ICanBus           *theCanBus;
    yarp::dev::ICanBufferFactory *theBufferFactory;
    yarp::dev::ICanBusErrors     *theCanBusErrors;

    std::vector<yarp::dev::CanBusAccessPoint*> accessPoints;

    yarp::dev::CanBuffer readBufferUnion;

    char *reqIdsUnion; //[0x800];
};

class SharedCanBusManager // singleton
{
public:
    ~SharedCanBusManager()
    {
        for (unsigned int i=0; i<mDevices.size(); ++i)
        {
            if (mDevices[i]) delete mDevices[i];
        }

        mDevices.clear();
    }

    static SharedCanBusManager& getInstance()
    {
        static SharedCanBusManager instance;

        return instance;
    }

    SharedCanBus* open(yarp::os::Searchable &config)
    {
        for (unsigned int i=0; i<mDevices.size(); ++i)
        {
            if (mDevices[i]->IloveUmom(config))
            {
                return mDevices[i];
            }
        }

        SharedCanBus *scb=new SharedCanBus();

        if (!scb->open(config))
        {
            delete scb;
            return NULL;
        }

        mDevices.push_back(scb);

        return scb;
    }

private:
    SharedCanBusManager()
    {
        mDevices.clear();
    }

    std::vector<SharedCanBus*> mDevices;
};

//////////////////////////////
// CanBusAccessPoint methods
//////////////////////////////

bool yarp::dev::CanBusAccessPoint::open(yarp::os::Searchable& config)
{
    mSharedPhysDevice=SharedCanBusManager::getInstance().open(config);
    
    if (!mSharedPhysDevice) return false;

    mBufferSize=(unsigned int)(mSharedPhysDevice->getBufferSize());

    readBuffer=createBuffer(mBufferSize);

    mSharedPhysDevice->attachAccessPoint(this);

    return true;
}

bool yarp::dev::CanBusAccessPoint::close()
{
    if (!mSharedPhysDevice) return false;

    mSharedPhysDevice->detachAccessPoint(this);

    return true;
}

bool yarp::dev::CanBusAccessPoint::canWrite(const CanBuffer &msgs, unsigned int size, unsigned int *sent, bool wait)
{
    if (!mSharedPhysDevice) return false;

    return mSharedPhysDevice->canWrite(msgs,size,sent,wait,this);
}

bool yarp::dev::CanBusAccessPoint::canGetBaudRate(unsigned int *rate)
{
    if (!mSharedPhysDevice) return false;

    return mSharedPhysDevice->getCanBus()->canGetBaudRate(rate);
}

bool yarp::dev::CanBusAccessPoint::canIdAdd(unsigned int id)
{
    if (!mSharedPhysDevice) return false;

    if (id>=0x800)
    {
        fprintf(stderr, "[ERROR] Id=%d is out of 11 bit address range\n",id);

        return false;
    }

    reqIds[id]=REQST;

    mSharedPhysDevice->canIdAdd(id);

    return true;
}

bool yarp::dev::CanBusAccessPoint::canIdDelete(unsigned int id)
{
    if (!mSharedPhysDevice) return false;

    if (id>=0x800)
    {
        fprintf(stderr, "[ERROR] Id=%d is out of 11 bit address range\n",id);

        return false;
    }

    reqIds[id]=UNREQ;

    mSharedPhysDevice->canIdDelete(id);

    return true;
}

yarp::dev::CanBuffer yarp::dev::CanBusAccessPoint::createBuffer(int nmessage)
{
    yarp::dev::CanBuffer cb;
    
    if (mSharedPhysDevice) cb = mSharedPhysDevice->getCanBufferFactory()->createBuffer(nmessage);

    return cb;
}

void yarp::dev::CanBusAccessPoint::destroyBuffer(CanBuffer &msgs)
{
    if (!mSharedPhysDevice) return;

    yarp::dev::ICanBufferFactory* tmp = mSharedPhysDevice->getCanBufferFactory();
    
    if (tmp) tmp->destroyBuffer(msgs);
}
