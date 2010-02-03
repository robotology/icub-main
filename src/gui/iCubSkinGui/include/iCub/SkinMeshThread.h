// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SKIN_MESH_THREAD_H__
#define __SKIN_MESH_THREAD_H__

//#include <stdio.h>
//#include <string>
//#include <iostream>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include <iCub/Triangle.h>

using namespace yarp::os;
using namespace yarp::dev;

class SkinMeshThread : public RateThread 
{
protected:
	PolyDriver driver;
    ICanBus *pCanBus;
    ICanBufferFactory *pCanBufferFactory;
    CanBuffer canBuffer;
    
    Triangle *triangles[16];

    yarp::os::Semaphore mutex;

    int cardId;

public:
	SkinMeshThread(Searchable& config,int period=40) : RateThread(period),mutex(1)
    {
        for (int t=0; t<16; ++t)
        {
            triangles[t]=NULL;
        }

        Bottle bot;
        bot=config.findGroup("triangle0");
        triangles[0]=new Triangle(bot.get(1).asDouble(),bot.get(2).asDouble(),bot.get(3).asDouble());
        bot=config.findGroup("triangle1");
        triangles[1]=new Triangle(bot.get(1).asDouble(),bot.get(2).asDouble(),bot.get(3).asDouble());
        bot=config.findGroup("triangle2");
        triangles[2]=new Triangle(bot.get(1).asDouble(),bot.get(2).asDouble(),bot.get(3).asDouble());
        bot=config.findGroup("triangle3");
        triangles[3]=new Triangle(bot.get(1).asDouble(),bot.get(2).asDouble(),bot.get(3).asDouble());
        bot=config.findGroup("triangle4");
        triangles[4]=new Triangle(bot.get(1).asDouble(),bot.get(2).asDouble(),bot.get(3).asDouble());
        bot=config.findGroup("triangle5");
        triangles[5]=new Triangle(bot.get(1).asDouble(),bot.get(2).asDouble(),bot.get(3).asDouble());

        int width =config.find("width" ).asInt();
        int height=config.find("height").asInt();

        resize(width,height);

        cardId=0x300 | (config.find("cardid").asInt() << 4);

    }

    ~SkinMeshThread()
    {
        for (int t=0; t<6; ++t)
        {
            if (triangles[t]) delete triangles[t];
        }
    }

	virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    void resize(int width,int height)
    {
        mutex.wait();

        for (int t=0; t<6; ++t)
        {
            triangles[t]->resize(width,height,64);
        }

        mutex.post();
    }

    void eval(double *image)
    {
        mutex.wait();

        for (int t=0; t<6; ++t)
        {
            triangles[t]->eval(image);
        }

        mutex.post();
    }

    void draw(unsigned char *image)
    {
        for (int t=0; t<6; ++t)
        {
            triangles[t]->draw(image);
        }        
    }
};

#endif
