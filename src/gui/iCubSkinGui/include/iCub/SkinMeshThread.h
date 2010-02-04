// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SKIN_MESH_THREAD_H__
#define __SKIN_MESH_THREAD_H__

//#include <stdio.h>
#include <string>

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

        cardId=0x300 | (config.find("cardid").asInt() << 4);
        int width =config.find("width" ).asInt();
        int height=config.find("height").asInt();

        yarp::os::Bottle sensorSet=config.findGroup("SENSORS").tail();

        for (int t=0; t<sensorSet.size(); ++t)
        {       
            yarp::os::Bottle sensor(sensorSet.get(t).toString());

            std::string type(sensor.get(0).asString());
            if (type=="triangle")
            {
                int id=sensor.get(1).asInt();
                double xc=sensor.get(2).asDouble();
                double yc=sensor.get(3).asDouble();
                double th=sensor.get(4).asDouble();

                if (id>=0 && id<16)
                {
                    if (triangles[id])
                    {
                        printf("WARNING: triangle %d already exists.\n",id);
                    }
                    else
                    {
                        triangles[id]=new Triangle(xc,yc,th);
                    }
                }
                else
                {
                    printf("WARNING: %d is invalid triangle Id [0:15].\n",id);
                }
            }
            else
            {
                printf("WARNING: sensor type %s unknown, discarded.\n",type.c_str());
            }
        }

        resize(width,height);
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
            triangles[t]->resize(width,height,40);
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
