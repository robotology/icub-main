// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __SKIN_MESH_THREAD_PORT_H__
#define __SKIN_MESH_THREAD_PORT_H__

//#include <stdio.h>
#include <string>

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>

#include <iCub/Triangle.h>
#include <iCub/Fingertip.h>

using namespace yarp::os;
using namespace yarp::dev;

class SkinMeshThreadPort : public RateThread 
{
protected:
	BufferedPort<Bottle> skin_port;							
	TouchSensor *sensor[16];

    yarp::os::Semaphore mutex;

    int sensorsNum;

public:
	SkinMeshThreadPort(Searchable& config,int period=40) : RateThread(period),mutex(1)
    {
        sensorsNum=0;

        for (int t=0; t<16; ++t)
        {
            sensor[t]=NULL;
        }
	
		std::string part="/skinGui/";
		part.append(config.find("robotPart").asString());
		part.append(":i");
		skin_port.open(part.c_str());
		int layoutNum=config.find("LayoutNum").asInt();
        int width =config.find("width" ).asInt();
        int height=config.find("height").asInt();

        yarp::os::Bottle sensorSetConfig=config.findGroup("SENSORS").tail();

        for (int t=0; t<sensorSetConfig.size(); ++t)
        {       
            yarp::os::Bottle sensorConfig(sensorSetConfig.get(t).toString());

            std::string type(sensorConfig.get(0).asString());
            if (type=="triangle" || type=="fingertip")
            {
                int id=sensorConfig.get(1).asInt();
                double xc=sensorConfig.get(2).asDouble();
                double yc=sensorConfig.get(3).asDouble();
                double th=sensorConfig.get(4).asDouble();
                double gain=sensorConfig.get(5).asDouble();
				int    lrMirror=sensorConfig.get(6).asInt();

                printf("%d %f\n",id,gain);

                if (id>=0 && id<16)
                {
                    if (sensor[id])
                    {
                        printf("WARNING: triangle %d already exists.\n",id);
                    }
                    else
                    {
                        if (type=="triangle")
                        {
                            sensor[id]=new Triangle(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        else
                        {
                            sensor[id]=new Fingertip(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        ++sensorsNum;
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

    ~SkinMeshThreadPort()
    {
        for (int t=0; t<16; ++t)
        {
            if (sensor[t]) delete sensor[t];
        }
    }

	virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

    void resize(int width,int height)
    {
        mutex.wait();

        for (int t=0; t<16; ++t)
        {
            if (sensor[t]) sensor[t]->resize(width,height,40);
        }

        mutex.post();
    }

    void eval(double *image)
    {
        mutex.wait();

        for (int t=0; t<16; ++t)
        {
            if (sensor[t]) sensor[t]->eval(image);
        }

        mutex.post();
    }

    void draw(unsigned char *image)
    {
        for (int t=0; t<16; ++t)
        {
            if (sensor[t]) sensor[t]->draw(image);
        }        
    }
};

#endif
