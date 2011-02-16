// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali marco.maggiali@iit.it, Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

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

#include "include/Triangle.h"
#include "include/Fingertip.h"

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
        
        if (config.check("name"))
        {
            part=config.find("name").asString().c_str();
            part+="/";
        }

		part.append(config.find("robotPart").asString());
		part.append(":i");
		skin_port.open(part.c_str());
        int width =config.find("width" ).asInt();
        int height=config.find("height").asInt();
		bool useCalibration = config.check("useCalibration");
		if (useCalibration==true) 
		{
			printf("Using calibrated skin values (0-255)\n");
		}
		else
		{
			printf("Using raw skin values (255-0)\n");
		}

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
				int    layoutNum=sensorConfig.get(7).asInt();

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
							sensor[id]->setCalibrationFlag(useCalibration);
                        }
                        else
                        {
                            sensor[id]=new Fingertip(xc,yc,th,gain,layoutNum,lrMirror);
							sensor[id]->setCalibrationFlag(useCalibration);
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

    void eval(unsigned char *image)
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
        //mutex.wait();
        for (int t=0; t<16; ++t)
        {
            if (sensor[t]) sensor[t]->draw(image);
        }        
        //mutex.post();
    }
};

#endif
