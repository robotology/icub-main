// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Alessandro Scalzo alessandro.scalzo@iit.it
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SKIN_MESH_THREAD_CAN_H__
#define __SKIN_MESH_THREAD_CAN_H__

#include <mutex>
#include <string>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>

#include "include/Quad16.h"
#include "include/PalmLeft.h"
#include "include/PalmRight.h"
#include "include/Triangle.h"
#include "include/Triangle_10pad.h"
#include "include/Fingertip.h"
#include "include/Fingertip2Left.h"
#include "include/Fingertip2Right.h"
#include "include/CER_SH_PDL.h"

using namespace yarp::os;
using namespace yarp::dev;

class SkinMeshThreadCan : public PeriodicThread 
{
protected:
    PolyDriver driver;
    ICanBus *pCanBus;
    ICanBufferFactory *pCanBufferFactory;
    CanBuffer canBuffer;
    std::string deviceName;
    int netId;
    
    TouchSensor *sensor[16];

    std::mutex mtx;

    int cardId;
    int sensorsNum;
    bool mbSimpleDraw;

public:
    SkinMeshThreadCan(Searchable& config,int period) : PeriodicThread((double)period/1000.0)
    {
        mbSimpleDraw=config.check("light");

        sensorsNum=0;

        for (int t=0; t<16; ++t)
        {
            sensor[t]=NULL;
        }
        netId=7;
        deviceName="cfw2can";
        cardId=0x300 | (config.find("cardid").asInt() << 4);
        netId=config.find("CanDeviceNum").asInt();
        deviceName=config.find("CanDeviceName").asString();
        std::string part="/skinGui/";
        part.append(config.find("robotPart").asString());
        part.append(":i");
        int width =config.find("width" ).asInt();
        int height=config.find("height").asInt();
        bool useCalibration = config.check("useCalibration");
        if (useCalibration==true) 
        {
            printf("Reading datadirectly from CAN! Calibration unsupported\n");
            useCalibration = false;
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
            if (type == "triangle" || type == "fingertip" || type == "fingertip2L" || type == "cer_sh_pdl"  || type == "fingertip2R" || type == "triangle_10pad" || type == "quad16" || type == "palmR" || type == "palmL")
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
                        }
                        if (type=="triangle_10pad")
                        {
                            sensor[id]=new Triangle_10pad(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        if (type=="fingertip")
                        {
                            sensor[id]=new Fingertip(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        if (type=="fingertip2L")
                        {
                            sensor[id]=new Fingertip2L(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        if (type == "cer_sh_pdl")
                        {
                            sensor[id] = new CER_SH_PDL(xc, yc, th, gain, layoutNum, lrMirror);
                        }
                        if (type=="fingertip2R")
                        {
                            sensor[id]=new Fingertip2R(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        if (type=="quad16")
                        {
                            sensor[id]=new Quad16(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        if (type=="palmR")
                        {
                            sensor[id]=new PalmR(xc,yc,th,gain,layoutNum,lrMirror);
                        }
                        if (type=="palmL")
                        {
                            sensor[id]=new PalmL(xc,yc,th,gain,layoutNum,lrMirror);
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
                printf("WARNING: sensor of type %s unknown, discarded.\n",type.c_str());
            }
        }

        resize(width,height);
    }

    ~SkinMeshThreadCan()
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
        std::lock_guard<std::mutex> lck(mtx);
        for (int t=0; t<16; ++t)
        {
            if (sensor[t]) sensor[t]->resize(width,height,40);
        }
    }

    void eval(unsigned char *image)
    {
        std::lock_guard<std::mutex> lck(mtx);
        for (int t=0; t<16; ++t)
        {
            if (sensor[t])
            {
                if (mbSimpleDraw)
                {
                    sensor[t]->eval_light(image);
                }
                else
                {
                    sensor[t]->eval(image);
                }
            }
        }
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
