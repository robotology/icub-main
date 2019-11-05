// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali, Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SKIN_MESH_THREAD_PORT_H__
#define __SKIN_MESH_THREAD_PORT_H__

#include <mutex>
#include <string>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>

#include "include/Quad16.h"
#include "include/PalmRight.h"
#include "include/PalmLeft.h"
#include "include/Triangle.h"
#include "include/Fingertip.h"
#include "include/Fingertip2Left.h"
#include "include/Fingertip2Right.h"
#include "include/Triangle_10pad.h"

using namespace yarp::os;
using namespace yarp::dev;

class SkinMeshThreadPort : public PeriodicThread 
{
protected:
    static const int MAX_SENSOR_NUM = 128;

    BufferedPort<Bottle> skin_port;                         
    TouchSensor *sensor[MAX_SENSOR_NUM];

    std::mutex mtx;

    int sensorsNum;
    bool mbSimpleDraw;

public:
    SkinMeshThreadPort(Searchable& config,int period) : PeriodicThread((double)period/1000.0)
    {
        mbSimpleDraw=config.check("light");

        sensorsNum=0;

        for (int t=0; t<MAX_SENSOR_NUM; ++t)
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
        
        Bottle *color = config.find("color").asList();
        unsigned char r=255, g=0, b=0;
        if(color)
        {
            if(color->size()<3 || !color->get(0).isInt() || !color->get(1).isInt() || !color->get(2).isInt())
            {
                printf("Error while reading the parameter color: three integer values should be specified (%s).\n", color->toString().c_str());
            }
            else
            {
                r = color->get(0).asInt();
                g = color->get(1).asInt();
                b = color->get(2).asInt();
                printf("Using specified color: %d %d %d\n", r, g, b);
            }
        }
        else
        {
            printf("Using red as default color.\n");
        }

        yarp::os::Bottle sensorSetConfig=config.findGroup("SENSORS").tail();

        for (int t=0; t<sensorSetConfig.size(); ++t)
        {       
            yarp::os::Bottle sensorConfig(sensorSetConfig.get(t).toString());

            std::string type(sensorConfig.get(0).asString());
          if (type=="triangle" || type=="fingertip" || type=="fingertip2L" || type=="fingertip2R" || type=="triangle_10pad" || type=="quad16" || type=="palmR" || type=="palmL")
             {
                int id=sensorConfig.get(1).asInt();
                double xc=sensorConfig.get(2).asDouble();
                double yc=sensorConfig.get(3).asDouble();
                double th=sensorConfig.get(4).asDouble();
                double gain=sensorConfig.get(5).asDouble();
                int    lrMirror=sensorConfig.get(6).asInt();
                int    layoutNum=sensorConfig.get(7).asInt();

                printf("%s %d %f\n",type.c_str(),id,gain);

                if (id>=0 && id<MAX_SENSOR_NUM)
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
                        if (type=="triangle_10pad")
                        {
                            sensor[id]=new Triangle_10pad(xc,yc,th,gain,layoutNum,lrMirror);
                            sensor[id]->setCalibrationFlag(useCalibration);
                        }
                        if (type=="fingertip")
                        {
                            sensor[id]=new Fingertip(xc,yc,th,gain,layoutNum,lrMirror);
                            sensor[id]->setCalibrationFlag(useCalibration);
                        }
                        if (type=="fingertip2L")
                        {
                            sensor[id]=new Fingertip2L(xc,yc,th,gain,layoutNum,lrMirror);
                            sensor[id]->setCalibrationFlag(useCalibration);
                        }
                        if (type=="fingertip2R")
                        {
                            sensor[id]=new Fingertip2R(xc,yc,th,gain,layoutNum,lrMirror);
                            sensor[id]->setCalibrationFlag(useCalibration);
                        }
                        if (type=="quad16")
                        {
                            printf("Quad16");
                            sensor[id]=new Quad16(xc,yc,th,gain,layoutNum,lrMirror);
                            sensor[id]->setCalibrationFlag(useCalibration);
                            }
                        if (type=="palmR")
                        {
                            sensor[id]=new PalmR(xc,yc,th,gain,layoutNum,lrMirror);
                            sensor[id]->setCalibrationFlag(useCalibration);
                        }
                        if (type=="palmL")
                        {
                            sensor[id]=new PalmL(xc,yc,th,gain,layoutNum,lrMirror);
                            sensor[id]->setCalibrationFlag(useCalibration);
                        }
                      
                        ++sensorsNum;
                    }
                }
                else
                {
                    printf("WARNING: %d is invalid triangle Id [0:%d].\n",id, MAX_SENSOR_NUM-1);
                }
            }
            else
            {
                printf("WARNING: sensor type %s unknown, discarded.\n",type.c_str());
            }
        }

        int max_tax=0;
        for (int t=0; t<MAX_SENSOR_NUM; ++t)
        {
            
            if (sensor[t]) 
            {
                sensor[t]->min_tax=max_tax;
                max_tax = sensor[t]->min_tax+sensor[t]->get_nTaxels();
                sensor[t]->max_tax=max_tax-1;
                sensor[t]->setColor(r, g, b);
            } 
            else
            {
                //this deals with the fact that some traingles can be not present,
                //but they anyway broadcast an array of zeros...
                max_tax += 12; 
            }
        }

        resize(width,height);
    }

    ~SkinMeshThreadPort()
    {
        for (int t=0; t<MAX_SENSOR_NUM; ++t)
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
        for (int t=0; t<MAX_SENSOR_NUM; ++t)
        {
            if (sensor[t]) sensor[t]->resize(width,height,40);
        }
    }

    void eval(unsigned char *image)
    {
        std::lock_guard<std::mutex> lck(mtx);
        for (int t=0; t<MAX_SENSOR_NUM; ++t)
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
        for (int t=0; t<MAX_SENSOR_NUM; ++t)
        {
            if (sensor[t]) sensor[t]->draw(image);
        }        
    }
};

#endif
