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
#include <vector>

#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
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
#include "include/CER_SH_PDL.h"

using namespace yarp::os;
using namespace yarp::dev;

class SkinMeshThreadPort : public PeriodicThread 
{
protected:
    static const int MAX_SENSOR_NUM = 128;

    BufferedPort<Bottle> skin_port;
    BufferedPort<Bottle> skin_port_virtual;
    TouchSensor *sensor[MAX_SENSOR_NUM];

    std::mutex mtx;

    int sensorsNum;
    bool mbSimpleDraw;
    
    std::vector<unsigned char> defaultColor;

    double skinThreshold;

public:
    SkinMeshThreadPort(Searchable& config,int period);

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
