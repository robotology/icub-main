// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Maggiali, Alessandro Scalzo, Jacopo Losi
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef __SKIN_MESH_CLIENT_H__
#define __SKIN_MESH_CLIENT_H__

#include <mutex>
#include <string>
#include <vector>

#include <yarp/os/Log.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/PolyDriverList.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/RFModule.h>
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IMultipleWrapper.h>

#include "include/Quad16.h"
#include "include/PalmRight.h"
#include "include/PalmLeft.h"
#include "include/fakePalm.h"
#include "include/Triangle.h"
#include "include/Fingertip.h"
#include "include/Fingertip2Left.h"
#include "include/Fingertip2Right.h"
#include "include/Fingertip3Left.h"
#include "include/Fingertip3Right.h"
#include "include/Fingertip4Left.h"
#include "include/Fingertip4Right.h"
#include "include/FingertipMID.h"
#include "include/Triangle_10pad.h"
#include "include/CER_SH_PDL.h"

using namespace yarp::os;
using namespace yarp::dev;

class SkinMeshClient : public yarp::os::RFModule
{
protected:
    static const int MAX_SENSOR_NUM = 128;

    BufferedPort<Bottle> skin_port;
    BufferedPort<Bottle> skin_port_virtual;
    TouchSensor *sensor[MAX_SENSOR_NUM];

    std::mutex mtx;

    int sensorsNum;
    bool mbSimpleDraw;
    bool m_configured{false};
    double m_period{0.05};
    bool m_externalConnection{true};
    mutable double m_lastNoSkinPatchWarningTime{0.0};

    std::vector<unsigned char> defaultColor;

    double skinThreshold;

private:

    yarp::os::Property m_config;
    yarp::dev::PolyDriver _multipleAnalogSensorsClientDevice;
    yarp::dev::PolyDriver _multipleAnalogSensorsRemapperDevice;
    struct
    {
        yarp::dev::ISkinPatches* iskinPatchesSensors{nullptr};
        yarp::dev::IMultipleWrapper* imultwrap{nullptr};
    } remappedMASInterfaces;

    bool configureFromSearchable(yarp::os::Searchable& config);
    bool configureSkinClient(yarp::os::Searchable& config, const std::string& localPrefix);
    bool readSkinPatches(yarp::sig::Vector& skinValue) const;

public:
    SkinMeshClient(Searchable& config,int period);

    ~SkinMeshClient()
    {
        for (int t=0; t<MAX_SENSOR_NUM; ++t)
        {
            if (sensor[t]) delete sensor[t];
        }
    }

    bool start();
    void stop();

    virtual bool configure(yarp::os::ResourceFinder &rf) override;
    virtual bool close() override;
    virtual double getPeriod() override;
    virtual bool updateModule() override;

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
