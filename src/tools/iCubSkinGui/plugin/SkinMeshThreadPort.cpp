// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Randazzo, Marco Maggiali, Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include "include/SkinMeshThreadPort.h"

#include <yarp/os/Time.h>

#define SKIN_THRESHOLD 15.0

SkinMeshThreadPort::SkinMeshThreadPort(Searchable& config,int period) : PeriodicThread((double)period/1000.0)
{
    yDebug("SkinMeshThreadPort running at %d ms.",(int)(1000.0*getPeriod()));
    mbSimpleDraw=config.check("light");

    sensorsNum=0;
    for (int t=0; t<MAX_SENSOR_NUM; ++t)
    {
        sensor[t]=NULL;
    }

    std::string part="/skinGui/";
    std::string part_virtual="";
    if (config.check("name"))
    {
        part ="/";
        part+=config.find("name").asString();
        part+="/";
    }
    part.append(config.find("robotPart").asString());
    part_virtual = part;
    part_virtual.append("_virtual");
    part.append(":i");
    part_virtual.append(":i");

    skin_port.open(part);

    // Ideally, we would use a --virtual flag. since this would make the skinmanager xml file unflexible,
    // let's keep the code structure without incurring in any problem whatsoever
    // if (config.check("virtual"))
    if (true)
    {
        skin_port_virtual.open(part_virtual);
    }

    int width =config.find("width" ).asInt();
    int height=config.find("height").asInt();

    bool useCalibration = config.check("useCalibration");
    if (useCalibration==true)   yInfo("Using calibrated skin values (0-255)");
    else                        yDebug("Using raw skin values (255-0)");

    Bottle *color = config.find("color").asList();
    unsigned char r=255, g=0, b=0;
    if(color)
    {
        if(color->size()<3 || !color->get(0).isInt() || !color->get(1).isInt() || !color->get(2).isInt())
        {
            yError("Error while reading the parameter color: three integer values should be specified (%s).", color->toString().c_str());
        }
        else
        {
            r = color->get(0).asInt();
            g = color->get(1).asInt();
            b = color->get(2).asInt();
            yInfo("Using specified color: %d %d %d", r, g, b);
        }
    }
    else
    {
        yDebug("Using red as default color.");
    }
    defaultColor.push_back(r);
    defaultColor.push_back(g);
    defaultColor.push_back(b);

    skinThreshold = config.check("skinThreshold")?config.find("skinThreshold").asDouble():SKIN_THRESHOLD;
    yDebug("Skin Threshold set to %g", skinThreshold);

    yarp::os::Bottle sensorSetConfig=config.findGroup("SENSORS").tail();

    for (int t=0; t<sensorSetConfig.size(); ++t)
    {
        yarp::os::Bottle sensorConfig(sensorSetConfig.get(t).toString());

        std::string type(sensorConfig.get(0).asString());

        if (type=="triangle"       ||
            type=="fingertip"      ||
            type=="fingertip2L"    ||
            type=="fingertip2R"    ||
            type=="triangle_10pad" ||
            type=="quad16"         ||
            type=="palmR"          ||
            type=="palmL"          ||
            type == "cer_sh_pdl"   ||
            type == "cer_sh_pdr"   ||
            type == "cer_sh_pp"    ||
            type == "cer_sh_td"    ||
            type == "cer_sh_tp")
        {
            int    id=sensorConfig.get(1).asInt();
            double xc=sensorConfig.get(2).asDouble();
            double yc=sensorConfig.get(3).asDouble();
            double th=sensorConfig.get(4).asDouble();
            double gain=sensorConfig.get(5).asDouble();
            int    lrMirror=sensorConfig.get(6).asInt();
            int    layoutNum=sensorConfig.get(7).asInt();

            yDebug("%s %d %f",type.c_str(),id,gain);

            if (id>=0 && id<MAX_SENSOR_NUM)
            {
                if (sensor[id])
                {
                    yError("Triangle %d already exists.",id);
                }
                else
                {
                    if (type=="triangle")
                    {
                        sensor[id]=new Triangle(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="triangle_10pad")
                    {
                        sensor[id]=new Triangle_10pad(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip")
                    {
                        sensor[id]=new Fingertip(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip2L")
                    {
                        sensor[id]=new Fingertip2L(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="fingertip2R")
                    {
                        sensor[id]=new Fingertip2R(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="quad16")
                    {
                        sensor[id]=new Quad16(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type == "cer_sh_pdl")
                    {
                        sensor[id] = new CER_SH_PDL(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type=="palmR")
                    {
                        sensor[id]=new PalmR(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type=="palmL")
                    {
                        sensor[id]=new PalmL(xc,yc,th,gain,layoutNum,lrMirror);
                    }
                    else if (type == "cer_sh_pdl")
                    {
                        sensor[id] = new CER_SH_PDL(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_pdr")
                    {
                        sensor[id] = new CER_SH_PDR(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_pp")
                    {
                        sensor[id] = new CER_SH_PP(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_td")
                    {
                        sensor[id] = new CER_SH_TD(xc, yc, th, gain, layoutNum, lrMirror);
                    }
                    else if (type == "cer_sh_tp")
                    {
                        sensor[id] = new CER_SH_TP(xc, yc, th, gain, layoutNum, lrMirror);
                    }

                    sensor[id]->setCalibrationFlag(useCalibration);
                    ++sensorsNum;
                }
            }
            else
            {
                yWarning(" %d is invalid triangle Id [0:%d].",id, MAX_SENSOR_NUM-1);
            }
        }
        else
        {
            yWarning(" sensor type %s unknown, discarded.",type.c_str());
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

bool SkinMeshThreadPort::threadInit()
{
    yDebug("SkinMeshThreadPort initialising..");
    yDebug("..done!");

    yInfo("Waiting for port connection..");
    return true;
}

void SkinMeshThreadPort::run()
{
    std::lock_guard<std::mutex> lck(mtx);

    bool gotRealData=false;
    bool gotVirtualData=false;
    yarp::sig::Vector skin_value;
    yarp::sig::Vector skin_value_virtual;
    std::vector<unsigned char> skin_color_virtual;

    // Read data from the real contacts
    if (Bottle *input=skin_port.read(false))
    {
        yTrace("Reading from real contacts...");
        gotRealData=true;

        skin_value.resize(input->size(),0.0);
        for (int i=0; i<input->size(); i++)
        {
            skin_value[i]=input->get(i).asDouble();
        }
    }

    // Read data from the virtual contacts
    if (Bottle *input_virtual=skin_port_virtual.read(false))
    {
        yTrace("Reading from virtual contacts...");
        gotVirtualData=true;

        Bottle *data_virtual  = input_virtual->get(0).asList();
        skin_value_virtual.resize(data_virtual->size(),0.0);
        Bottle *color_virtual = input_virtual->get(1).asList();

        for (int i=0; i<data_virtual->size(); i++)
        {
            skin_value_virtual[i] = data_virtual->get(i).asDouble();
        }

        for (int i = 0; i<color_virtual->size(); i++)
        {
            skin_color_virtual.push_back(color_virtual->get(i).asInt());
        }

        yTrace("Virtual contacts: %s",skin_value_virtual.toString(3,3).c_str());
        yTrace("\n");
        yTrace("Virtual contacts color: %i %i %i",skin_color_virtual[0],skin_color_virtual[1],skin_color_virtual[2]);
    }

    for (int sensorId=0; sensorId<MAX_SENSOR_NUM; sensorId++)
    {
        if (sensor[sensorId]==0) continue;

        // First, let's see if this touchSensor is over threshold in the real skin
        bool isRealSensorOverThreshold=false;
        if (gotRealData)
        {
            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
            {
                if (skin_value[i]>skinThreshold)
                {
                    isRealSensorOverThreshold = true;
                    break;
                }
            }
        }

        // Second, let's see if this touchSensor is over threshold in the virtual skin
        bool isVirtualSensorOverThreshold=false;
        if (gotVirtualData)
        {
            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
            {
                if (skin_value_virtual[i]>skinThreshold)
                {
                    isVirtualSensorOverThreshold = true;
                    break;
                }
            }
        }

        // Then, let's process the sensor with either the real or the virtual (or none)
        // EVerything will be clearly decoupled, a couple lines more do not hurt

        // If there are both, handle them
        if (gotRealData && gotVirtualData)
        {
            // Then, let's process the sensor with either the real or the virtual skin
            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
            {
                int curr_tax = i-sensor[sensorId]->min_tax;

                if (isRealSensorOverThreshold)
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
                    sensor[sensorId]->setColor(defaultColor[0],defaultColor[1],defaultColor[2]);
                }
                else if(isVirtualSensorOverThreshold)
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value_virtual[i],curr_tax);
                    sensor[sensorId]->setColor(skin_color_virtual[0],skin_color_virtual[1],skin_color_virtual[2]);
                }
                else
                {
                    sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
                    sensor[sensorId]->setColor(defaultColor[0],defaultColor[1],defaultColor[2]);
                }
            }
        }
        else if (gotRealData || gotVirtualData)
        {
            for (int sensorId=0; sensorId<MAX_SENSOR_NUM; sensorId++)
            {
                if (sensor[sensorId]==0) continue;

                for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
                {
                    int curr_tax = i-sensor[sensorId]->min_tax;

                    if (gotRealData)
                    {
                        sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
                        sensor[sensorId]->setColor(defaultColor[0],defaultColor[1],defaultColor[2]);
                    }
                    else if (gotVirtualData)
                    {
                        sensor[sensorId]->setActivationFromPortData(skin_value_virtual[i],curr_tax);
                        sensor[sensorId]->setColor(skin_color_virtual[0],skin_color_virtual[1],skin_color_virtual[2]);
                    }
                }
            }
        }
    }
}

void SkinMeshThreadPort::threadRelease()
{
    yDebug("SkinMeshThreadPort releasing...");
    skin_port.close();
    skin_port_virtual.close();
    yDebug("... done.");
}
