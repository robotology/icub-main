// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium
 * Author: Marco Randazzo, Marco Maggiali, Alessandro Scalzo
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */ 

#include "include/SkinMeshThreadPort.h"

#include <yarp/os/Time.h>

SkinMeshThreadPort::SkinMeshThreadPort(Searchable& config,int period) : RateThread(period),mutex(1)
{
    yDebug("SkinMeshThreadPort running at %g ms.",getRate());
    mbSimpleDraw=config.check("light");

    sensorsNum=0;

    for (int t=0; t<MAX_SENSOR_NUM; ++t)
    {
        sensor[t]=NULL;
    }

    std::string part="/skinGui/";
    
    if (config.check("name"))
    {
        part ="/";
        part+=config.find("name").asString().c_str();
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
        yInfo("Using calibrated skin values (0-255)");
    }
    else
    {
        yDebug("Using raw skin values (255-0)");
    }
    
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

            yDebug("%s %d %f",type.c_str(),id,gain);

            if (id>=0 && id<MAX_SENSOR_NUM)
            {
                if (sensor[id])
                {
                    yWarning("WARNING: triangle %d already exists.",id);
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
    mutex.wait();

    if (Bottle *input=skin_port.read(false)) 
    {    
        yarp::sig::Vector skin_value;
        skin_value.resize(input->size());
        for (int i=0; i<input->size(); i++)
        {
            skin_value[i] = input->get(i).asDouble();
        }

        for (int sensorId=0; sensorId<MAX_SENSOR_NUM; sensorId++)
        {
            if (sensor[sensorId]==0) continue;

            for (int i=sensor[sensorId]->min_tax; i<=sensor[sensorId]->max_tax; i++)
            {
                int curr_tax = i-sensor[sensorId]->min_tax;
            
                sensor[sensorId]->setActivationFromPortData(skin_value[i],curr_tax);
            }
        }
    }

    mutex.post();
}

void SkinMeshThreadPort::threadRelease()
{
    yDebug("SkinMeshThreadPort releasing...");
    yDebug("... done.");
}
