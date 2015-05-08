// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright (C) 2015 Istituto Italiano di Tecnologia - iCub Facility
// Author: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <iCub/DualCamCalibModule.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

CamCalibModule::CamCalibModule()
{
    calibToolLeft = NULL;
    calibToolRight = NULL;
    align=ALIGN_WIDTH;
    requested_fps=0;
    time_lastOut=yarp::os::Time::now();
}

CamCalibModule::~CamCalibModule()
{

}

bool CamCalibModule::configure(yarp::os::ResourceFinder &rf)
{
    ConstString str = rf.check("name", Value("/camCalib"), "module name (string)").asString();
    verboseExecTime = rf.check("verboseExecTime");

    if      (rf.check("w_align")) align=ALIGN_WIDTH;
    else if (rf.check("h_align")) align=ALIGN_HEIGHT;

    if (rf.check("fps"))
    {
        requested_fps=rf.find("fps").asDouble();
        yInfo() << "Module will run at " << requested_fps;
    }
    else
    {
        yInfo() << "Module will run at max fps";
    }

    setName(str.c_str()); // modulePortName

    Bottle botLeftConfig(rf.toString().c_str());
    Bottle botRightConfig(rf.toString().c_str());

    botLeftConfig.setMonitor(rf.getMonitor());
    botRightConfig.setMonitor(rf.getMonitor());

    string strLeftGroup = "CAMERA_CALIBRATION_LEFT";
    if (botLeftConfig.check(strLeftGroup.c_str()))
    {            
        Bottle &group=botLeftConfig.findGroup(strLeftGroup.c_str(),string("Loading configuration from group " + strLeftGroup).c_str());
        botLeftConfig.fromString(group.toString());
    }
    else
    {
        yError() << "Group " << strLeftGroup << " not found.";
        return false;
    }
    
    string strRightGroup = "CAMERA_CALIBRATION_RIGHT";
    if (botRightConfig.check(strRightGroup.c_str()))
    {            
        Bottle &group=botRightConfig.findGroup(strRightGroup.c_str(),string("Loading configuration from group " + strRightGroup).c_str());
        botRightConfig.fromString(group.toString());
    }
    else
    {
        yError() << "Group " << strRightGroup << " not found.";
        return false;
    }

    string calibToolLeftName  = botLeftConfig.check("projection", Value("pinhole"), "Projection/mapping applied to calibrated image [projection|spherical] (string).").asString().c_str();
    string calibToolRightName = botRightConfig.check("projection", Value("pinhole"), "Projection/mapping applied to calibrated image [projection|spherical] (string).").asString().c_str();

    calibToolLeft = CalibToolFactories::getPool().get(calibToolLeftName.c_str());
    if (calibToolLeft!=NULL)
    {
        bool ok = calibToolLeft->open(botLeftConfig);
        if (!ok)
        {
            delete calibToolLeft;
            calibToolLeft = NULL;
            return false;
        }
    }
    calibToolRight = CalibToolFactories::getPool().get(calibToolRightName.c_str());
    if (calibToolRight!=NULL)
    {
        bool ok = calibToolRight->open(botRightConfig);
        if (!ok)
        {
            delete calibToolRight;
            calibToolRight = NULL;
            return false;
        }
    }

    if (yarp::os::Network::exists(getName("/left:i")))
    {
        yWarning() << "port " << getName("/left:i") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/right:i")))
    {
        yWarning() << "port " << getName("/right:i") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/out")))
    {
        yWarning() << "port " << getName("/out") << " already in use";
    }
    if (yarp::os::Network::exists(getName("/conf")))
    {
        yWarning() << "port " << getName("/conf") << " already in use";
    }
    imageInLeft.open(getName("/left:i"));
    imageInRight.open(getName("/right:i"));
    imageInLeft.setStrict(false);
    imageInRight.setStrict(false);

    imageOut.open(getName("/out"));

    configPort.open(getName("/conf"));
    attach(configPort);

    return true;
}

bool CamCalibModule::close()
{
    imageInLeft.close();
    imageInRight.close();
    imageOut.close();
    configPort.close();
    if (calibToolLeft != NULL)
    {
        calibToolLeft->close();
        delete calibToolLeft;
        calibToolLeft = NULL;
    }
    if (calibToolRight != NULL)
    {
        calibToolRight->close();
        delete calibToolRight;
        calibToolRight = NULL;
    }
    return true;
}

bool CamCalibModule::interruptModule()
{
    imageInLeft.interrupt();
    imageInRight.interrupt();
    imageOut.interrupt();
    configPort.interrupt();
    return true;
}

bool CamCalibModule::updateModule()
{
    bool lready=false;
    bool rready=false;
    bool init=false;
    while(1)
    {
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* left  = imageInLeft.read(false);
        yarp::sig::ImageOf<yarp::sig::PixelRgb>* right = imageInRight.read(false);

        if (calibToolLeft!=NULL && left!=NULL)
        {
            calibToolLeft->apply(*left,calibratedImgLeft);
            lready=true;

            if (init==false)
            {
                int outw = 0;
                int outh = 0;
                if (align == ALIGN_WIDTH)
                {
                    outw = calibratedImgLeft.width()*2;
                    outh = calibratedImgLeft.height();
                }
                else if (align==ALIGN_HEIGHT)
                {
                    outw = calibratedImgLeft.width();
                    outh = calibratedImgLeft.height()*2;
                }
                else
                {
                    yError() << "Invalid alignment";
                }
                calibratedImgOut.copy(calibratedImgLeft,outw,outh);
                yDebug() << "created output buffer from left input, size: " << outw << "x" << outh;
                yDebug() << calibratedImgOut.width();
                yDebug() << calibratedImgOut.height();
                init=true;
            }

            for (int r=0; r<calibratedImgLeft.height(); r++)
            for (int c=0; c<calibratedImgLeft.width(); c++)
            {
                unsigned char *pixel = calibratedImgOut.getPixelAddress(c,r);
                
                pixel[0] = *(calibratedImgLeft.getPixelAddress(c,r)+0);
                pixel[1] = *(calibratedImgLeft.getPixelAddress(c,r)+1);
                pixel[2] = *(calibratedImgLeft.getPixelAddress(c,r)+2);
            }

        }
        if (calibToolRight!=NULL && right!=NULL)
        {
            calibToolRight->apply(*right,calibratedImgRight);
            rready=true;

            if (init==false)
            {
                int outw = 0;
                int outh = 0;
                if (align == ALIGN_WIDTH)
                {
                    outw = calibratedImgLeft.width()*2;
                    outh = calibratedImgLeft.height();
                }
                else if (align==ALIGN_HEIGHT)
                {
                    outw = calibratedImgLeft.width();
                    outh = calibratedImgLeft.height()*2;
                }
                else
                {
                    yError() << "Invalid alignment";
                }
                calibratedImgOut.copy(calibratedImgRight,outw,outh);
                yDebug() << "created output buffer from right input, size: " << outw << "x" << outh;
                yDebug() << calibratedImgOut.width();
                yDebug() << calibratedImgOut.height();
                init=true;
            }

            for (int r=0; r<calibratedImgLeft.height(); r++)
            for (int c=0; c<calibratedImgLeft.width(); c++)
            {
                int cp = 0;
                int rp = 0;
                if      (align == ALIGN_WIDTH)  {cp = c+calibratedImgLeft.width();  rp = r;}
                else if (align == ALIGN_HEIGHT) {cp = c; rp = r+calibratedImgLeft.height();}
                unsigned char *pixel = calibratedImgOut.getPixelAddress(cp,rp);
                pixel[0] = *(calibratedImgRight.getPixelAddress(c,r)+0);
                pixel[1] = *(calibratedImgRight.getPixelAddress(c,r)+1);
                pixel[2] = *(calibratedImgRight.getPixelAddress(c,r)+2);
            }
        }

        if (requested_fps==0)
        {
            if (lready==true || rready==true)
            {
                imageOut.write(calibratedImgOut);
                double diffOut = yarp::os::Time::now() -time_lastOut;
                time_lastOut = yarp::os::Time::now();
                if (verboseExecTime) yDebug ("%f", diffOut);
                lready=false;
                rready=false;
            }
        }
        else
        {
            double diffOut = yarp::os::Time::now() - time_lastOut;
            if (diffOut>(1/requested_fps))
            {
                imageOut.write(calibratedImgOut);
                time_lastOut = yarp::os::Time::now();
                if (verboseExecTime) yDebug ("%f", diffOut);
                lready=false;
                rready=false;
            }
        }

        yarp::os::Time::delay(0.001);
    }
    return true;
}

double CamCalibModule::getPeriod()
{
  return 0.0;
}

bool CamCalibModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else
    {
        yError() << "command not known - type help for more info";
    }
    return true;
}


