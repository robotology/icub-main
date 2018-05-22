// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright (C) 2015 Istituto Italiano di Tecnologia - iCub Facility
// Author: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "positionDirectThread.h"
#include <cstring>
#include <string>
#include <cmath>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

positionDirectControlThread::positionDirectControlThread(int period):
                yarp::os::RateThread(period)
{
    control_period = period;
    suspended = true;
    control_joints_list = 0;
}

positionDirectControlThread::~positionDirectControlThread()
{
}

void positionDirectControlThread::run()
{
    double t_start = yarp::os::Time::now();

    if (getIterations()>100)
    {
        yDebug("Thread ran %d times, est period %lf[ms], used %lf[ms]\n",
                getIterations(),
                getEstPeriod(),
                getEstUsed());
        resetStat();
    }
    _mutex.wait();

    //read the position targets
    yarp::os::Bottle *bot = command_port.read(false);
    if(bot!=NULL)
    {
        unsigned int botsize = bot->size();
        if (botsize == control_joints)
        {
            prev_targets=targets;
            for (unsigned int i=0; i< control_joints; i++)
            {
                targets[i] = bot->get(i).asDouble();
            }
        }
        else
        {
            yError ("Your bottle does not have the right size: module is configured to control %d joints", control_joints);
        }
    }
    else
    {
        _mutex.post();
        return;
    }

    //apply the joints limits
    for (unsigned int i=0; i< control_joints; i++)
    {
        if (targets[i]>max_limits[i]) targets[i]=max_limits[i];
        if (targets[i]<min_limits[i]) targets[i]=min_limits[i];
    }

    //get the current position
    for (unsigned int i=0; i< control_joints; i++)
    {
        double val =0;
        ienc->getEncoder(control_joints_list[i],&val);
        encoders[i]=val;
    }

    //apply a limit on the difference between prev target and current target
    double cmd_step = 1.0;
    for (unsigned int i=0; i< control_joints; i++)
    {
        double diff = (targets[i]-prev_targets[i]);
        if      (diff > +cmd_step) targets[i]=prev_targets[i]+cmd_step;
        else if (diff < -cmd_step) targets[i]=prev_targets[i]-cmd_step;
    }

    //slew rate limiter
    double pos_step = 2.0;
    for(unsigned int i = 0; i<control_joints; i++)
    {
        double diff = (targets[i]-encoders[i]);
        if      (diff > +pos_step) targets[i]=encoders[i]+pos_step;
        else if (diff < -pos_step) targets[i]=encoders[i]-pos_step;
    }

    //appy the command
    for(unsigned int i = 0; i<control_joints; i++)
    {
        idir->setPosition(control_joints_list[i],targets[i]);
    }

    _mutex.post();

}

bool positionDirectControlThread::threadInit()
{
    suspended=true;
    return true;
}

void positionDirectControlThread::threadRelease()
{
    for(unsigned int i=0; i<control_joints; i++)
    {
       imod->setControlMode(control_joints_list[i], VOCAB_CM_POSITION);
    }

    suspended = true;
    command_port.close();
}

bool positionDirectControlThread::init(PolyDriver *d, std::string moduleName, std::string partName, std::string robotName, Bottle* jointsList)
{
    yarp::os::Time::turboBoost();

    ///opening port command input
    char tmp[255];
    sprintf(tmp, "/%s/%s/%s/command:i", moduleName.c_str(), robotName.c_str(), partName.c_str());
    yInfo("opening port for part %s\n",tmp);
    command_port.open(tmp);

    if (d==0)
    {
        yError ("Invalid device driver pointer");
        return false;
    }

    driver=d;
    driver->view(idir);
    driver->view(ipos);
    driver->view(ienc);
    driver->view(imod);
    driver->view(ilim);

    if ( (idir==0)||(ienc==0) || (imod==0) || (ipos==0) || (ilim==0))
    {
        yError ("Failed to view motor interfaces");
        return false;
    }

    int tmpj=0;
    ipos->getAxes(&tmpj);
    part_joints=tmpj;
    control_joints= jointsList->size();
    if (control_joints>part_joints)
    {
        yError ("you cannot control more of %d joints for this robot part", part_joints);
        return false;
    }
    else if (control_joints<=0)
    {
        yError ("invalid number of control joints (%d)", control_joints);
        return false;
    }
    else
    {
        control_joints_list = new int [control_joints];
        for (unsigned int i=0; i< control_joints; i++)
        {
            if (jointsList->get(i).isInt() && jointsList->get(i).asInt()>=0)
            {
                control_joints_list[i] = jointsList->get(i).asInt();
            }
            else
            {
                yError ("invalid list of jonts to control");
                return false;
            }
        }
    }
    yInfo("part has %d joints, controlling %d joints\n",part_joints, control_joints);

    Vector speeds;
    speeds.resize(control_joints);
    speeds=10.0;
    for (unsigned int i=0; i<control_joints; i++)
    {
        ipos->setRefSpeed(control_joints_list[i],speeds[i]);
    }

    encoders.resize(control_joints);
    targets.resize(control_joints);
    prev_targets.resize(control_joints);
    error.resize(control_joints);
    encoders.zero();
    targets.zero();
    prev_targets.zero();
    error.zero();

    min_limits.resize(control_joints);
    max_limits.resize(control_joints);
    for (unsigned int i=0; i<control_joints; i++)
    {
        double min=0;
        double max=0;
        ilim->getLimits(control_joints_list[i],&min,&max);
        min_limits[i]=min;
        max_limits[i]=max;
    }

    for (unsigned int i=0; i<control_joints; i++)
    {
        imod->setControlMode(control_joints_list[i],VOCAB_CM_POSITION_DIRECT);
    }

    //get the current position
    for (unsigned int i=0; i< control_joints; i++)
    {
        double val =0;
        ienc->getEncoder(control_joints_list[i],&val);
        targets[i] = encoders[i] = val;
        prev_targets[i] = encoders[i];
    }

    return true;
}

void positionDirectControlThread::halt()
{
    suspended=true;
    yInfo("Suspended\n");
}

void positionDirectControlThread::go()
{
    suspended=false;
    yInfo("Run\n");
}

void positionDirectControlThread::setVel(int i, double vel)
{
    _mutex.wait();

    _mutex.post();
}

void positionDirectControlThread::setGain(int i, double gain)
{
    _mutex.wait();

    _mutex.post();
}


void positionDirectControlThread::limitSpeed(Vector &v)
{

}
