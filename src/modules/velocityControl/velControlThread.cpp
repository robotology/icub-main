// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright (C) 2008 RobotCub Consortium
// Author: Lorenzo Natale
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include "velControlThread.h"
#include <string.h>
#include <string>
#include <math.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

const double MAX_SPEED=250;
const double AVERAGE=50;
const double ACCELERATIONS=1000000;
const double MAX_GAIN = 10.0;

const int VELOCITY_INDEX_OFFSET=1000;

velControlThread::velControlThread(int rate):
                yarp::os::RateThread(rate)
{
    control_rate = rate;
    suspended = true;
}

velControlThread::~velControlThread()
{}

void velControlThread::run()
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

    ////getting new commands from the fast command port
    //this command receives also feedforward velocities
    yarp::os::Bottle *bot = command_port.read(false);
    if(bot!=NULL)
    {
        nb_void_loops = 0;
        //fprintf(stderr, "\n Receiving command: \t");
        int size = bot->size()/2;
        for(int i=0;i<size;i++)
        {
            int ind = bot->get(2*i).asInt();
            if(ind < VELOCITY_INDEX_OFFSET) 
            {//this is a position command
                targets(ind) = bot->get(2*i+1).asDouble();
            } 
            else 
            {//this is a velocity command
                ffVelocities(ind - VELOCITY_INDEX_OFFSET) = bot->get(2*i+1).asDouble();
            }
            //fprintf(stderr, "for joint *%d, received %f, \t", ind, targets(ind));
        }
    } else {
        nb_void_loops++;
        if(nb_void_loops > 5) {
            ffVelocities = 0.0;
        }
    }

    //getting commands from the slow port
    yarp::os::Bottle *vec = command_port2.read(false);
    if (vec!=NULL)
    {
        for(int k=0;k<nJoints;k++)
            targets(k)=vec->get(k).asDouble();
    }

    static int count=0;
    count++;

    ienc->getEncoders(encoders.data());

    //ienc->getEncoderSpeeds(encoders_speed.data());
    //fprintf(stderr, "printing to file \n");
#if 0
    for(int i=0;i<nJoints;i++)
        fprintf(currentSpeedFile,"%f ",encoders(i));
    fprintf(currentSpeedFile,"%f\n",t_start-time_watch);
#endif

    Kd=0.0;

    for(int k=0;k<nJoints;k++)
    {
      //  printf ("%+6.3f ", targets(k));
        double current_err = targets(k)-encoders(k);
        error_d(k) = (current_err - error(k))/((double)control_rate)*1000.0;
        error(k) = current_err;

        //we calculate the command Adding the ffVelocities
        command(k) = Kp(k)*error(k) + Kd(k)*error_d(k) + ffVelocities(k);
        //printf ("%+3.3f ", command(k));
    }
    //printf ("\n");
    //    std::cout << command.toString() << std::endl;
   //     printf ("\n");
    limitSpeed(command);

    if (suspended)
        command=0;

#if 0
    for(int i=0;i<nJoints;i++)
        fprintf(targetSpeedFile,"%f ",command(i));
    fprintf(targetSpeedFile,"%f\n",t_start-time_watch);
#endif
    //printf ("%s\n", command.toString().c_str());
    if(!suspended)
    {
        if (0)
        {
            //@@@this is used to directly command positions, with no minum jerk tajectory
            //not to be used (risky!), unless you know wahat you are doing.
            //ipid->setReferences(this->targets.data());
            for(int i=0;i<nJoints;i++)
            {
                if (fabs(targets(i)-encoders(i))>6.0)
                    ipos->positionMove(i,targets(i));
                else
                    ipid->setPidReference(yarp::dev::VOCAB_PIDTYPE_POSITION,i,targets(i));
            }
        }
        else
        {
            int trials = 0;
            while(!ivel->velocityMove(command.data()))
            {
                trials++;
                yError("velcontrol ERROR>> velocity move sent false\n");
                if(trials>10)
                {
                yError("velcontrol ERROR>> tried 10 times to velocityMove, halting...\n");
                this->halt();
                break;
                }
            }
        }
    }
    _mutex.post();

}

bool velControlThread::threadInit()
{
    suspended=true;
    ienc->getEncoders(encoders.data());
    //ienc->getEncoderSpeeds(encoders_speed.data());
    targets=encoders;
    ffVelocities = 0;
    count = 0;
    time_watch = Time::now();
    time_loop = 0.0;

    return true;
}

void velControlThread::threadRelease()
{
    for(int k=0;k<nJoints;k++)
    {
        ivel->stop();
        for (int k = 0; k < nJoints; k++)
            imod->setControlMode(k, VOCAB_CM_POSITION);
        suspended = true;
    }

    command_port.close();
    command_port2.close();

#if 0
    fclose(targetSpeedFile);
    fclose(currentSpeedFile);
#endif
}

bool velControlThread::init(PolyDriver *d, std::string partName, std::string robotName)
{
    char tmp[255];

    yarp::os::Time::turboBoost();

    nb_void_loops = 0;

    ///opening port for fast transfer of position command
    sprintf(tmp,"/%s/vc/%s/fastCommand", robotName.c_str(), partName.c_str());
    yInfo("opening port for part %s\n",tmp);
    command_port.open(tmp);

    std::string tmp2;
    tmp2="/";
    tmp2+=robotName.c_str();
    tmp2+="/vc/";
    tmp2+=partName.c_str();
    tmp2+="/command";
    command_port2.open(tmp2.c_str());

    if (d==0)
        return false;

    driver=d;

    driver->view(ivel);
    driver->view(ipos);
    driver->view(ienc);
    driver->view(imod);
    driver->view(ipid);

    if ( (ivel==0)||(ienc==0) || (imod==0) || (ipid==0) || (ipos==0))
        return false;

    ivel->getAxes(&nJoints);
    yInfo("controlling %d DOFs\n",nJoints);

    Vector accs;
    accs.resize(nJoints);
    Vector speeds;
    speeds.resize(nJoints);
    ivel->getRefAccelerations(accs.data());
    accs=ACCELERATIONS;
    ivel->setRefAccelerations(accs.data());
    speeds=50.0;
    ipos->setRefSpeeds(speeds.data());

    encoders.resize(nJoints);
    encoders_speed.resize(nJoints);
    command.resize(nJoints);
    targets.resize(nJoints);
    ffVelocities.resize(nJoints);
    command=0;
    targets=0;
    ffVelocities = 0;
    Kp.resize(nJoints);
    Kp=0;
    Kd.resize(nJoints);
    Kd=0;
    error.resize(nJoints);
    error=0;
    error_d.resize(nJoints);
    error_d=0;

    maxVel.resize(nJoints);
    maxVel = 0.0;

#if 0
    sprintf(tmp,"%s_target_speed.dat",partName.c_str());
    targetSpeedFile = fopen(tmp,"w");
    sprintf(tmp,"%s_current_speed.dat",partName.c_str());
    currentSpeedFile = fopen(tmp,"w");
#endif

    return true;
}

void velControlThread::halt()
{
    suspended=true;
    ivel->stop();
    yInfo("Suspended\n");
    targets=encoders;
    ffVelocities = 0;
}

void velControlThread::go()
{
    suspended=false;
    int mode=0;
    for(int k = 0; k < nJoints; k++)
    {
        imod->getControlMode(k, &mode);
        if (mode!=VOCAB_CM_MIXED && mode!=VOCAB_CM_VELOCITY)
        {
            std::string s = yarp::os::Vocab::decode(mode);
            yWarning("Joint (%d) is in mode (%s) and does not accepts velocty commands. You have first to set either VOCAB_CM_VELOCITY or VOCAB_CM_MIXED control mode\n", k, s.c_str());
        }
    }
    yInfo("Run\n");
    targets=encoders;
    ffVelocities = 0;
}

void velControlThread::setRef(int i, double pos)
{
    yInfo("Setting new target %d to %lf\n", i, pos);

    _mutex.wait();
    targets(i)=pos;
    ffVelocities(i)=0;
    _mutex.post();
}

void velControlThread::setVel(int i, double vel)
{
    _mutex.wait();

    if((vel > 0.0) && (vel < MAX_SPEED) && (i>=0) && (i<nJoints))
    {
        maxVel(i) = vel;
        yInfo("setting max vel of joint %d to %f\n",i,maxVel(i));
    }
    else
        yError("Impossible to set max vel higher than %f\n",MAX_SPEED);

    _mutex.post();
}

void velControlThread::setGain(int i, double gain)
{
    _mutex.wait();

    if (gain>MAX_GAIN)
        gain=MAX_GAIN;

    if((gain >= 0.0) && (i>=0) && (i<nJoints))
    {
        Kp(i) = gain;
        yInfo("Setting gain for joint %d to %lf\n", i, Kp(i));
    }
    else
        yError("Cannot set gain of joint %d\n",i);

    _mutex.post();
}


void velControlThread::limitSpeed(Vector &v)
{
    for(int k=0; k<nJoints;k++)
    {
        if(command(k)!=command(k))//check not Nan
        {
            command(k)=0.0;
            yWarning("WARNING::Receiving NaN values\n");
        }
        if (fabs(command(k))>maxVel(k))
        {
            if (command(k)>0)
                command(k)=maxVel(k);
            else
                command(k)=-maxVel(k);
        }
    }
}
