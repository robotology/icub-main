// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubLegsCalibrator.h"
#include <math.h>

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the legs of the Legs iCub
const int PARK_TIMEOUT=30;
const int GO_TO_ZERO_TIMEOUT=20;
const int CALIBRATE_JOINT_TIMEOUT=25;
const double POSITION_THRESHOLD=2.0;

iCubLegsCalibrator::iCubLegsCalibrator()
{
	canID  = -1;
    type   = NULL;
    param1 = NULL;
    param2 = NULL;
    param3 = NULL;
	original_pid = NULL;
    limited_pid = NULL;
	maxPWM = NULL;
    currPos = NULL;
    currVel = NULL;
	zeroPos = NULL;
    zeroVel = NULL;
    homeVel=0;
    homePos=0;
}

iCubLegsCalibrator::~iCubLegsCalibrator()
{
    //empty now
}

bool iCubLegsCalibrator::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(stderr, "LEGSCALIB::Cannot understand configuration parameters\n");
        return false;
    }

	canID =  p.findGroup("CAN").find("CanDeviceNum").asInt();

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
    type = new unsigned char[nj];
    param1 = new double[nj];
    param2 = new double[nj];
    param3 = new double[nj];
	maxPWM = new int[nj];

    zeroPos = new double[nj];
    zeroVel = new double[nj];
	currPos = new double[nj];
    currVel = new double[nj];
    homePos = new double[nj];
    homeVel = new double[nj];

    Bottle& xtmp = p.findGroup("CALIBRATION").findGroup("Calibration1");
    int i;
    for (i = 1; i < xtmp.size(); i++)
        param1[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("CALIBRATION").findGroup("Calibration2");

    for (i = 1; i < xtmp.size(); i++)
        param2[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("CALIBRATION").findGroup("Calibration3");

    for (i = 1; i < xtmp.size(); i++)
        param3[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("CALIBRATION").findGroup("CalibrationType");

    for (i = 1; i < xtmp.size(); i++)
        type[i-1] = (unsigned char) xtmp.get(i).asDouble();


    xtmp = p.findGroup("CALIBRATION").findGroup("PositionZero");

    for (i = 1; i < xtmp.size(); i++)
        zeroPos[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("VelocityZero");

    for (i = 1; i < xtmp.size(); i++)
        zeroVel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("PositionHome");

    for (i = 1; i < xtmp.size(); i++)
        homePos[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("VelocityHome");

    for (i = 1; i < xtmp.size(); i++)
        homeVel[i-1] = xtmp.get(i).asDouble();

   if (p.findGroup("CALIBRATION").check("MaxPWM")) 
   {
	   xtmp = p.findGroup("CALIBRATION").findGroup("MaxPWM");
	   for (i = 1; i < xtmp.size(); i++) maxPWM[i-1] =  xtmp.get(i).asInt();
   }
   else
   {
	   fprintf(stderr, "ARMCALIB[%d] :MaxPWM parameter not found, assuming 60\n", canID);
	   for (i = 1; i < nj+1; i++) maxPWM[i-1] = 60;
   }

    return true;
}

bool iCubLegsCalibrator::close ()
{
    if (type != NULL) delete[] type;
    type = NULL;
    if (param1 != NULL) delete[] param1;
    param1 = NULL;
    if (param2 != NULL) delete[] param2;
    param2 = NULL;
    if (param3 != NULL) delete[] param3;
    param3 = NULL;

	if (maxPWM != NULL) delete [] maxPWM;
	maxPWM = NULL;
    if (original_pid != NULL) delete [] original_pid;
	original_pid = NULL;
    if (limited_pid != NULL) delete [] limited_pid;
	limited_pid = NULL;

    if (currPos != NULL) delete[] currPos;
    currPos = NULL;
    if (currVel != NULL) delete[] currVel;
    currVel = NULL;

	if (zeroPos != NULL) delete[] zeroPos;
    zeroPos = NULL;
    if (zeroVel != NULL) delete[] zeroVel;
    zeroVel = NULL;

    if (homePos != NULL) delete[] homePos;
    homePos = NULL;
    if (homeVel != NULL) delete[] homeVel;
    homeVel = NULL;

    return true;
}

bool iCubLegsCalibrator::calibrate(DeviceDriver *dd)
{
    fprintf(stderr, "Calling iCubLegsCalibrator::calibrate!\n");
    abortCalib=false;

    dd->view(iCalibrate);
    dd->view(iEncoders);
    dd->view(iPosition);
    dd->view(iPids);
    dd->view(iControlMode);

    if (!(iCalibrate&&iPosition&&iPids&&iControlMode))
        return false;

    // ok we have all interfaces
    int nj=0;
    bool ret=iEncoders->getAxes(&nj);

    if (!ret)
        return false;

	original_pid=new Pid[nj];
	limited_pid =new Pid[nj];
	bool calibration_ok=true;
    int k=0;
	int j=0;

	for (j=0; j < nj; j++)
	{
        Time::delay(0.040);
        //fprintf(stderr, "LEGSCALIB::Sending offset for joint %d\n", k);
        //iEncoders->getEncoders(currPos);
		//fprintf(stderr, "LEGSCALIB[%d]: calibrating leg (j:%d) current enc values: %.2f %.2f %.2f %.2f\n", canID, k, currPos[0], currPos[1], currPos[2], currPos[3]);
		calibrateJoint(j);
	}

    for(k = 0; k < nj; k++) 
    {
		iPids->getPid(k,&original_pid[k]);
		limited_pid[k]=original_pid[k];
		limited_pid[k].max_int=maxPWM[k];
		limited_pid[k].max_output=maxPWM[k];
		iPids->setPid(k,limited_pid[k]);
        iControlMode->setControlMode((k), VOCAB_CM_POSITION);
    }

	for (k = 0; k < nj; k++)
    {
        //fprintf(stderr, "LEGSCALIB::Moving joint %d to zero\n", k);
		goToZero(k);
    }
	for (k = 0; k < nj; k++)
    {
        //fprintf(stderr, "ARMCALIB::Waiting for joint %d movement\n", k);
		calibration_ok &= checkGoneToZeroThreshold(k);
    }
    if (calibration_ok)
	{
		fprintf(stderr, "LEGSCALIB[%d]: Calibration done!\n", canID);
		for (k = 0; k < nj; k++)
			iPids->setPid(k,original_pid[k]);
	}
	else
	{
		fprintf(stderr, "LEGSCALIB[%d]: Calibration failed!\n", canID);
		for (k = 0; k < nj; k++)
            iControlMode->setControlMode((k), VOCAB_CM_IDLE);
	}

    ret = true;

    fprintf(stderr, "LEGSCALIB[%d]: Calibration done!\n", canID);
    return ret;
}

void iCubLegsCalibrator::calibrateJoint(int joint)
{
    fprintf(stderr, "LEGSCALIB[%d]: Calling calibrateJoint on joint %d with params: %d  %+6.1f %+6.1f %+6.1f\n", canID, joint, type[joint], param1[joint], param2[joint], param3[joint]);
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool iCubLegsCalibrator::checkCalibrateJointEnded(int joint)
{
    const int timeout = CALIBRATE_JOINT_TIMEOUT;
    int i;
    for (i = 0; i < timeout; i++)
    {
        if (iCalibrate->done(joint))
            break;   
        if (abortCalib)
            break;
        Time::delay(1.0);
    }
    if (i == timeout)
    {
        fprintf(stderr, "LEGSCALIB::Timeout on joint %d while calibrating!\n", joint);
        return false;
    }
    else if (abortCalib)
        {
            fprintf(stderr, "LEGSCALIB::Aborting calibration of joint %d\n", joint);
            return false;
        }
    else
        {
            fprintf(stderr, "LEGSCALIB::calibration of joint %d done\n", joint);
        }

    return true;
}


void iCubLegsCalibrator::goToZero(int j)
{
    if (abortCalib)
        return;
    iControlMode->setPositionMode(j);
    iPosition->setRefSpeed(j, zeroVel[j]);
    iPosition->positionMove(j, zeroPos[j]);
}

void iCubLegsCalibrator::checkGoneToZero(int j)
{
    // wait.
    bool finished = false;
    int timeout = 0;
    while ( (!finished) && (!abortCalib))
    {
        iPosition->checkMotionDone(j, &finished);

        Time::delay (0.5);
        timeout ++;
        if (timeout >= GO_TO_ZERO_TIMEOUT)
        {
            fprintf(stderr, "LEGSCALIB[%d] Timeout on joint %d while going to zero!\n", canID, j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(stderr, "LEGSCALIB[%d] abort wait for joint %d going to zero!\n", canID, j);
}

bool iCubLegsCalibrator::checkGoneToZeroThreshold(int j)
{
    // wait.
    bool finished = false;
    int timeout = 0;
	double ang=0;
	double delta=0;
    while ( (!finished) && (!abortCalib))
    {
		iEncoders->getEncoder(j, &ang);
		delta = fabs(ang-zeroPos[j]);
		fprintf(stderr, "LEGSCALIB[%d] (joint %d) curr:%.2f des:%.2f -> delta:%.2f\n", canID, j, ang, zeroPos[j], delta);
		if (delta<POSITION_THRESHOLD)
		{
			fprintf(stderr, "LEGSCALIB[%d] (joint %d) completed! delta:%f\n", canID, j,delta);
			finished=true;
		}

        Time::delay (0.5);
        timeout ++;

        if (timeout >= GO_TO_ZERO_TIMEOUT)
        {
            fprintf(stderr, "LEGSCALIB[%d]: Timeout on joint %d while going to zero!\n", canID, j);
			return false;
        }
    }
    if (abortCalib)
        fprintf(stderr, "LEGSCALIB[%d]: Abort wait for joint %d going to zero!\n", canID, j);

	return finished;
}

bool iCubLegsCalibrator::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    bool ret=false;
    abortParking=false;

    ret=iEncoders->getAxes(&nj);
    if (!ret)
    {
       fprintf(stderr, "LEGSCALIB[%d]: error getting number of encoders\n",canID);
       return false;
    }



    int timeout = 0;
    fprintf(stderr, "LEGSCALIB[%d]: Calling iCubLegsCalibrator::park() \n",canID);
    iPosition->setPositionMode();
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
    {
        fprintf(stderr, "LEGSCALIB[%d]: Moving to park positions \n",canID);
        bool done=false;
        while( (!done) && (timeout < PARK_TIMEOUT) && (!abortParking))
        {
            iPosition->checkMotionDone(&done);
            fprintf(stderr, ".");
            Time::delay(1);
            timeout++;
        }
		if(!done)
        {
			for(int j=0; j < nj; j++)
			{
				iPosition->checkMotionDone(j, &done);
				if (iPosition->checkMotionDone(j, &done))
				{
					if (!done)
						fprintf(stderr, "LEGSCALIB:[%d]: joint %d not in position\n", canID, j);
				}
				else
					fprintf(stderr, "LEGSCALIB[%d]: joint %d did not answer during park() \n", canID, j);
			}
		}
    }

    if (abortParking)
	    fprintf(stderr, "LEGSCALIB[%d]::Park was aborted!\n", canID);
    else
        fprintf(stderr, "LEGSCALIB[%d]::Park was done!\n", canID);
    return true;
}

bool iCubLegsCalibrator::quitCalibrate()
{
    fprintf(stderr, "LEGSCALIB[%d]: Quitting calibrate\n", canID);
    abortCalib=true;
    return true;
}

bool iCubLegsCalibrator::quitPark()
{
    fprintf(stderr, "LEGSCALIB[%d]::quitting park\n", canID);
    abortParking=true;
    return true;
}
