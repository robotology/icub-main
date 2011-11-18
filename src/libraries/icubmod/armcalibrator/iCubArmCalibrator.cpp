// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>
#include "iCubArmCalibrator.h"
#include <math.h>

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the arm of the Arm iCub

const int PARK_TIMEOUT=30;
const int GO_TO_ZERO_TIMEOUT=20;
const int CALIBRATE_JOINT_TIMEOUT=25;
const double POSITION_THRESHOLD=2.0;

const int numberOfJoints=16;

iCubArmCalibrator::iCubArmCalibrator()
{
    logfile = stderr;
    type   = NULL;
    param1 = NULL;
    param2 = NULL;
    param3 = NULL;
	original_pid = NULL;
    limited_pid = NULL;
    pos = NULL;
    vel = NULL;
    homeVel=0;
    homePos=0;
}

iCubArmCalibrator::~iCubArmCalibrator()
{
    //empty now
}

bool iCubArmCalibrator::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(logfile, "ARMCALIB::Cannot understand configuration parameters\n");
        return false;
    }

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
    if (nj!=numberOfJoints)
        {
            fprintf(logfile, "ARMCALIB::calibrator is for %d joints but device has %d\n", numberOfJoints, nj);
            return false;
        }
        
    type = new unsigned char[nj];
    param1 = new double[nj];
    param2 = new double[nj];
    param3 = new double[nj];

    pos = new double[nj];
    vel = new double[nj];

    homePos = new double[nj];
    homeVel = new double[nj];

    logfile_name = p.findGroup("CALIBRATION").find("Logfile").asString();
    if (logfile_name != "") 
    {
        fprintf(stdout, "ARMCALIB::calibrator: opening logfile %s\n", logfile_name.c_str());
        logfile = fopen (logfile_name.c_str(), "w");
    }
    else
    {
        fprintf(stdout, "ARMCALIB::calibrator: no logfile specified, errors displayed on standard stderr\n");
    }

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
        pos[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("VelocityZero");

    for (i = 1; i < xtmp.size(); i++)
        vel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("PositionHome");

    for (i = 1; i < xtmp.size(); i++)
        homePos[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("VelocityHome");

    for (i = 1; i < xtmp.size(); i++)
        homeVel[i-1] = xtmp.get(i).asDouble();

    return true;
}

bool iCubArmCalibrator::close ()
{
    if (type != NULL) delete[] type;
    type = NULL;
    if (param1 != NULL) delete[] param1;
    param1 = NULL;
    if (param2 != NULL) delete[] param2;
    param2 = NULL;
    if (param3 != NULL) delete[] param3;
    param3 = NULL;

    if (original_pid != NULL) delete [] original_pid;
	original_pid = NULL;
    if (limited_pid != NULL) delete [] limited_pid;
	limited_pid = NULL;

    if (pos != NULL) delete[] pos;
    pos = NULL;
    if (vel != NULL) delete[] vel;
    vel = NULL;

    if (homePos != NULL) delete[] homePos;
    homePos = NULL;
    if (homeVel != NULL) delete[] homeVel;
    homeVel = NULL;

    if (logfile_name!="") fclose(logfile);

    return true;
}

bool iCubArmCalibrator::calibrate(DeviceDriver *dd)
{
    fprintf(logfile, "Calling iCubArmCalibrator::calibrate: \n");
    abortCalib=false;

    iCalibrate = dynamic_cast<IControlCalibration2 *>(dd);
    iAmps =  dynamic_cast<IAmplifierControl *>(dd);
    iEncoders = dynamic_cast<IEncoders *>(dd);
    iPosition = dynamic_cast<IPositionControl *>(dd);
    iPids = dynamic_cast<IPidControl *>(dd);
	iControlMode = dynamic_cast<IControlMode *>(dd);

    if (!(iCalibrate&&iAmps&&iPosition&&iPids&&iControlMode))
        return false;

    // ok we have all interfaces
    int nj=0;
    bool ret=iEncoders->getAxes(&nj);

    if (!ret)
        return false;

	original_pid=new Pid[nj];
	limited_pid =new Pid[nj];
	bool calibration_ok=true;
    int k;
	int shoulderSetOfJoints[] = {0, 1 , 2, 3};
    for (k =0; k < 4; k++)
    {
        //fprintf(logfile, "ARMCALIB::Sending offset for joint %d\n", k);
        calibrateJoint(shoulderSetOfJoints[k]);
    }
    Time::delay(1.0);

    for (k = 0; k < nj; k++) 
    {
        fprintf(logfile, "ARMCALIB::Calling enable amp for joint %d\n", k);
        iAmps->enableAmp(k);
        fprintf(logfile, "ARMCALIB::Calling enable pid for joint %d\n", k);
        iPids->enablePid(k);
    }

    ret = true;
    bool x;

	////////////////////////////////////////////
    iPids->disablePid(2);
    iAmps->disableAmp(2);

	int firstSetOfJoints[] = {0, 1 , 3, 4, 6, 7, 8, 9, 11, 13};
    for (k =0; k < 10; k++)
		calibrateJoint(firstSetOfJoints[k]);
	for (k =0; k < 10; k++)
	{
		x = checkCalibrateJointEnded(firstSetOfJoints[k]);
		ret = ret && x;
	}
	for (k = 0; k < 10; k++)
		goToZero(firstSetOfJoints[k]);
	for (k = 0; k < 10; k++)
		checkGoneToZero(firstSetOfJoints[k]);
	//////////////////////////////////////////
    iAmps->enableAmp(2);
    iPids->enablePid(2);
	int secondSetOfJoints[] = {2, 5, 10, 12, 14, 15};
	for (k =0; k < 6; k++)
		calibrateJoint(secondSetOfJoints[k]);
	for (k =0; k < 6; k++)
	{
		x = checkCalibrateJointEnded(secondSetOfJoints[k]);
		ret = ret && x;
	}
	for (k = 0; k < 6; k++)
		goToZero(secondSetOfJoints[k]);
	for (k = 0; k < 6; k++)
		checkGoneToZero(secondSetOfJoints[k]);
    
    fprintf(logfile, "ARMCALIB::calibration done!\n");
    return ret;
}

void iCubArmCalibrator::calibrateJoint(int joint)
{
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool iCubArmCalibrator::checkCalibrateJointEnded(int joint)
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
        fprintf(logfile, "ARMCALIB::Timeout on joint %d while calibrating!\n", joint);
        return false;
    }
    if (abortCalib)
    {
        fprintf(logfile, "ARMCALIB::aborted\n");
    }

    return true;
}


void iCubArmCalibrator::goToZero(int j)
{
    if (abortCalib)
        return;
	iControlMode->setPositionMode(j);
    iPosition->setRefSpeed(j, vel[j]);
    iPosition->positionMove(j, pos[j]);
}

void iCubArmCalibrator::checkGoneToZero(int j)
{
    // wait.
    bool finished = false;
    int timeout = 0;
    while ( (!finished) && (!abortCalib))
    {
        iPosition->checkMotionDone(j, &finished);

        if (logfile_name != "")
            Time::delay (0.1);
        else
            Time::delay (0.5);
        timeout ++;
        if (timeout >= GO_TO_ZERO_TIMEOUT)
        {
            fprintf(logfile, "ARMCALIB::Timeout on joint %d while going to zero!\n", j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(logfile, "ARMCALIB::abort wait for joint %d going to zero!\n", j);
}

bool iCubArmCalibrator::checkGoneToZeroThreshold(int j)
{
    // wait.
    bool finished = false;
    int timeout = 0;
	double ang=0;
	double delta=0;
    while ( (!finished) && (!abortCalib))
    {
		iEncoders->getEncoder(j, &ang);
		delta = fabs(ang-pos[j]);
		fprintf(logfile, "ARMCALIB (joint %d) curr:%f des:%f -> delta:%f\n", j, ang, pos[j], delta);
		if (delta<POSITION_THRESHOLD)
		{
			fprintf(logfile, "ARMCALIB (joint %d) completed! delta:%f\n", j,delta);
			finished=true;
		}

        Time::delay (0.5);
        timeout ++;

        if (timeout >= GO_TO_ZERO_TIMEOUT)
        {
            fprintf(logfile, "ARMCALIB::Timeout on joint %d while going to zero!\n", j);
			return false;
        }
    }
    if (abortCalib)
        fprintf(logfile, "ARMCALIB::abort wait for joint %d going to zero!\n", j);

	return finished;
}

bool iCubArmCalibrator::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    bool ret=false;
    abortParking=false;

    ret=iEncoders->getAxes(&nj);
    if (!ret)
        {
            fprintf(logfile, "ARMCALIB: error getting number of encoders\n");
            return false;
        }

	int timeout = 0;
    fprintf(logfile, "ARMCALIB::Calling iCubArmCalibrator::park() \n");
	iPosition->setPositionMode();
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
    {
        fprintf(logfile, "ARMCALIB::moving to park positions \n");
        bool done=false;
        while((!done) && (timeout<PARK_TIMEOUT) && (!abortParking))
        {
            iPosition->checkMotionDone(&done);
            fprintf(logfile, ".");
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
						fprintf(logfile, "iCubArmCalibrator::park(): joint %d not in position ", j);
				}
				else
					fprintf(logfile, "iCubArmCalibrator::park(): joint %d did not answer ", j);
			}
		}
    }

    if (abortParking)
        fprintf(logfile, "ARMCALIB::park was aborted!\n");
    else
        fprintf(logfile, "ARMCALIB::park was done!\n");

    return true;
}

bool iCubArmCalibrator::quitCalibrate()
{
    fprintf(logfile, "ARMCALIB::quitting calibrate\n");
    abortCalib=true;
    return true;
}

bool iCubArmCalibrator::quitPark()
{
    fprintf(logfile, "ARMCALIB::quitting parking\n");
    abortParking=true;
    return true;
}
