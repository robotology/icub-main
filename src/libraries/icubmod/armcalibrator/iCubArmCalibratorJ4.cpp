// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubArmCalibratorJ4.h"
#include <math.h>

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the arm of the Arm iCub

const int PARK_TIMEOUT=30;
const double GO_TO_ZERO_TIMEOUT=10; //seconds
const int CALIBRATE_JOINT_TIMEOUT=25;
const double POSITION_THRESHOLD=2.0;

const int numberOfJoints=4;

iCubArmCalibratorJ4::iCubArmCalibratorJ4()
{
	logfile = stderr;
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

iCubArmCalibratorJ4::~iCubArmCalibratorJ4()
{
    //empty now
}

bool iCubArmCalibratorJ4::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(logfile, "ARMCALIB::Cannot understand configuration parameters\n");
        return false;
    }

	canID =  p.findGroup("CAN").find("CanDeviceNum").asInt();

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
    if (nj!=numberOfJoints)
        {
            fprintf(logfile, "ARMCALIB[%d]: Calibrator is for %d joints but device has %d\n", canID, numberOfJoints, nj);
            return false;
        }
        
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
	   fprintf(logfile, "ARMCALIB[%d] :MaxPWM parameter not found, assuming 60\n", canID);
	   for (i = 1; i < nj+1; i++) maxPWM[i-1] = 60;
   }

    return true;
}

bool iCubArmCalibratorJ4::close ()
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

    if (logfile_name!="") fclose(logfile);

    return true;
}

bool iCubArmCalibratorJ4::calibrate(DeviceDriver *dd)
{
    fprintf(logfile, "Calling iCubArmCalibratorJ8::calibrate \n");
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

    if (nj!=numberOfJoints)
        {
			fprintf(logfile, "ARMCALIB[%d]: Calibrator is for %d joints but device has %d\n", canID, numberOfJoints, nj);
            return false;
        }

    if (!ret)
        return false;

	fprintf(stderr, "ARMCALIB::start! new calibrator for torque controlled shoulder by randaz \n");

	original_pid=new Pid[nj];
	limited_pid =new Pid[nj];
	bool calibration_ok=true;
    int k;
	for (k =0; k < nj; k++)
		calibrateJoint(k);
	Time::delay(1.0);

    for (k = 0; k < nj; k++) 
    {
		iPids->getPid(k,&original_pid[k]);
		limited_pid[k]=original_pid[k];
		limited_pid[k].max_int=maxPWM[k];
		limited_pid[k].max_output=maxPWM[k];
		iPids->setPid(k,limited_pid[k]);

        fprintf(logfile, "ARMCALIB[%d]: Calling enable amp for joint %d\n", canID, k);
        iAmps->enableAmp(k);
        fprintf(logfile, "ARMCALIB[%d]: Calling enable pid for joint %d\n", canID, k);
        iPids->enablePid(k);
    }
	////////////////////////////////////////////
	for (k = 0; k < nj; k++)
		goToZero(k);
	for (k = 0; k < nj; k++)
		calibration_ok &= checkGoneToZeroThreshold(k);
	//////////////////////////////////////////
	if (calibration_ok)
	{
		fprintf(stderr, "ARMCALIB::calibration done!\n");
		for (k = 0; k < nj; k++)
			iPids->setPid(k,original_pid[k]);
	}
	else
	{
		fprintf(stderr, "ARMCALIB::calibration failed!\n");
		for (k = 0; k < nj; k++)
			iAmps->disableAmp(k);
	}

    return ret;
}

void iCubArmCalibratorJ4::calibrateJoint(int joint)
{
	fprintf(logfile, "ARMCALIB[%d]: Calling calibrateJoint on joint %d with params: %d  %+6.1f %+6.1f %+6.1f\n", canID, joint, type[joint], param1[joint], param2[joint], param3[joint]);
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool iCubArmCalibratorJ4::checkCalibrateJointEnded(int joint)
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
        fprintf(logfile, "ARMCALIB[%d]: Timeout on joint %d while calibrating!\n", canID, joint);
        return false;
    }
    if (abortCalib)
    {
        fprintf(logfile, "ARMCALIB[%d]: aborted\n", canID);
    }

    return true;
}


void iCubArmCalibratorJ4::goToZero(int j)
{
    if (abortCalib)
        return;
	iControlMode->setPositionMode(j);
    iPosition->setRefSpeed(j, zeroVel[j]);
    iPosition->positionMove(j, zeroPos[j]);
}

void iCubArmCalibratorJ4::checkGoneToZero(int j)
{
    // wait.
    bool finished = false;
    double start_time = yarp::os::Time::now();
   
    while ( (!finished) && (!abortCalib))
    {
        iPosition->checkMotionDone(j, &finished);

        if (logfile_name != "")
            Time::delay (0.1);
        else
        Time::delay (0.5);
        
        if (yarp::os::Time::now() - start_time > GO_TO_ZERO_TIMEOUT)
        {
            fprintf(logfile, "ARMCALIB[%d]: Timeout on joint %d while going to zero!\n", canID, j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(logfile, "ARMCALIB[%d]: abort wait for joint %d going to zero!\n", canID, j);
}

bool iCubArmCalibratorJ4::checkGoneToZeroThreshold(int j)
{
    // wait.
    bool finished = false;
	double ang=0;
	double delta=0;

    double start_time = yarp::os::Time::now();
    while ( (!finished) && (!abortCalib))
    {
		iEncoders->getEncoder(j, &ang);
		delta = fabs(ang-zeroPos[j]);
		fprintf(logfile, "ARMCALIB[%d] (joint %d) curr:%.2f des:%.2f -> delta:%.2f\n", canID, j, ang, zeroPos[j], delta);
		if (delta<POSITION_THRESHOLD) 
		{
			fprintf(logfile, "ARMCALIB[%d] (joint %d) completed! delta:%f\n", canID, j,delta);
			finished=true;
		}

        Time::delay (0.5);

        if (yarp::os::Time::now() - start_time > GO_TO_ZERO_TIMEOUT)
        {
            fprintf(logfile, "ARMCALIB[%d]: Timeout on joint %d while going to zero!\n", canID, j);
			return false;
        }
    }
    if (abortCalib)
        fprintf(logfile, "ARMCALIB[%d]: Abort wait for joint %d going to zero!\n", canID, j);

	return finished;
}

bool iCubArmCalibratorJ4::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    bool ret=false;
    abortParking=false;

    ret=iEncoders->getAxes(&nj);
    if (!ret)
        {
            fprintf(logfile, "ARMCALIB[%d]: error getting number of encoders\n",canID);
            return false;
        }

    if (nj!=numberOfJoints)
        {
            fprintf(logfile, "ARMCALIB[%d]: calibrator is for %d joints but device has %d\n", canID, numberOfJoints, nj);
            return false;
        }

	int timeout = 0;
    fprintf(logfile, "ARMCALIB[%d]: Calling iCubArmCalibratorJ8::park() \n",canID);
	iPosition->setPositionMode();
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
    {
        fprintf(logfile, "ARMCALIB[%d]: Moving to park positions \n",canID);
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
						fprintf(logfile, "ARMCALIB[%d]: joint %d not in position during park()\n",canID, j);
				}
				else
					fprintf(logfile, "ARMCALIB[%d]: joint %d did not answer during park() \n", canID, j);
			}
		}
    }

    if (abortParking)
        fprintf(logfile, "ARMCALIB[%d]: Park was aborted!\n", canID);
    else
        fprintf(logfile, "ARMCALIB[%d]: Park was done!\n", canID);

    return true;
}

bool iCubArmCalibratorJ4::quitCalibrate()
{
    fprintf(logfile, "ARMCALIB[%d]: Quitting calibrate\n", canID);
    abortCalib=true;
    return true;
}

bool iCubArmCalibratorJ4::quitPark()
{
    fprintf(logfile, "ARMCALIB[%d]: Quitting parking\n", canID);
    abortParking=true;
    return true;
}
