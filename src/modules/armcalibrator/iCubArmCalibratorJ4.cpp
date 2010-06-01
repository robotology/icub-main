// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubArmCalibratorJ4.h"

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the arm of the Arm iCub

const int PARK_TIMEOUT=30;
const int GO_TO_ZERO_TIMEOUT=20;
const int CALIBRATE_JOINT_TIMEOUT=20;

const int numberOfJoints=4;

iCubArmCalibratorJ4::iCubArmCalibratorJ4()
{
    type   = NULL;
    param1 = NULL;
    param2 = NULL;
    param3 = NULL;
    pos = NULL;
    vel = NULL;
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
        fprintf(stderr, "ARMCALIB::Cannot understand configuration parameters\n");
        return false;
    }

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
    if (nj!=numberOfJoints)
        {
            fprintf(stderr, "ARMCALIB::calibrator is for %d joints but device has %d\n", numberOfJoints, nj);
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

    if (pos != NULL) delete[] pos;
    pos = NULL;
    if (vel != NULL) delete[] vel;
    vel = NULL;

    if (homePos != NULL) delete[] homePos;
    homePos = NULL;
    if (homeVel != NULL) delete[] homeVel;
    homeVel = NULL;

    return true;
}

bool iCubArmCalibratorJ4::calibrate(DeviceDriver *dd)
{
    fprintf(stderr, "Calling iCubArmCalibratorJ4::calibrate: \n");
    abortCalib=false;

    iCalibrate = dynamic_cast<IControlCalibration2 *>(dd);
    iAmps =  dynamic_cast<IAmplifierControl *>(dd);
    iEncoders = dynamic_cast<IEncoders *>(dd);
    iPosition = dynamic_cast<IPositionControl *>(dd);
    iPids = dynamic_cast<IPidControl *>(dd);

    if (!(iCalibrate&&iAmps&&iPosition&&iPids))
        return false;

    // ok we have all interfaces
    int nj=0;
    bool ret=iEncoders->getAxes(&nj);

    if (nj!=numberOfJoints)
        {
            fprintf(stderr, "ARMCALIB::calibrator is for %d joints but device has %d\n", numberOfJoints, nj);
            return false;
        }

    if (!ret)
        return false;

	fprintf(stderr, "ARMCALIB::start! new calibrator for torque controlled shoulder by randaz \n");

    int k;
	for (k =0; k < nj; k++)
		calibrateJoint(k);
	Time::delay(1.0);

    for (k = 0; k < nj; k++) 
    {
        fprintf(stderr, "ARMCALIB::Calling enable amp for joint %d\n", k);
        iAmps->enableAmp(k);
        fprintf(stderr, "ARMCALIB::Calling enable pid for joint %d\n", k);
        iPids->enablePid(k);
    }
	////////////////////////////////////////////
	for (k = 0; k < nj; k++)
		goToZero(k);
	for (k = 0; k < nj; k++)
		checkGoneToZero(k);
	//////////////////////////////////////////
    fprintf(stderr, "ARMCALIB::calibration done!\n");
    return ret;
}

void iCubArmCalibratorJ4::calibrateJoint(int joint)
{
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
        fprintf(stderr, "ARMCALIB::Timeout on joint %d while calibrating!\n", joint);
        return false;
    }
    if (abortCalib)
    {
        fprintf(stderr, "ARMCALIB::aborted\n");
    }

    return true;
}


void iCubArmCalibratorJ4::goToZero(int j)
{
    if (abortCalib)
        return;
    iPosition->setRefSpeed(j, vel[j]);
    iPosition->positionMove(j, pos[j]);
}

void iCubArmCalibratorJ4::checkGoneToZero(int j)
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
            fprintf(stderr, "ARMCALIB::Timeout on joint %d while going to zero!\n", j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(stderr, "ARMCALIB::abort wait for joint %d going to zero!\n", j);
}

bool iCubArmCalibratorJ4::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    bool ret=false;
    abortParking=false;

    ret=iEncoders->getAxes(&nj);
    if (!ret)
        {
            fprintf(stderr, "ARMCALIB: error getting number of encoders\n");
            return false;
        }

    if (nj!=numberOfJoints)
        {
            fprintf(stderr, "ARMCALIB::calibrator is for %d joints but device has %d\n", numberOfJoints, nj);
            return false;
        }

	int timeout = 0;
    fprintf(stderr, "ARMCALIB::Calling iCubArmCalibratorJ4::park()");
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
    {
        fprintf(stderr, "ARMCALIB::moving to park positions \n");
        bool done=false;
        while((!done) && (timeout<PARK_TIMEOUT) && (!abortParking))
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
						fprintf(stderr, "iCubArmCalibratorJ4::park(): joint %d not in position ", j);
				}
				else
					fprintf(stderr, "iCubArmCalibratorJ4::park(): joint %d did not answer ", j);
			}
		}
    }

    if (abortParking)
        fprintf(stderr, "ARMCALIB::park was aborted!\n");
    else
        fprintf(stderr, "ARMCALIB::park was done!\n");

    return true;
}

bool iCubArmCalibratorJ4::quitCalibrate()
{
    fprintf(stderr, "ARMCALIB::quitting calibrate\n");
    abortCalib=true;
    return true;
}

bool iCubArmCalibratorJ4::quitPark()
{
    fprintf(stderr, "ARMCALIB::quitting parking\n");
    abortParking=true;
    return true;
}
