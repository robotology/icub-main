// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubHeadCalibrator.h"

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the armn of the head iCub

const int GO_TO_ZERO_TIMEOUT=20;
const int PARK_TIMEOUT=30;
const int CALIBRATE_TIMEOUT=20;

iCubHeadCalibrator::iCubHeadCalibrator()
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

iCubHeadCalibrator::~iCubHeadCalibrator()
{
    //empty now
}

bool iCubHeadCalibrator::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    bool ret=false;
    abortParking=false;
    ret=iEncoders->getAxes(&nj);
    if (!ret)
        {
            fprintf(stderr, "HEADCALIB: error getting number of encoders\n");
            return false;
        }

	int timeout = 0;
    fprintf(stderr, "HEADCALIB::Calling iCubHeadCalibrator::park()");
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
        {
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
							fprintf(stderr, "iCubHeadCalibrator::park() : joint %d not in position ", j);
					}
					else
						fprintf(stderr, "iCubHeadCalibrator::park() : joint %d did not answer ", j);
				}
			}
        }

    if (abortParking)
        fprintf(stderr, "Head parking was aborted!\n");
    else
        fprintf(stderr, "Head parking was done\n");

    return true;
}

bool iCubHeadCalibrator::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(stderr, "HEADCALIB::Cannot understand configuration parameters\n");
        return false;
    }

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
    type = new unsigned char[nj];

    param1 = new double[nj];
    param2 = new double[nj];
    param3 = new double[nj];

    pos = new double[nj];
    vel = new double[nj];

    homeVel=new double[nj];
    homePos=new double[nj];

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

    xtmp = p.findGroup("HOME").findGroup("VelocityHome");

    for (i = 1; i < xtmp.size(); i++)
        homeVel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("PositionHome");

    for (i = 1; i < xtmp.size(); i++)
        homePos[i-1] = xtmp.get(i).asDouble();

    return true;
}

bool iCubHeadCalibrator::close ()
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

bool iCubHeadCalibrator::calibrate(DeviceDriver *dd)
{
    fprintf(stderr, "HEAD::CALIB Calling iCubHeadCalibrator::calibrate\n");
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

    if (!ret)
        return false;

    int k;
    for(k=0;k<nj;k++)
    {
        iEncoders->resetEncoder(k);
    }

	bool x;

    /////////////////////////////////////
	//enable all joints                //
    /////////////////////////////////////

    for (k = 0; k < nj; k++) 
    {
        iAmps->enableAmp(k);
        iPids->enablePid(k);
    }
    /////////////////////////////////////
	//calibrate the first set of joints//
    /////////////////////////////////////
	int firstSetOfJoints[] = {0, 2, 4, 6, 7, 8}; //and joint 5
	int secondSetOfJoints[] = {1, 3};
	for (k =0; k < 6; k++)
        if (firstSetOfJoints[k]<nj)
            calibrateJoint(firstSetOfJoints[k]);
	for (k =0; k < 6; k++)
    {
        if (firstSetOfJoints[k]<nj)
            {
                x = checkCalibrateJointEnded(firstSetOfJoints[k]);
                ret = ret && x;
            }
	}
	if(5<nj)
    {
        calibrateJoint(5);
        x = checkCalibrateJointEnded(5);
        ret = ret && x;
    }

	for (k = 0; k < 6; k++)
        if (firstSetOfJoints[k]<nj)
            goToZero(firstSetOfJoints[k]);
	for (k = 0; k < 6; k++)
        if (firstSetOfJoints[k]<nj)
            checkGoneToZero(firstSetOfJoints[k]);
    goToZero(5);
    checkGoneToZero(5);

    /////////////////////////////////////
    //calibrate the second set of joint//
    /////////////////////////////////////
	for (k =0; k < 2; k++)
        if (secondSetOfJoints[k]<nj)
            calibrateJoint(secondSetOfJoints[k]);
	for (k =0; k < 2; k++)
    {
        if (secondSetOfJoints[k]<nj)
            {
                x = checkCalibrateJointEnded(secondSetOfJoints[k]);
                ret = ret && x;
            }
	}

	for (k = 0; k < 2; k++)
        if (secondSetOfJoints[k]<nj)
            goToZero(secondSetOfJoints[k]);
	for (k = 0; k < 2; k++)
        if (secondSetOfJoints[k]<nj)
            checkGoneToZero(secondSetOfJoints[k]);


    return ret;
}

void iCubHeadCalibrator::calibrateJoint(int joint)
{
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);

}

bool iCubHeadCalibrator::checkCalibrateJointEnded(int joint)
{
    const int timeout = CALIBRATE_TIMEOUT;
    int i;
    for (i = 0; i < timeout; i++)
    {
        fprintf(stderr, ".");
        if (iCalibrate->done(joint))
            break;
        if (abortCalib)
            break;
        Time::delay(1.0);
    }
    if (i == timeout)
    {
        fprintf(stderr, "HEADCALIB::Timeout on joint %d while calibrating!\n", joint);
        return false;
    }
    if (abortCalib)
    {
        fprintf(stderr, "HEADCALIB::aborting calibration of %d\n", joint);
        return false;
    }

    return true;
}


void iCubHeadCalibrator::goToZero(int j)
{
    iPosition->setRefSpeed(j, vel[j]);
    iPosition->positionMove(j, pos[j]);
}

void iCubHeadCalibrator::checkGoneToZero(int j)
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
            fprintf(stderr, "HEADCALIB::Timeout on joint %d while going to zero!\n", j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(stderr, "HEADCALIB::Aborted wait for joint %d\n", j);
}

bool iCubHeadCalibrator::quitCalibrate()
{
    fprintf(stderr, "HEADCALIB::quitting calibrate\n");
    abortCalib=true;
    return true;
}

bool iCubHeadCalibrator::quitPark()
{
    fprintf(stderr, "HEADCALIB::quitting parking\n");
    abortParking=true;
    return true;
}

