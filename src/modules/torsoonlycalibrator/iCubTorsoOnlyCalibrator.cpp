// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2009 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <ace/config.h>
#include <ace/OS.h>
#include <ace/Log_Msg.h>

#include <yarp/os/Time.h>

#include "iCubTorsoOnlyCalibrator.h"

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the armn of the head iCub

const int GO_TO_ZERO_TIMEOUT=20;
const int PARK_TIMEOUT=30;
const int CALIBRATE_TIMEOUT=20;

iCubTorsoOnlyCalibrator::iCubTorsoOnlyCalibrator()
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

iCubTorsoOnlyCalibrator::~iCubTorsoOnlyCalibrator()
{
    //empty now
}

bool iCubTorsoOnlyCalibrator::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    bool ret=false;
    abortParking=false;
    ret=iEncoders->getAxes(&nj);
    if (!ret)
        {
            fprintf(stderr, "TORSOCALIB: error getting number of encoders\n");
            return false;
        }

	int timeout = 0;
    fprintf(stderr, "TORSOCALIB::Calling iCubTorsoOnlyCalibrator::park()");
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
        {
            bool done=false;
            while((!done) && (timeout<PARK_TIMEOUT) && (!abortParking))
            {
                iPosition->checkMotionDone(1, &done);
                iPosition->checkMotionDone(2, &done);
                fprintf(stderr, "."); 
                Time::delay(1);
				timeout++;
            }
			if(!done)
			{
				for(int j=1; j < nj; j++)                      //joint 0 is not enabled during calibration and parking
				{
					iPosition->checkMotionDone(j, &done);
					if (iPosition->checkMotionDone(j, &done))
					{
						if (!done)
							fprintf(stderr, "iCubTorsoOnlyCalibrator::park() : joint %d not in position ", j);
					}
					else
						fprintf(stderr, "iCubTorsoOnlyCalibrator::park() : joint %d did not answer ", j);
				}
			}
        }

    if (abortParking)
        fprintf(stderr, "Torso parking was aborted!\n");
    else
        fprintf(stderr, "Torso parking was done\n");

    return true;
}

bool iCubTorsoOnlyCalibrator::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(stderr, "TORSOCALIB::Cannot understand configuration parameters\n");
        return false;
    }

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
    type = new unsigned char[nj];
    ACE_ASSERT (type != NULL);
    param1 = new double[nj];
    ACE_ASSERT (param1 != NULL);
    param2 = new double[nj];
    ACE_ASSERT (param2 != NULL);
    param3 = new double[nj];
    ACE_ASSERT (param3 != NULL);

    pos = new double[nj];
    ACE_ASSERT (pos != NULL);
    vel = new double[nj];
    ACE_ASSERT (vel != NULL);

    homeVel=new double[nj];
    homePos=new double[nj];

    Bottle& xtmp = p.findGroup("CALIBRATION").findGroup("Calibration1");
    ACE_ASSERT (xtmp.size() == nj+1);
    int i;
    for (i = 1; i < xtmp.size(); i++)
        param1[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("CALIBRATION").findGroup("Calibration2");
    ACE_ASSERT (xtmp.size() == nj+1);
    for (i = 1; i < xtmp.size(); i++)
        param2[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("CALIBRATION").findGroup("Calibration3");
    ACE_ASSERT (xtmp.size() == nj+1);
    for (i = 1; i < xtmp.size(); i++)
        param3[i-1] = xtmp.get(i).asDouble();
    xtmp = p.findGroup("CALIBRATION").findGroup("CalibrationType");
    ACE_ASSERT (xtmp.size() == nj+1);
    for (i = 1; i < xtmp.size(); i++)
        type[i-1] = (unsigned char) xtmp.get(i).asDouble();


    xtmp = p.findGroup("CALIBRATION").findGroup("PositionZero");
    ACE_ASSERT (xtmp.size() == nj+1);
    for (i = 1; i < xtmp.size(); i++)
        pos[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("VelocityZero");
    ACE_ASSERT (xtmp.size() == nj+1);
    for (i = 1; i < xtmp.size(); i++)
        vel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("VelocityHome");
    ACE_ASSERT (xtmp.size() == nj+1);
    for (i = 1; i < xtmp.size(); i++)
        homeVel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("PositionHome");
    ACE_ASSERT (xtmp.size() == nj+1);
    for (i = 1; i < xtmp.size(); i++)
        homePos[i-1] = xtmp.get(i).asDouble();

    return true;
}

bool iCubTorsoOnlyCalibrator::close ()
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

bool iCubTorsoOnlyCalibrator::calibrate(DeviceDriver *dd)
{
    fprintf(stderr, "HEAD::CALIB Calling iCubTorsoOnlyCalibrator::calibrate\n");
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
	//calibrate the joint 1 and 2      //
    /////////////////////////////////////
	int firstSetOfJoints[] = {1,2};       //joint 0 is not enabled during calibration and parking
    for (k = 0; k < 2; k++) 
    {
        if (firstSetOfJoints[k]<nj)
            {
                iAmps->enableAmp(firstSetOfJoints[k]);
                iPids->enablePid(firstSetOfJoints[k]);
            }
    }
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
    return ret;
}

void iCubTorsoOnlyCalibrator::calibrateJoint(int joint)
{
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);

}

bool iCubTorsoOnlyCalibrator::checkCalibrateJointEnded(int joint)
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
        fprintf(stderr, "TORSOCALIB::Timeout on joint %d while calibrating!\n", joint);
        return false;
    }
    if (abortCalib)
    {
        fprintf(stderr, "TORSOCALIB::aborting calibration of %d\n", joint);
        return false;
    }

    return true;
}


void iCubTorsoOnlyCalibrator::goToZero(int j)
{
    iPosition->setRefSpeed(j, vel[j]);
    iPosition->positionMove(j, pos[j]);
}

void iCubTorsoOnlyCalibrator::checkGoneToZero(int j)
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
            fprintf(stderr, "TORSOCALIB::Timeout on joint %d while going to zero!\n", j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(stderr, "TORSOCALIB::Aborted wait for joint %d\n", j);
}

bool iCubTorsoOnlyCalibrator::quitCalibrate()
{
    fprintf(stderr, "TORSOCALIB::quitting calibrate\n");
    abortCalib=true;
    return true;
}

bool iCubTorsoOnlyCalibrator::quitPark()
{
    fprintf(stderr, "TORSOCALIB::quitting parking\n");
    abortParking=true;
    return true;
}

