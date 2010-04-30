// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubLegsCalibrator.h"

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the legs of the Legs iCub
const int PARK_TIMEOUT=30;
const int CALIBRATION_TIMEOUT=20;
const int GO_TO_ZERO_TIMEOUT=20;

iCubLegsCalibrator::iCubLegsCalibrator()
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

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
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

bool iCubLegsCalibrator::calibrate(DeviceDriver *dd)
{
    fprintf(stderr, "Calling iCubLegsCalibrator::calibrate!\n");
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

    for(k = 0; k < nj; k++) 
    {
        iAmps->enableAmp(k);
		iPids->enablePid(k);
    }

    ret = true;
    bool x;
	int j;
		for (j=0; j < nj; j++)
			calibrateJoint(j);

		for (j =0; j < nj; j++)
		{
			x = checkCalibrateJointEnded(j);
			ret = ret && x;
		}

		for (j =0; j < nj; j++)
			goToZero(j);	
		
		for (j =0; j < nj; j++)
			checkGoneToZero(j);

    return ret;
}

void iCubLegsCalibrator::calibrateJoint(int joint)
{
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);

}

bool iCubLegsCalibrator::checkCalibrateJointEnded(int joint)
{
    const int timeout = CALIBRATION_TIMEOUT;
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
    iPosition->setRefSpeed(j, vel[j]);
    iPosition->positionMove(j, pos[j]);
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
            fprintf(stderr, "LEGSCALIB::Timeout on joint %d while going to zero!\n", j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(stderr, "LEGSCALIB::Aborted wait for joint %d\n", j);
}

bool iCubLegsCalibrator::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    bool ret=false;
    abortParking=false;

    ret=iEncoders->getAxes(&nj);
    if (!ret)
    {
       fprintf(stderr, "LEGSCALIB: error getting number of encoders\n");
       return false;
    }

    fprintf(stderr, "LEGSCALIB::Calling iCubLegsCalibrator::park()");

    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    int timeout = 0;
    if (wait)
    {
        bool done=false;
        while( (!done) && (timeout < PARK_TIMEOUT) && (!abortParking))
        {
            iPosition->checkMotionDone(&done);
            Time::delay(1);
            fprintf(stderr, ".");
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
						fprintf(stderr, "LEGSCALIB::park() : joint %d not in position\n", j);
				}
				else
					fprintf(stderr, "LEGSCALIB::park() : joint %d did not answer\n", j);
			}
		}
    }

    if (abortParking)
	    fprintf(stderr, "LEGSCALIB::park() was aborted\n");
    else
        fprintf(stderr, "LEGSCALIB::park() done!\n");
    return true;
}

bool iCubLegsCalibrator::quitCalibrate()
{
    fprintf(stderr, "LEGSCALIB::quitting calibrate\n");
    abortCalib=true;
    return true;
}

bool iCubLegsCalibrator::quitPark()
{
    fprintf(stderr, "LEGSCALIB::quitting park\n");
    abortParking=true;
    return true;
}
