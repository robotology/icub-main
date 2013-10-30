// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubHeadCalibrator.h"
#include <math.h>
#include <string>
#include <algorithm>

using namespace yarp::os;
using namespace yarp::dev;

const int PARK_TIMEOUT=30;
const int GO_TO_ZERO_TIMEOUT=20;
const int CALIBRATE_JOINT_TIMEOUT=25;
const double POSITION_THRESHOLD=2.0;

#define TORSO_IS_AVAILABLE (nj > 6)

iCubHeadCalibrator::iCubHeadCalibrator()
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

iCubHeadCalibrator::~iCubHeadCalibrator()
{
    //empty now
}



bool readCalibparam (Property p, std::string groupstring, std::string paramstring, double variable [], int nj)
{
    Bottle b;
    Bottle& xtmp = b;
    xtmp = p.findGroup(groupstring).findGroup(paramstring);
    if (xtmp.isNull()==true)
    {
        std::transform(groupstring.begin(), groupstring.end(), groupstring.begin(), ::tolower);
        std::transform(paramstring.begin(), paramstring.end(), paramstring.begin(), ::tolower);
        xtmp = p.findGroup(groupstring).findGroup(paramstring);
    }
    int joints = xtmp.size()<nj ? xtmp.size():nj;
    for (int i=0; i<joints; i++)
    {
         variable[i] = xtmp.get(i+1).asDouble();
    }
    return true;
}

bool iCubHeadCalibrator::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(logfile, "HEADCALIB::Cannot understand configuration parameters\n");
        return false;
    }

    canID =  p.findGroup("CAN").find("CanDeviceNum").asInt();

    int nj = p.findGroup("GENERAL").find("Joints").asInt();
    dtype = new double[nj];
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
    homeVel=new double[nj];

    readCalibparam(p, "CALIBRATION", "Calibration1", param1, nj);
    readCalibparam(p, "CALIBRATION", "Calibration2", param2, nj);
    readCalibparam(p, "CALIBRATION", "Calibration3", param3, nj);
    readCalibparam(p, "CALIBRATION", "CalibrationType", dtype, nj); for (int i=0; i<nj; i++) type[i]=(unsigned char)(dtype[i]);
    readCalibparam(p, "CALIBRATION", "PositionZero", zeroPos, nj);
    readCalibparam(p, "CALIBRATION", "VelocityZero", zeroVel, nj);
    readCalibparam(p, "HOME", "PositionHome", homePos, nj);
    readCalibparam(p, "HOME", "VelocityHome", homeVel, nj);
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

    /*
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

    if (p.findGroup("CALIBRATION").check("maxPWM"))
    {
        xtmp = p.findGroup("CALIBRATION").findGroup("maxPWM");
        for (i = 1; i < xtmp.size(); i++) maxPWM[i-1] =  xtmp.get(i).asInt();
    }
    else
    {
        fprintf(logfile, "HEADCALIB[%d] :MaxPWM parameter not found, using default values\n", canID);
        for (i = 0; i < 3; i++) maxPWM[i] = 250;    //head
        for (i = 3; i < 6; i++) maxPWM[i] = 1333;   //eyes do not use maxPWM
        if (TORSO_IS_AVAILABLE)
        {
            for (i = 6; i < 9; i++) maxPWM[i] = 200;    //torso
        }
    }
    */

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

bool iCubHeadCalibrator::calibrate(DeviceDriver *dd)
{
    fprintf(logfile, "Calling iCubHeadCalibratorV2::calibrate\n");
    abortCalib=false;

    dd->view(iCalibrate);
    dd->view(iAmps);
    dd->view(iEncoders);
    dd->view(iPosition);
    dd->view(iPids);
    dd->view(iControlMode);

    if (!(iCalibrate && iAmps && iEncoders && iPosition && iPids && iControlMode)) {
        fprintf(logfile, "HEADCALIB[%d]: Error. This device cannot be calibrated\n", canID);
        return false;
    }

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
	//calibrate the torso              //
    /////////////////////////////////////

	//BLL boards must receive calibration message before enabling PWM
	for (k = 6; k < nj; k++) 
    {
		//bll boards are joint 6 7 8 (9)
        calibrateJoint(k);
	}
    
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
            calibrateJoint(firstSetOfJoints[k]); //LATER: calibrateJoint will be removed after enableAmp
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
            calibrateJoint(secondSetOfJoints[k]); //LATER: calibrateJoint will be removed after enableAmp
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
    fprintf(logfile, "HEADCALIB[%d]: Calling calibrateJoint on joint %d with params: %d  %+6.1f %+6.1f %+6.1f\n", canID, joint, type[joint], param1[joint], param2[joint], param3[joint]);
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool iCubHeadCalibrator::checkCalibrateJointEnded(int joint)
{
    const int timeout = CALIBRATE_JOINT_TIMEOUT;
    int i;
    for (i = 0; i < timeout; i++)
    {
        fprintf(logfile, ".");
        if (iCalibrate->done(joint))
            break;
        if (abortCalib)
            break;
        Time::delay(1.0);
    }
    if (i == timeout)
    {
        fprintf(logfile, "HEADCALIB[%d]: Timeout on joint %d while calibrating!\n", canID, joint);
        return false;
    }
    else if (abortCalib)
    {
        fprintf(logfile, "HEADCALIB[%d]: Aborting calibration of %d\n", canID, joint);
        return false;
    }

    fprintf(logfile, "HEADCALIB[%d]: Calibration of joint %d done\n", canID, joint);
    return true;
}


void iCubHeadCalibrator::goToZero(int j)
{
    if (abortCalib)
        return;
	iControlMode->setPositionMode(j);
    iPosition->setRefSpeed(j, zeroVel[j]);
    iPosition->positionMove(j, zeroPos[j]);
}

void iCubHeadCalibrator::checkGoneToZero(int j)
{
    bool finished = false;
    int timeout = 0;
    while ( (!finished) && (!abortCalib))
    {
        iPosition->checkMotionDone(j, &finished);

        Time::delay (0.5);
        timeout ++;
        if (timeout >= GO_TO_ZERO_TIMEOUT)
        {
            fprintf(logfile, "HEADCALIB[%d]: Timeout on joint %d while going to zero!\n", canID, j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(logfile, "HEADCALIB[%d]: Abort wait for joint %d going to zero!\n", canID, j);
}

bool iCubHeadCalibrator::checkGoneToZeroThreshold(int j)
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
        fprintf(logfile, "HEADCALIB[%d]: (joint %d) curr:%.2f des:%.2f -> delta:%.2f\n", canID, j, ang, zeroPos[j], delta);
        if (delta<POSITION_THRESHOLD)
        {
            fprintf(logfile, "HEADCALIB[%d]: (joint %d) completed! delta:%f\n", canID, j,delta);
            finished=true;
        }

        Time::delay (0.5);
        timeout ++;

        if (timeout >= GO_TO_ZERO_TIMEOUT)
        {
            fprintf(logfile, "HEADCALIB[%d]: Timeout on joint %d while going to zero!\n", canID, j);
            return false;
        }
    }
    if (abortCalib)
        fprintf(logfile, "HEADCALIB[%d]: Abort wait for joint %d going to zero!\n", canID, j);

    return finished;
}

bool iCubHeadCalibrator::park(DeviceDriver *dd, bool wait)
{
	int nj=0;
    abortParking=false;

    if (!iEncoders || !iEncoders->getAxes(&nj))
        {
            fprintf(logfile, "HEADCALIB[%d]: Error getting number of encoders\n", canID);
            return false;
        }

	int timeout = 0;
    fprintf(logfile, "HEADCALIB[%d]: Calling iCubHeadCalibrator::park() \n", canID);
	iPosition->setPositionMode();
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
        {
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
                        fprintf(logfile, "HEADCALIB[%d]: iCubHeadCalibrator::park() : joint %d not in position ", canID, j);
					}
					else
                    fprintf(logfile, "HEADCALIB[%d]: iCubHeadCalibrator::park() : joint %d did not answer ", canID, j);
				}
			}
        }

    if (abortParking)
        fprintf(logfile, "HEADCALIB[%d]: Park was aborted!\n", canID);
    else
        fprintf(logfile, "HEADCALIB[%d]: Park was done!\n", canID);
    return true;
}

bool iCubHeadCalibrator::quitCalibrate()
{
    fprintf(logfile, "HEADCALIB[%d]: Quitting calibrate\n", canID);
    abortCalib=true;
    return true;
}

bool iCubHeadCalibrator::quitPark()
{
    fprintf(logfile, "HEADCALIB[%d]: Quitting park\n", canID);
    abortParking=true;
    return true;
}
