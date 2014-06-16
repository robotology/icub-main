// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2009 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubTorsoOnlyCalibrator.h"
#include <math.h>

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the armn of the head iCub

const int PARK_TIMEOUT=30;
const int GO_TO_ZERO_TIMEOUT=20;
const int CALIBRATE_JOINT_TIMEOUT=25;
const double POSITION_THRESHOLD=2.0;

iCubTorsoOnlyCalibrator::iCubTorsoOnlyCalibrator()
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

iCubTorsoOnlyCalibrator::~iCubTorsoOnlyCalibrator()
{
    //empty now
}

bool iCubTorsoOnlyCalibrator::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(logfile, "TORSOCALIB::Cannot understand configuration parameters\n");
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
    homeVel=new double[nj];

    logfile_name = p.findGroup("CALIBRATION").find("Logfile").asString();
    if (logfile_name != "")
    {
        fprintf(stdout, "TORSOCALIB::calibrator: opening logfile %s\n", logfile_name.c_str());
        logfile = fopen (logfile_name.c_str(), "w");
    }
    else
    {
        fprintf(stdout, "TORSOCALIB::calibrator: no logfile specified, errors displayed on standard stderr\n");
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
        fprintf(logfile, "TORSOCALIB[%d] :MaxPWM parameter not found, using default values\n", canID);
        for (i = 0; i < 3; i++) maxPWM[i] = 200;    //torso
    }

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

bool iCubTorsoOnlyCalibrator::calibrate(DeviceDriver *dd)
{
    fprintf(logfile, "TORSOCALIB[%d]: Calling iCubTorsoOnlyCalibrator::calibrate\n", canID);
    abortCalib=false;

    dd->view(iCalibrate);
    dd->view(iEncoders);
    dd->view(iPosition);
    dd->view(iPids);
    dd->view(iControlMode);

    if (!(iCalibrate && iEncoders && iPosition && iPids && iControlMode)) {
        fprintf(logfile, "TORSOCALIB[%d]: Error. This device cannot be calibrated\n", canID);
        return false;
    }

    // ok we have all interfaces
    int nj=0;
    if (!iEncoders->getAxes(&nj)) {
        fprintf(logfile, "TORSOCALIB[%d]: Error getting number of encoders\n", canID);
        return false;
    }

    original_pid=new Pid[nj];
    limited_pid =new Pid[nj];
    bool calibration_ok=true;
    int k=0;
    int j=0;

    for(k=0;k<nj;k++)
    {
        iEncoders->resetEncoder(k);
    }

    /////////////////////////////////////
    //limit the pwm of head and torso  //
    /////////////////////////////////////
    //BLL board of the neck must receive calibration message before enabling PWM
    for (k = 0; k < 3; k++)
    {
        //bll boards are joint 0 1 2
        calibrateJoint(k);
        iPids->getPid(k,&original_pid[k]);
        limited_pid[k]=original_pid[k];
        limited_pid[k].max_int=maxPWM[k];
        limited_pid[k].max_output=maxPWM[k];
        iPids->setPid(k,limited_pid[k]);
    }

    /////////////////////////////////////
    //enable all joints                //
    /////////////////////////////////////
    for (k = 0; k < 3; k++)
    {
        iControlMode->setControlMode((k), VOCAB_CM_POSITION);
    }

    int torsoSetOfJoints[] = {0, 1, 2}; // these are BLL motors

    /////////////////////////////////////
    //calibrate the torso set of joints//
    /////////////////////////////////////
    if (1)
    {
        calibration_ok = true;
        for (k =0; k < 3; k++)
        {
            //fprintf(logfile, "TORSOCALIB::Moving joint %d to zero\n", k);
            goToZero(torsoSetOfJoints[k]);
        }
        for (k = 0; k < 3; k++)
        {
            //fprintf(logfile, "TORSOCALIB::Waiting for joint %d movement\n", k);
            calibration_ok &= checkGoneToZeroThreshold(torsoSetOfJoints[k]);
        }
        if (calibration_ok)
        {
            fprintf(logfile, "TORSOCALIB[%d]: Calibration done!\n", canID);
            for (k = 0; k < 3; k++)
            iPids->setPid(torsoSetOfJoints[k],original_pid[torsoSetOfJoints[k]]);
        }
        else
        {
            fprintf(logfile, "TORSOCALIB[%d]: Calibration failed!\n", canID);
            for (k = 0; k < 3; k++)
            iControlMode->setControlMode((torsoSetOfJoints[k]), VOCAB_CM_IDLE);
        }
    }
    /////////////////////////////////////
    //finished!                        //
    /////////////////////////////////////
    fprintf(logfile, "TORSOCALIB[%d]: Calibration done!\n", canID);
    return true;
}

void iCubTorsoOnlyCalibrator::calibrateJoint(int joint)
{
    fprintf(logfile, "TORSOCALIB[%d]: Calling calibrateJoint on joint %d with params: %d  %+6.1f %+6.1f %+6.1f\n", canID, joint, type[joint], param1[joint], param2[joint], param3[joint]);
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool iCubTorsoOnlyCalibrator::checkCalibrateJointEnded(int joint)
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
        fprintf(logfile, "TORSOCALIB[%d]: Timeout on joint %d while calibrating!\n", canID, joint);
        return false;
    }
    else if (abortCalib)
    {
        fprintf(logfile, "TORSOCALIB[%d]: Aborting calibration of %d\n", canID, joint);
        return false;
    }

    fprintf(logfile, "TORSOCALIB[%d]: Calibration of joint %d done\n", canID, joint);
    return true;
}


void iCubTorsoOnlyCalibrator::goToZero(int j)
{
    if (abortCalib)
        return;
    iControlMode->setPositionMode(j);
    iPosition->setRefSpeed(j, zeroVel[j]);
    iPosition->positionMove(j, zeroPos[j]);
}

void iCubTorsoOnlyCalibrator::checkGoneToZero(int j)
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
            fprintf(logfile, "TORSOCALIB[%d]: Timeout on joint %d while going to zero!\n", canID, j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(logfile, "TORSOCALIB[%d]: Abort wait for joint %d going to zero!\n", canID, j);
}

bool iCubTorsoOnlyCalibrator::checkGoneToZeroThreshold(int j)
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
        fprintf(logfile, "TORSOCALIB[%d]: (joint %d) curr:%.2f des:%.2f -> delta:%.2f\n", canID, j, ang, zeroPos[j], delta);
        if (delta<POSITION_THRESHOLD)
        {
            fprintf(logfile, "TORSOCALIB[%d]: (joint %d) completed! delta:%f\n", canID, j,delta);
            finished=true;
        }

        Time::delay (0.5);
        timeout ++;

        if (timeout >= GO_TO_ZERO_TIMEOUT)
        {
            fprintf(logfile, "TORSOCALIB[%d]: Timeout on joint %d while going to zero!\n", canID, j);
            return false;
        }
    }
    if (abortCalib)
        fprintf(logfile, "TORSOCALIB[%d]: Abort wait for joint %d going to zero!\n", canID, j);

    return finished;
}

bool iCubTorsoOnlyCalibrator::park(DeviceDriver *dd, bool wait)
{
    int nj=0;
    abortParking=false;

    if (!iEncoders || !iEncoders->getAxes(&nj))
    {
        fprintf(logfile, "TORSOCALIB[%d]: Error getting number of encoders\n", canID);
        return false;
    }

    int timeout = 0;
    fprintf(logfile, "TORSOCALIB[%d]: Calling iCubHeadCalibratorV2::park() \n", canID);
    iPosition->setPositionMode();
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
    {
        fprintf(logfile, "TORSOCALIB[%d]: Moving to park positions \n", canID);
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
                        fprintf(logfile, "TORSOCALIB[%d]: iCubHeadCalibratorV2::park() : joint %d not in position ", canID, j);
                }
                else
                    fprintf(logfile, "TORSOCALIB[%d]: iCubHeadCalibratorV2::park() : joint %d did not answer ", canID, j);
            }
        }
    }

    if (abortParking)
        fprintf(logfile, "TORSOCALIB[%d]: Park was aborted!\n", canID);
    else
        fprintf(logfile, "TORSOCALIB[%d]: Park was done!\n", canID);
    return true;
}

bool iCubTorsoOnlyCalibrator::quitCalibrate()
{
    fprintf(logfile, "TORSOCALIB[%d]: Quitting calibrate\n", canID);
    abortCalib=true;
    return true;
}

bool iCubTorsoOnlyCalibrator::quitPark()
{
    fprintf(logfile, "TORSOCALIB[%d]: Quitting park\n", canID);
    abortParking=true;
    return true;
}

