// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/


#include <yarp/os/Time.h>

#include "iCubHeadCalibratorV2.h"
#include <math.h>

using namespace yarp::os;
using namespace yarp::dev;

const int PARK_TIMEOUT=30;
const int GO_TO_ZERO_TIMEOUT=20;
const int CALIBRATE_JOINT_TIMEOUT=25;
const double POSITION_THRESHOLD=2.0;

#define TORSO_IS_AVAILABLE (nj > 5)

iCubHeadCalibratorV2::iCubHeadCalibratorV2()
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

iCubHeadCalibratorV2::~iCubHeadCalibratorV2()
{
    //empty now
}

bool iCubHeadCalibratorV2::open (yarp::os::Searchable& config)
{
    Property p;
    p.fromString(config.toString());

    if (!p.check("GENERAL")) {
        fprintf(logfile, "HEADCALIB::Cannot understand configuration parameters\n");
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
       fprintf(logfile, "HEADCALIB[%d] :MaxPWM parameter not found, using default values\n", canID);
       for (i = 0; i < 3; i++) maxPWM[i] = 250;    //head
       for (i = 3; i < 6; i++) maxPWM[i] = 1333;   //eyes do not use maxPWM
       if (TORSO_IS_AVAILABLE)
        {
            for (i = 6; i < 9; i++) maxPWM[i] = 200;    //torso
        }
   }

    return true;
}

bool iCubHeadCalibratorV2::close ()
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

bool iCubHeadCalibratorV2::calibrate(DeviceDriver *dd)
{
    fprintf(logfile, "Calling iCubHeadCalibratorV2::calibrate\n");
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
    //BLL boards must receive calibration message before enabling PWM
    if (TORSO_IS_AVAILABLE)
    {
        for (k = 6; k < nj; k++) 
        {
            //bll boards are joint 6 7 8 (9)
            calibrateJoint(k);
            iPids->getPid(k,&original_pid[k]);
            limited_pid[k]=original_pid[k];
            limited_pid[k].max_int=maxPWM[k];
            limited_pid[k].max_output=maxPWM[k];
            iPids->setPid(k,limited_pid[k]);
        }
    }
    
    /////////////////////////////////////
    //enable all joints                //
    /////////////////////////////////////
    for (k = 0; k < nj; k++) 
    {
        fprintf(logfile, "HEADCALIB[%d]: Calling enable amp for joint %d\n", canID, k);
        iAmps->enableAmp(k);
        fprintf(logfile, "HEADCALIB[%d]: Calling enable pid for joint %d\n", canID, k);
        iPids->enablePid(k);
    }

    /////////////////////////////////////
    //create three set of joints       //
    /////////////////////////////////////
    int headSetOfJoints[] =  {0, 1, 2}; // these are BLL motors
    int torsoSetOfJoints[] = {6, 7, 8}; // these are BLL motors
    int eyeSetOfJoints[] =   {3, 4, 5}; // these are NOT BLL motors

    /////////////////////////////////////
    //calibrate the torso set of joints//
    /////////////////////////////////////
    if (TORSO_IS_AVAILABLE)
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
                iAmps->disableAmp(torsoSetOfJoints[k]);
        }
    }
    /////////////////////////////////////
    //calibrate the head set of joints //
    /////////////////////////////////////
    calibration_ok = true;
    for (k =0; k < 3; k++)
    {
        //fprintf(logfile, "HEADCALIB::Moving joint %d to zero\n", k);
        goToZero(headSetOfJoints[k]);
    }
    for (k = 0; k < 3; k++)
    {
        //fprintf(logfile, "HEADCALIB::Waiting for joint %d movement\n", k);
        calibration_ok &= checkGoneToZeroThreshold(headSetOfJoints[k]);
    }
    if (calibration_ok)
    {
        fprintf(logfile, "HEADCALIB[%d]: Calibration done!\n", canID);
    for (k = 0; k < 3; k++)
        iPids->setPid(headSetOfJoints[k],original_pid[headSetOfJoints[k]]);
    }
    else
    {
        fprintf(logfile, "HEADCALIB[%d]: Calibration failed!\n", canID);
        for (k = 0; k < 3; k++)
            iAmps->disableAmp(headSetOfJoints[k]);
    }

    //////////////////////////////////////
    //calibrate the eye set of joints   //
    //////////////////////////////////////
/* for (k =0; k < 3; k++)
    {
        //LATER: calibrateJoint will be removed after enableAmp
        calibrateJoint(eyeSetOfJoints[k]); 
    }
    Time::delay(0.010);
    for (k =0; k < 3; k++)
    {
        checkCalibrateJointEnded(eyeSetOfJoints[k]);
    }
    for (k = 0; k < 3; k++)
    {
        goToZero(eyeSetOfJoints[k]);
    }
    for (k = 0; k < 3; k++)
    {
        checkGoneToZero(eyeSetOfJoints[k]);
    }
*/
    for (k =0; k < 3; k++)
    {
        calibrateJoint(eyeSetOfJoints[k]); 
        checkCalibrateJointEnded(eyeSetOfJoints[k]);
        goToZero(eyeSetOfJoints[k]);
        checkGoneToZero(eyeSetOfJoints[k]);
        Time::delay(0.010);
    }


    /////////////////////////////////////
    //finished!                        //
    /////////////////////////////////////
    fprintf(logfile, "HEADCALIB[%d]: Calibration done!\n", canID);
    return true;
}

void iCubHeadCalibratorV2::calibrateJoint(int joint)
{
    fprintf(logfile, "HEADCALIB[%d]: Calling calibrateJoint on joint %d with params: %d  %+6.1f %+6.1f %+6.1f\n", canID, joint, type[joint], param1[joint], param2[joint], param3[joint]);
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool iCubHeadCalibratorV2::checkCalibrateJointEnded(int joint)
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
        fprintf(logfile, "HEADCALIB::Timeout on joint %d while calibrating!\n", joint);
        return false;
    }
    else if (abortCalib)
    {
        fprintf(logfile, "HEADCALIB::aborting calibration of %d\n", joint);
        return false;
    }
    else
    {
        fprintf(logfile, "HEADCALIB::calibration of joint %d done\n", joint);
    }

    return true;
}


void iCubHeadCalibratorV2::goToZero(int j)
{
    if (abortCalib)
        return;
    iControlMode->setPositionMode(j);
    iPosition->setRefSpeed(j, zeroVel[j]);
    iPosition->positionMove(j, zeroPos[j]);
}

void iCubHeadCalibratorV2::checkGoneToZero(int j)
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
            fprintf(logfile, "HEADCALIB[%d] Timeout on joint %d while going to zero!\n", canID, j);
            finished = true;
        }
    }
    if (abortCalib)
        fprintf(logfile, "HEADCALIB[%d] abort wait for joint %d going to zero!\n", canID, j);
}

bool iCubHeadCalibratorV2::checkGoneToZeroThreshold(int j)
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
        fprintf(logfile, "HEADCALIB[%d] (joint %d) curr:%.2f des:%.2f -> delta:%.2f\n", canID, j, ang, zeroPos[j], delta);
        if (delta<POSITION_THRESHOLD)
        {
            fprintf(logfile, "HEADCALIB[%d] (joint %d) completed! delta:%f\n", canID, j,delta);
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

bool iCubHeadCalibratorV2::park(DeviceDriver *dd, bool wait)
{
    int nj=0;
    bool ret=false;
    abortParking=false;

    ret=iEncoders->getAxes(&nj);
    if (!ret)
    {
        fprintf(logfile, "HEADCALIB[%d]: error getting number of encoders\n",canID);
        return false;
    }



    int timeout = 0;
    fprintf(logfile, "HEADCALIB[%d]: Calling iCubHeadCalibratorV2::park() \n",canID);
    iPosition->setPositionMode();
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);

    if (wait)
        {
        fprintf(logfile, "HEADCALIB[%d]: Moving to park positions \n",canID);
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
                            fprintf(logfile, "iCubHeadCalibratorV2::park() : joint %d not in position ", j);
                    }
                    else
                        fprintf(logfile, "iCubHeadCalibratorV2::park() : joint %d did not answer ", j);
                }
            }
        }

    if (abortParking)
        fprintf(logfile, "HEADCALIB[%d]::Park was aborted!\n", canID);
    else
        fprintf(logfile, "HEADCALIB[%d]::Park was done!\n", canID);
    return true;
}

bool iCubHeadCalibratorV2::quitCalibrate()
{
    fprintf(logfile, "HEADCALIB[%d]: Quitting calibrate\n", canID);
    abortCalib=true;
    return true;
}

bool iCubHeadCalibratorV2::quitPark()
{
    fprintf(logfile, "HEADCALIB[%d]::quitting park\n", canID);
    abortParking=true;
    return true;
}
