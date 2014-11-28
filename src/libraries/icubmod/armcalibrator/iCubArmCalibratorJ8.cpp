// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <yarp/os/Time.h>

#include "iCubArmCalibratorJ8.h"
#include <math.h>

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the arm of the Arm iCub

const int PARK_TIMEOUT=30;
const double GO_TO_ZERO_TIMEOUT=10; //seconds
const int CALIBRATE_JOINT_TIMEOUT=25;
const double POSITION_THRESHOLD=2.0;

const int numberOfJoints=8;

iCubArmCalibratorJ8::iCubArmCalibratorJ8()
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

iCubArmCalibratorJ8::~iCubArmCalibratorJ8()
{
    //empty now
}

bool iCubArmCalibratorJ8::open (yarp::os::Searchable& config)
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
        fprintf(stdout, "ARMCALIB::calibrator (r2) : opening logfile %s\n", logfile_name.c_str());
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

bool iCubArmCalibratorJ8::close ()
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

bool iCubArmCalibratorJ8::calibrate(DeviceDriver *dd)
{
    fprintf(logfile, "Calling iCubArmCalibratorJ8::calibrate \n");
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

    if (nj!=numberOfJoints)
        {
            fprintf(logfile, "ARMCALIB[%d]: Calibrator is for %d joints but device has %d\n", canID, numberOfJoints, nj);
            return false;
        }

    if (!ret)
        return false;

    original_pid=new Pid[nj];
    limited_pid =new Pid[nj];
    bool calibration_ok=true;
    int k;
    int shoulderSetOfJoints[] = {0, 1 , 2, 3};
    for (k =0; k < 4; k++)
    {
        Time::delay(0.040);
        //fprintf(logfile, "ARMCALIB::Sending offset for joint %d\n", k);
        iEncoders->getEncoders(currPos);
        fprintf(logfile, "ARMCALIB[%d]: calibrating shoulder (j:%d) current enc values: %.2f %.2f %.2f %.2f\n", canID, k, currPos[0], currPos[1], currPos[2], currPos[3]);
        calibrateJoint(shoulderSetOfJoints[k]);
    }
    Time::delay(1.0);

    for (k = 0; k < nj; k++) 
    {
        iPids->getPid(k,&original_pid[k]);
        limited_pid[k]=original_pid[k];
        limited_pid[k].max_int=maxPWM[k];
        limited_pid[k].max_output=maxPWM[k];
        if (k<4) iPids->setPid(k,limited_pid[k]);

        iControlMode->setControlMode((k), VOCAB_CM_POSITION);
    }

    for (k = 0; k < 4; k++)
    {
        //fprintf(logfile, "ARMCALIB::Moving joint %d to zero\n", k);
        goToZero(shoulderSetOfJoints[k]);
    }
    for (k = 0; k < 4; k++)
    {
        //fprintf(logfile, "ARMCALIB::Waiting for joint %d movement\n", k);
        calibration_ok &= checkGoneToZeroThreshold(shoulderSetOfJoints[k]);
    }
    if (calibration_ok)
    {
        fprintf(logfile, "ARMCALIB[%d]: Calibration done!\n", canID);
        for (k = 0; k < 4; k++)
            iPids->setPid(k,original_pid[k]);
    }
    else
    {
        fprintf(logfile, "ARMCALIB[%d]: Calibration failed!\n", canID);
        for (k = 0; k < 4; k++)
            iControlMode->setControlMode((k), VOCAB_CM_IDLE);
    }

    ret = true;
    bool x;

    int firstSetOfJoints[] = {4, 6, 7};
    for (k =0; k < 3; k++)
        calibrateJoint(firstSetOfJoints[k]);
    for (k =0; k < 3; k++)
    {
        x = checkCalibrateJointEnded(firstSetOfJoints[k]);
        ret = ret && x;
    }
    for (k = 0; k < 3; k++)
        goToZero(firstSetOfJoints[k]);
    for (k = 0; k < 3; k++)
        checkGoneToZero(firstSetOfJoints[k]);
    //////////////////////////////////////////
    int secondSetOfJoints[] = {5};
    for (k =0; k < 1; k++)
        calibrateJoint(secondSetOfJoints[k]);
    for (k =0; k < 1; k++)
    {
        x = checkCalibrateJointEnded(secondSetOfJoints[k]);
       ret = ret && x;
    }
    for (k = 0; k < 1; k++)
        goToZero(secondSetOfJoints[k]);
    for (k = 0; k < 1; k++)
        checkGoneToZero(secondSetOfJoints[k]);
    
    fprintf(logfile, "ARMCALIB[%d]: Calibration done!\n", canID);
    return ret;
}

void iCubArmCalibratorJ8::calibrateJoint(int joint)
{
    fprintf(logfile, "ARMCALIB[%d]: Calling calibrateJoint on joint %d with params: %d  %+6.1f %+6.1f %+6.1f\n", canID, joint, type[joint], param1[joint], param2[joint], param3[joint]);
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
}

bool iCubArmCalibratorJ8::checkCalibrateJointEnded(int joint)
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


void iCubArmCalibratorJ8::goToZero(int j)
{
    if (abortCalib)
        return;
    iControlMode->setPositionMode(j);
    iPosition->setRefSpeed(j, zeroVel[j]);
    iPosition->positionMove(j, zeroPos[j]);
}

void iCubArmCalibratorJ8::checkGoneToZero(int j)
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

bool iCubArmCalibratorJ8::checkGoneToZeroThreshold(int j)
{
    // wait.
    bool finished = false;
    double ang[4];
    double angj = 0;
    double pwm[4];
    double delta=0;

    double start_time = yarp::os::Time::now();
    while ( (!finished) && (!abortCalib))
    {
        iEncoders->getEncoder(j, &angj);
        for (int l=0; l<4; l++)
        {
            iEncoders->getEncoder(l, &ang[l]);
            iPids->getOutput(l,&pwm[l]);
        }
        
        delta = fabs(angj-zeroPos[j]);
        fprintf(logfile, "ARMCALIB[%d] (joint %d) curr:%+7.2f des:%+7.2f -> delta:%+7.2f \
                          *** encs: %+7.2f %+7.2f %+7.2f %+7.2f pwms: %+7.2f %+7.2f %+7.2f %+7.2f\n",
                          canID, j, angj, zeroPos[j], delta,
                          ang[0], ang[1], ang[2], ang[3],
                          pwm[0], pwm[1], pwm[2], pwm[3]);
        if (delta<POSITION_THRESHOLD)
        {
            fprintf(logfile, "ARMCALIB[%d] (joint %d) completed! delta:%f\n", canID, j,delta);
            finished=true;
        }

        if (logfile_name != "")
            Time::delay (0.1);
        else
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

bool iCubArmCalibratorJ8::park(DeviceDriver *dd, bool wait)
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

bool iCubArmCalibratorJ8::quitCalibrate()
{
    fprintf(logfile, "ARMCALIB[%d]: Quitting calibrate\n", canID);
    abortCalib=true;
    return true;
}

bool iCubArmCalibratorJ8::quitPark()
{
    fprintf(logfile, "ARMCALIB[%d]: Quitting parking\n", canID);
    abortParking=true;
    return true;
}
