// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2014 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino, Marco Randazzo, Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>

#include "parametricCalibrator.h"
#include <math.h>
#include <algorithm>
#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::dev;

// calibrator for the arm of the Arm iCub

const int       PARK_TIMEOUT            = 30;
const double    GO_TO_ZERO_TIMEOUT      = 10; //seconds how many? // was 10
const int       CALIBRATE_JOINT_TIMEOUT = 20;
const double    POSITION_THRESHOLD      = 2.0;

// TODO use it!!
//#warning "Use extractGroup to verify size of parameters matches with number of joints, this will avoid crashes"
static bool extractGroup(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        yError () << key1.c_str() << " not found\n";
        return false;
    }

    if(tmp.size()!=size)
    {
        yError () << key1.c_str() << " incorrect number of entries in board.";
        return false;
    }

    out=tmp;
    return true;
}

parametricCalibrator::parametricCalibrator() :
    type(NULL),
    param1(NULL),
    param2(NULL),
    param3(NULL),
    original_max_pwm(NULL),
    limited_max_pwm(NULL),
    maxPWM(NULL),
    currPos(NULL),
    currVel(NULL),
    zeroPos(NULL),
    zeroVel(NULL),
    homeVel(0),
    homePos(0),
    zeroPosThreshold(0),
    abortCalib(false),
    isCalibrated(false),
    calibMutex(1),
    skipCalibration(false),
    clearHwFault(false),
    n_joints(0),
    totJointsToCalibrate(0)
{
}

parametricCalibrator::~parametricCalibrator()
{
    yTrace();
    close();
}

bool parametricCalibrator::open(yarp::os::Searchable& config)
{
    yTrace();
    Property p;
    p.fromString(config.toString());

    if (p.check("GENERAL") == false)
    {
        yError() << "Parametric calibrator: missing [GENERAL] section";
        return false;
    }

    if (p.findGroup("GENERAL").check("deviceName"))
    {
        deviceName = p.findGroup("GENERAL").find("deviceName").asString();
    }
    else
    {
        yError() << "Parametric calibrator: missing deviceName parameter";
        return false;
    }

    std::string str;
    if (config.findGroup("GENERAL").find("verbose").asInt())
    {
        str = config.toString().c_str();
        yTrace() << deviceName.c_str() << str;
    }

    // Check clearHwFaultBeforeCalibration
    Value val_clearHwFault = config.findGroup("GENERAL").find("clearHwFaultBeforeCalibration");
    if (val_clearHwFault.isNull())
    {
        clearHwFault = false;
    }
    else
    {
        if (!val_clearHwFault.isBool())
        {
            yError() << deviceName.c_str() << ": clearHwFaultBeforeCalibration bool param is different from accepted values (true / false). Assuming false";
            clearHwFault = false;
        }
        else
        {
            clearHwFault = val_clearHwFault.asBool();
            if (clearHwFault)
                yInfo() << deviceName.c_str() << ":  clearHwFaultBeforeCalibration option enabled\n";
        }
    }

    // Check Vanilla = do not use calibration!
    skipCalibration = config.findGroup("GENERAL").find("skipCalibration").asBool();// .check("Vanilla",Value(1), "Vanilla config");
    skipCalibration = !!skipCalibration;
    if (skipCalibration)
    {
        yWarning() << deviceName << ": skipping calibration!! This option was set in general.xml file.";
        yWarning() << deviceName << ": BE CAREFUL USING THE ROBOT IN THIS CONFIGURATION! See 'skipCalibration' param in config file";
    }

    int nj = 0;
    if (p.findGroup("GENERAL").check("joints"))
    {
        nj = p.findGroup("GENERAL").find("joints").asInt();
    }
    else if (p.findGroup("GENERAL").check("Joints"))
    {
        // This is needed to be backward compatibile with old iCubInterface
        nj = p.findGroup("GENERAL").find("Joints").asInt();
    }
    else
    {
        yError() << deviceName.c_str() << ": missing joints parameter";
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
    zeroPosThreshold = new double[nj];

    int i = 0;

    Bottle& xtmp = p.findGroup("CALIBRATION").findGroup("calibration1");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of Calibration1 params " << xtmp.size() << " " << nj; return false; }
    for (i = 1; i < xtmp.size(); i++) param1[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration2");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of Calibration2 params"; return false; }
    for (i = 1; i < xtmp.size(); i++) param2[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration3");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of Calibration3 params"; return false; }
    for (i = 1; i < xtmp.size(); i++) param3[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibrationType");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of Calibration3 params"; return false; }
    for (i = 1; i < xtmp.size(); i++) type[i - 1] = (unsigned char)xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("startupPosition");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of startupPosition params"; return false; }
    for (i = 1; i < xtmp.size(); i++) zeroPos[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("startupVelocity");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of startupVelocity params"; return false; }
    for (i = 1; i < xtmp.size(); i++) zeroVel[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("positionHome");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of PositionHome params"; return false; }
    for (i = 1; i < xtmp.size(); i++) homePos[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("velocityHome");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of VelocityHome params"; return false; }
    for (i = 1; i < xtmp.size(); i++) homeVel[i - 1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("startupMaxPwm");
    if (xtmp.size() - 1 != nj) { yError() << deviceName << ": invalid number of startupMaxPwm params"; return false; }
    for (i = 1; i < xtmp.size(); i++) maxPWM[i - 1] = xtmp.get(i).asInt();

    xtmp = p.findGroup("CALIBRATION").findGroup("startupPosThreshold");
    if (xtmp.size()-1!=nj) {yError() <<  deviceName << ": invalid number of startupPosThreshold params"; return false;}
    for (i = 1; i < xtmp.size(); i++) zeroPosThreshold[i-1] =  xtmp.get(i).asDouble();
 
    calibJointsString = p.findGroup("CALIB_ORDER");
    int calib_order_size = calibJointsString.size();
    if (calib_order_size <= 1) {yError() << deviceName << ": invalid number CALIB_ORDER params"; return false;}
    //yDebug() << "CALIB_ORDER: group size: " << calibJointsString.size() << " values: " << calibJointsString.toString().c_str();

    std::list<int>  tmp;
    for(int i=1; i<calibJointsString.size(); i++)
    {
        tmp.clear();
        Bottle *set;
        set= calibJointsString.get(i).asList();

        for(int j=0; j<set->size(); j++)
        {
            tmp.push_back(set->get(j).asInt() );
        }
        joints.push_back(tmp);
    }
    return true;
}

bool parametricCalibrator::close ()
{
    yTrace();
    if (type != NULL) {
        delete[] type;
        type = NULL;
    }
    if (param1 != NULL) {
        delete[] param1;
        param1 = NULL;
    }
    if (param2 != NULL) {
        delete[] param2;
        param2 = NULL;
    }
    if (param3 != NULL) {
        delete[] param3;
        param3 = NULL;
    }

    if (maxPWM != NULL) {
        delete[] maxPWM;
        maxPWM = NULL;
    }
    if (original_max_pwm != NULL) {
        delete[] original_max_pwm;
        original_max_pwm = NULL;
    }
    if (limited_max_pwm != NULL) {
        delete[] limited_max_pwm;
        limited_max_pwm = NULL;
    }

    if (currPos != NULL) {
        delete[] currPos;
        currPos = NULL;
    }
    if (currVel != NULL) {
        delete[] currVel;
        currVel = NULL;
    }

    if (zeroPos != NULL) {
        delete[] zeroPos;
        zeroPos = NULL;
    }
    if (zeroVel != NULL) {
        delete[] zeroVel;
        zeroVel = NULL;
    }

    if (homePos != NULL) {
        delete[] homePos;
        homePos = NULL;
    }
    if (homeVel != NULL) {
        delete[] homeVel;
        homeVel = NULL;
    }

    return true;
}

bool parametricCalibrator::calibrate(DeviceDriver *device)
{
    yInfo() << deviceName << ": starting calibration";
    yTrace();
    abortCalib  = false; //set true in quitCalibrate function  (called on ctrl +c signal )


    if (device==0)
    {
        yError() << deviceName << ": invalid device driver";
        return false;
    }

    yarp::dev::PolyDriver *p = dynamic_cast<yarp::dev::PolyDriver *>(device);
	dev2calibrate = p;
    if (p!=0)
    {
        p->view(iCalibrate);
        p->view(iEncoders);
        p->view(iPosition);
        p->view(iPids);
        p->view(iControlMode);
        p->view(iAmp);
    }
    else
    {
        //yError() << deviceName << ": invalid dynamic cast to yarp::dev::PolyDriver";
        //return false;
        
        //This is to ensure backward-compatibility with iCubInterface
        yWarning() << deviceName << ": using parametricCalibrator on an old iCubInterface system. Upgrade to robotInterface is recommended."; 
        device->view(iCalibrate);
        device->view(iEncoders);
        device->view(iPosition);
        device->view(iPids);
        device->view(iControlMode);
        device->view(iAmp);
    }

    if (!(iCalibrate && iEncoders && iPosition && iPids && iControlMode)) {
        yError() << deviceName << ": interface not found" << iCalibrate << iPosition << iPids << iControlMode;
        return false;
    }

    return calibrate();
}

bool parametricCalibrator::calibrate()
{
    int  setOfJoint_idx = 0;
    totJointsToCalibrate = 0;
    calibJoints.clear();

    if (dev2calibrate==0)
    {
        yError() << deviceName << ": not able to find a valid device to calibrate";
        return false;
    }

    if ( !iEncoders->getAxes(&n_joints))
    {
        yError() << deviceName << ": error getting number of axes" ;
        return false;
    }

    std::list<int>  currentSetList;
    std::list<std::list<int> >::iterator Bit=joints.begin();
    std::list<std::list<int> >::iterator Bend=joints.end();

    // count how many joints are there in the list of things to be calibrated
    while(Bit != Bend)
    {
        currentSetList.clear();
        currentSetList = (*Bit);
        std::list<int>::iterator lit  = currentSetList.begin();
        std::list<int>::iterator lend = currentSetList.end();
        totJointsToCalibrate += currentSetList.size();

        while(lit != lend)
        {
            calibJoints.push_back(*lit);
            lit++;
        }
        Bit++;
    }

    //before starting the calibration, checks for joints in hardware fault, and clears them if the user set the clearHwFaultBeforeCalibration option
    for (int i=0; i<totJointsToCalibrate; i++)
    {
        checkHwFault(i);
    }

    yDebug() << deviceName << ": Joints calibration order:" << calibJointsString.toString();

    if (totJointsToCalibrate > n_joints)
    {
        yError() << deviceName << ": too much axis to calibrate for this part..." << totJointsToCalibrate << " bigger than "<< n_joints;
        return false;
    }

    if (totJointsToCalibrate < n_joints)
    {
        yWarning() << deviceName << " is calibrating only a subset of the robot part. Calibrating " << totJointsToCalibrate << " over a total of " << n_joints;
    }

    original_max_pwm = new double[n_joints];
    limited_max_pwm = new double[n_joints];

    if(skipCalibration)
        yWarning() << deviceName << ": skipCalibration flag is on! Setting safe pid but skipping calibration.";

    Bit=joints.begin();
    std::list<int>::iterator lit; //iterator for joint in a set 
    while( (Bit != Bend) && (!abortCalib) )   // for each set of joints
    {
        setOfJoint_idx++;
        currentSetList.clear();
        currentSetList = (*Bit);

        // 1) set safe pid
        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            yWarning() << deviceName << "lit is " << *lit;
            if ( ((*lit) <0) || ((*lit) >= n_joints) )   // check the axes is within range
            {
                yError() << deviceName << ": asked to calibrate joint" << (*lit) << ", which is negative OR bigger than the number of axes for this part ("<< n_joints << ")";
                abortCalib = true;
                break;
            }

            if(!iAmp->getPWMLimit((*lit), &original_max_pwm[(*lit)]) )
            {
                yError() << deviceName << ": getPWMLimit joint " << (*lit) << "failed... aborting calibration";
                abortCalib = true;
                break;
            }

            limited_max_pwm[(*lit)] = original_max_pwm[(*lit)];
            
            if (maxPWM[(*lit)]==0)
            {
                yDebug() << deviceName << ": skipping maxPwm=0 of joint " << (*lit);
                iAmp->setPWMLimit((*lit), original_max_pwm[(*lit)]);
            }
            else
            {
                if (maxPWM[(*lit)]<limited_max_pwm[(*lit)])
                {
                    limited_max_pwm[(*lit)] = maxPWM[(*lit)];
                    iAmp->setPWMLimit((*lit), limited_max_pwm[(*lit)]);
                }
                else
                {
                    yDebug() << deviceName << ": joint " << (*lit) << " has max_output already limited to a safe value: " << limited_max_pwm[(*lit)];
                }
            }
        }

        if(skipCalibration)     // if this flag is on, fake calibration
        {
            Bit++;
            continue;
        }

        //2) if calibration needs to go to hardware limits, enable joint
        //VALE: i can add this cycle for calib on eth because it does nothing,
        //     because enablePid doesn't send command because joints are not calibrated

        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            if (type[*lit]==0 ||
                type[*lit]==2 ||
                type[*lit]==4 ) 
            {
                yDebug() << "In calibration " <<  deviceName  << ": enabling joint " << *lit << " to test hardware limit";
                iControlMode->setControlMode((*lit), VOCAB_CM_POSITION);
            }
        }
        
        Time::delay(0.1f);
        if(abortCalib)
        {
            Bit++;
            continue;
        }

        //3) send calibration command
        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            // Enable amp moved into EMS class;
            // Here we just call the calibration procedure
            calibrateJoint((*lit));
        }

        Time::delay(0.1f);

        for(lit  = currentSetList.begin(); lit != currentSetList.end(); lit++)      //for each joint of set
        {
            iEncoders->getEncoder((*lit), &currPos[(*lit)]);
            yDebug() <<  deviceName  << ": set" << setOfJoint_idx << "j" << (*lit) << ": Calibrating... enc values AFTER calib: " << currPos[(*lit)];
        }

        if(abortCalib)
        {
            Bit++;
            continue;
        }

        //4) check calibration result
        if(checkCalibrateJointEnded((*Bit)) ) //check calibration on entire set
        {
            yDebug() <<  deviceName  << ": set" << setOfJoint_idx  << ": Calibration ended, going to zero!\n";
        }
        else    // keep pid safe  and go on
        {
            yError() <<  deviceName  << ": set" << setOfJoint_idx << ": Calibration went wrong! Disabling axes and keeping safe pid limit\n";

            for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++)  //for each joint of set
            {
                iControlMode->setControlMode((*lit),VOCAB_CM_IDLE);
            }
            Bit++;
            continue; //go to next set
        }

        // 5) if calibration finish with success enable disabled joints in order to move them to zero
        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++)   //for each joint of set
        {
            // if the joint han not been enabled at point 1, now i enable it 
            //iAmps->enableAmp((*lit));
            if (type[*lit]!=0 &&
                type[*lit]!=2 &&
                type[*lit]!=4 ) 
            {
                iControlMode->setControlMode((*lit), VOCAB_CM_POSITION);
            }
        }

        if(abortCalib)
        {
            Bit++;
            continue;
        }
        Time::delay(0.5f);    // needed?

        //6) go to zero
        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            // Manda in Zero
            goToZero((*lit));
        }

        if(abortCalib)
        {
            Bit++;
            continue;
        }
        Time::delay(1.0);     // needed?

        //7) check joints are in position
        bool goneToZero = true;
        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            goneToZero &= checkGoneToZeroThreshold(*lit);
        }

        if(abortCalib)
        {
            Bit++;
            continue;
        }

        if(goneToZero)
        {
            yDebug() <<  deviceName  << ": set" << setOfJoint_idx  << ": Reached zero position!\n";
            for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
            {
                iAmp->setPWMLimit((*lit),original_max_pwm[(*lit)]);
            }
        }
        else          // keep pid safe and go on
        {
            yError() <<  deviceName  << ": set" << setOfJoint_idx  << ": some axis got timeout while reaching zero position... disabling this set of axes\n";
            for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
            {
                iControlMode->setControlMode((*lit),VOCAB_CM_IDLE);
            }
        }

        // Go to the next set of joints to calibrate... if any
        Bit++;
    }
    
    if(abortCalib)
    {
        yError() << deviceName << ": calibration has been aborted!I'm going to disable all joints..." ;
        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            iControlMode->setControlMode(*lit, VOCAB_CM_IDLE);
        }
        return false;
    }
    calibMutex.wait();
    isCalibrated = true;
    calibMutex.post();
    return isCalibrated;
}

bool parametricCalibrator::calibrateJoint(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'calibration' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }
    yInfo() <<  deviceName  << ": Calling calibrateJoint on joint "<< j << " with params: " << type[j] << param1[j] << param2[j] << param3[j];
    return iCalibrate->calibrate2(j, type[j], param1[j], param2[j], param3[j]);
}

bool parametricCalibrator::checkCalibrateJointEnded(std::list<int> set)
{
    int timeout = 0;
    bool calibration_ok = false;

    std::list<int>::iterator lit;
    std::list<int>::iterator lend;

    lend = set.end();
    while(!calibration_ok && (timeout <= CALIBRATE_JOINT_TIMEOUT))
    {
        calibration_ok = true;
        Time::delay(1.0);
        lit  = set.begin();
        while(lit != lend)    // per ogni giunto del set
        {
            if (abortCalib)
            {
                yWarning() << deviceName  << ": calibration aborted\n";
            }

            // Joint with absolute sensor doesn't need to move, so they are ok with just the calibration message,
            // but I'll check anyway, in order to have everything the same
            if( !(calibration_ok &=  iCalibrate->done((*lit))) )  // the assignement inside the if is INTENTIONAL
                break;
            lit++;
        }

        timeout++;
    }

    if(timeout > CALIBRATE_JOINT_TIMEOUT)
        yError() << deviceName << ":Timeout while calibrating " << (*lit) << "\n";
    else
        yDebug() << deviceName << ": calib joint ended";

    return calibration_ok;
}

bool parametricCalibrator::checkHwFault(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'check hw fault' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    int mode=0;
    iControlMode->getControlMode(j,&mode);
    if (mode == VOCAB_CM_HW_FAULT)
    {
        if (clearHwFault)
        {
            iControlMode->setControlMode(j,VOCAB_CM_FORCE_IDLE);
            yWarning() << deviceName <<": detected an hardware fault on joint " << j << ". An attempt will be made to clear it.";
            Time::delay(0.02f);
            iControlMode->getControlMode(j,&mode);
            if (mode == VOCAB_CM_HW_FAULT)
            {
                yError() << deviceName <<": unable to clear the hardware fault detected on joint " << j << " before starting the calibration procedure!";
                return false;
            }
            else if (mode == VOCAB_CM_IDLE)
            {
                yWarning() << deviceName <<": hardware fault on joint " << j << " successfully cleared.";
                return true;
            }
            else
            {
                yError() << deviceName <<": an unknown error occured while trying the hardware fault on joint " << j ;
                return false;
            }
        }
        else
        {
            yError() << deviceName <<": detected an hardware fault on joint " << j << " before starting the calibration procedure!";
            return false;
        }
    }
    return true;
}

bool parametricCalibrator::goToZero(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'go to zero' operation because joint number %d is out of range [%s]!!", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    bool ret = true;
    if (abortCalib) return true;
    yDebug() <<  deviceName  << ": Sending positionMove to joint" << j << " (desired pos: " << zeroPos[j] << "desired speed: " << zeroVel[j] <<" )";
    ret  = iPosition->setRefSpeed(j, zeroVel[j]);
    ret &= iPosition->positionMove(j, zeroPos[j]);
    return ret;
}

bool parametricCalibrator::checkGoneToZeroThreshold(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'check gone to zero' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    if (skipCalibration) return false;

    // wait.
    bool finished = false;
//    double ang[4];
    double angj = 0;
    double output = 0;
    double delta=0;
    int mode=0;
    bool done = false;

    double start_time = yarp::os::Time::now();
    while ( (!finished) && (!abortCalib))
    {
        iEncoders->getEncoder(j, &angj);
        iPosition->checkMotionDone(j, &done);
        iControlMode->getControlMode(j, &mode);
        iPids->getOutput(j, &output);

        delta = fabs(angj-zeroPos[j]);
        yDebug("%s: checkGoneToZeroThreshold: joint: %d curr: %.3f des: %.3f -> delta: %.3f threshold: %.3f output: %.3f mode: %s" ,deviceName.c_str(),j,angj, zeroPos[j],delta, zeroPosThreshold[j], output, yarp::os::Vocab::decode(mode).c_str());

        if (delta < zeroPosThreshold[j] && done)
        {
            yDebug("%s: checkGoneToZeroThreshold: joint: %d completed with delta: %.3f over: %.3f" ,deviceName.c_str(),j,delta, zeroPosThreshold[j]);
            finished=true;
            break;
        }
        if (yarp::os::Time::now() - start_time > GO_TO_ZERO_TIMEOUT)
        {
            yError() <<  deviceName << ": checkGoneToZeroThreshold: joint " << j << " Timeout while going to zero!";
            break;
        }
        if (mode == VOCAB_CM_IDLE)
        {
            yError() <<  deviceName << ": checkGoneToZeroThreshold: joint " << j << " is idle, skipping!";
            break;
        }
        if (mode == VOCAB_CM_HW_FAULT)
        {
            yError() << deviceName <<": checkGoneToZeroThreshold: hardware fault on joint " << j << ", skipping!";
            break;
        }
        if (abortCalib)
        {
            yWarning() << deviceName <<": checkGoneToZeroThreshold: joint " << j << " Aborting wait while going to zero!\n";
            break;
        }
        Time::delay(0.5);
    }
    return finished;
}

bool parametricCalibrator::park(DeviceDriver *dd, bool wait)
{
    yTrace();
    bool ret=false;
    abortParking=false;

    calibMutex.wait();
    if(!isCalibrated)
    {
        yWarning() << deviceName << ": Calling park without calibration... skipping";
        calibMutex.post();
        return true;
    }
    calibMutex.post();

    if(skipCalibration)
    {
        yWarning() << deviceName << ": skipCalibration flag is on!! Faking park!!";
        return true;
    }

    int*  currentControlModes = new int  [n_joints];
    bool* cannotPark          = new bool [n_joints];
    bool res = iControlMode->getControlModes(currentControlModes);
    if(!res)
    {
        yError() << deviceName << ": error getting control mode during parking";
    }

    std::list<int>::iterator lit;
    for(lit  = calibJoints.begin(); lit != calibJoints.end() && !abortCalib; lit++) //for each joint of set
    {
        switch(currentControlModes[(*lit)])
        {
            case VOCAB_CM_IDLE:
            {
                yError() << deviceName << ": joint " << (*lit) << " is idle, skipping park";
                cannotPark[(*lit)] = true;
            }
            break;

            case VOCAB_CM_HW_FAULT:
            {
                yError() << deviceName << ": joint " << (*lit) << " has an hardware fault, skipping park";
                cannotPark[(*lit)] = true;
            }
            break;

            case VOCAB_CM_NOT_CONFIGURED:
            {
                yError() << deviceName << ": joint " << (*lit) << " is not configured, skipping park";
                cannotPark[(*lit)] = true;
            }
            break;

            case VOCAB_CM_UNKNOWN:
            {
                yError() << deviceName << ": joint " << (*lit) << " is in unknown state, skipping park";
                cannotPark[(*lit)] = true;
            }

            default:
            {
                iControlMode->setControlMode((*lit), VOCAB_CM_POSITION);
                cannotPark[(*lit)] = false;
            }
        }
    }

    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);
    Time::delay(0.01);
    
    if (wait)
    {
       int timeout = 0; //this variable is shared between all joints
       for(lit  = calibJoints.begin(); lit != calibJoints.end() && !abortCalib; lit++) //for each joint of set
       {
          if (cannotPark[(*lit)] ==false)
          {
             yDebug() << deviceName.c_str() << ": Moving to park position, joint:" << (*lit);
             bool done=false;
             iPosition->checkMotionDone((*lit), &done);
             while ( (!done) && (timeout<PARK_TIMEOUT) && (!abortParking) )
             {
               Time::delay(1);
               timeout++;
               iPosition->checkMotionDone((*lit), &done);
             }
             if (!done)
             {
                 yError() << deviceName << ": joint " << (*lit) << " not in position after a timeout of" << PARK_TIMEOUT <<" seconds";
             }
          }
       }
    }

    yDebug() << deviceName.c_str() << ": Park " << (abortParking ? "aborted" : "completed");
    for(lit  = calibJoints.begin(); lit != calibJoints.end() && !abortCalib; lit++) //for each joint of set

    {
        switch(currentControlModes[(*lit)])
        {
            case VOCAB_CM_IDLE:
            case VOCAB_CM_HW_FAULT:
            case VOCAB_CM_NOT_CONFIGURED:
            case VOCAB_CM_UNKNOWN:
                // Do nothing.
                break;
            default:
            {
                iControlMode->setControlMode((*lit), VOCAB_CM_IDLE);
            }
        }
    }
    return true;
}

bool parametricCalibrator::quitCalibrate()
{
    yDebug() << deviceName.c_str() << ": Quitting calibrate\n";
    abortCalib = true;
    return true;
}

bool parametricCalibrator::quitPark()
{
    yDebug() << deviceName.c_str() << ": Quitting parking\n";
    abortParking=true;
    return true;
}

yarp::dev::IRemoteCalibrator *parametricCalibrator::getCalibratorDevice()
{
    return this;
}

bool parametricCalibrator::calibrateSingleJoint(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'calibrate' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }
    return calibrateJoint(j);
}

bool parametricCalibrator::calibrateWholePart()
{
    yTrace();
    return calibrate();
}

bool parametricCalibrator::homingSingleJoint(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'homing' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }
    return goToZero(j);
}

bool parametricCalibrator::homingWholePart()
{
    yTrace();
    bool ret = true;
    std::list<int>::iterator lit;
    for(lit  = calibJoints.begin(); lit != calibJoints.end() && !abortCalib; lit++) //for each joint of set
    {
        ret = homingSingleJoint(*lit) && ret;
    }
    return ret;
}

bool parametricCalibrator::parkSingleJoint(int j, bool _wait)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'park' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    int nj=0;
    abortParking=false;

    calibMutex.wait();
    if(!isCalibrated)
    {
        yWarning() << deviceName << ": Calling park without calibration... skipping";
        calibMutex.post();
        return true;
    }
    calibMutex.post();

    if(skipCalibration)
    {
        yWarning() << deviceName << ": skipCalibration flag is on!! Faking park!!";
        return true;
    }

    int  currentControlMode;
    bool cannotPark;
    bool res = iControlMode->getControlMode(j, &currentControlMode);
    if(!res)
    {
        yError() << deviceName << ": error getting control mode during parking";
    }

    if(currentControlMode != VOCAB_CM_IDLE &&
       currentControlMode != VOCAB_CM_HW_FAULT)
    {
        iControlMode->setControlMode(j, VOCAB_CM_POSITION);
        cannotPark = false;
    }
    else if (currentControlMode == VOCAB_CM_IDLE)
    {
        yError() << deviceName << ": joint " << j << " is idle, skipping park";
        cannotPark = true;
    }
    else if (currentControlMode == VOCAB_CM_HW_FAULT)
    {
        yError() << deviceName << ": joint " << j << " has an hardware fault, skipping park";
        cannotPark = true;
    }

    iPosition->setRefSpeed(j, homeVel[j]);
    iPosition->positionMove(j, homePos[j]);
    Time::delay(0.01);

    if (_wait)
    {
        int timeout = 0; //this variable is shared between all joints
        if (cannotPark == false)
        {
            yDebug() << deviceName.c_str() << ": Moving to park position, joint:" << j;
            bool done=false;
            iPosition->checkMotionDone(j, &done);
            while((!done) && (timeout<PARK_TIMEOUT) && (!abortParking))
            {
                Time::delay(1);
                timeout++;
                iPosition->checkMotionDone(j, &done);
            }
            if(!done)
            {
                yError() << deviceName << ": joint " << j << " not in position after a timeout of" << PARK_TIMEOUT <<" seconds";
            }
        }
    }

    yDebug() << deviceName.c_str() << ": Park " << (abortParking ? "aborted" : "completed");
    iControlMode->setControlMode(j,VOCAB_CM_IDLE);
    return true;
}

bool parametricCalibrator::parkWholePart()
{
    yTrace();
    if(!isCalibrated)
    {
        yError() << "Device is not calibrated therefore cannot be parked";
        return false;
    }

    return park(dev2calibrate);
}


// eof

