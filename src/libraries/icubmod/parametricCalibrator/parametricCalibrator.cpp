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

#include "Debug.h"

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
    original_pid(NULL),
    limited_pid(NULL),
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
    skipCalibration(false)
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

    if (p.check("GENERAL")==false)
    {
      yError() << "missing [GENERAL] section"; 
      return false;
    } 

    if(p.findGroup("GENERAL").check("deviceName"))
    {
      deviceName = p.findGroup("GENERAL").find("deviceName").asString();
    } 
    else
    {
      yError() << "missing deviceName parameter"; 
      return false;
    } 

    std::string str;
    if(config.findGroup("GENERAL").find("verbose").asInt())
    {
        str=config.toString().c_str();
        yTrace() << deviceName.c_str() << str;
    }  

    // Check Vanilla = do not use calibration!
    skipCalibration =config.findGroup("GENERAL").find("skipCalibration").asBool() ;// .check("Vanilla",Value(1), "Vanilla config");
    skipCalibration = !!skipCalibration;
    if(skipCalibration )
        yWarning() << "parametric calibrator: skipping calibration!! This option was set in general.xml file.";

    int nj = 0;
    if(p.findGroup("GENERAL").check("joints"))
    {
        nj = p.findGroup("GENERAL").find("joints").asInt();
    }
    else if(p.findGroup("GENERAL").check("Joints"))
    {
        // This is needed to be backward compatibile with old iCubInterface
        nj = p.findGroup("GENERAL").find("Joints").asInt();
    }
    else
    {
        yError() << deviceName.c_str() <<  ": missing joints parameter" ;
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

    int i=0;

    Bottle& xtmp = p.findGroup("CALIBRATION").findGroup("calibration1");
    if (xtmp.size()-1!=nj) {yError() << deviceName << ": invalid number of Calibration1 params " << xtmp.size()<< " " << nj; return false;}
    for (i = 1; i < xtmp.size(); i++) param1[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration2");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of Calibration2 params"; return false;}
    for (i = 1; i < xtmp.size(); i++) param2[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration3");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of Calibration3 params"; return false;}
    for (i = 1; i < xtmp.size(); i++) param3[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibrationType");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of Calibration3 params"; return false;}
    for (i = 1; i < xtmp.size(); i++) type[i-1] = (unsigned char) xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("positionZero");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of PositionZero params"; return false;}
    for (i = 1; i < xtmp.size(); i++) zeroPos[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("velocityZero");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of VelocityZero params"; return false;}
    for (i = 1; i < xtmp.size(); i++) zeroVel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("positionHome");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of PositionHome params"; return false;}
    for (i = 1; i < xtmp.size(); i++) homePos[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("HOME").findGroup("velocityHome");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of VelocityHome params"; return false;}
    for (i = 1; i < xtmp.size(); i++) homeVel[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("maxPwm");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of MaxPwm params"; return false;}
    for (i = 1; i < xtmp.size(); i++) maxPWM[i-1] =  xtmp.get(i).asInt();

    xtmp = p.findGroup("CALIBRATION").findGroup("posZeroThreshold");
    if (xtmp.size()-1!=nj) {yError() << "invalid number of PosZeroThreshold params"; return false;}
    for (i = 1; i < xtmp.size(); i++) zeroPosThreshold[i-1] =  xtmp.get(i).asDouble();
 
    xtmp = p.findGroup("CALIB_ORDER");
    int calib_order_size = xtmp.size();
    if (calib_order_size <= 1) {yError() << "invalid number CALIB_ORDER params"; return false;}
    //yDebug() << "CALIB_ORDER: group size: " << xtmp.size() << " values: " << xtmp.toString().c_str();

    std::list<int>  tmp;
    for(int i=1; i<xtmp.size(); i++)
    {
        tmp.clear();
        Bottle *set;
        set= xtmp.get(i).asList();

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
    if (original_pid != NULL) {
        delete[] original_pid;
        original_pid = NULL;
    }
    if (limited_pid != NULL) {
        delete[] limited_pid;
        limited_pid = NULL;
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

bool parametricCalibrator::calibrate(DeviceDriver *dd)
{
    yDebug() << deviceName << "Entering parametricCalibrator::calibrate()";
    yTrace();
    abortCalib  = false; //set true in quitCalibrate function  (called on ctrl +c signal )
    int  setOfJoint_idx = 0;

    int nj=0;
    int totJointsToCalibrate = 0;

    if (dd==0)
    {
        yError() << deviceName << ": invalid device driver";
        return false;
    }

    yarp::dev::PolyDriver *p = dynamic_cast<yarp::dev::PolyDriver *>(dd);
    if (p!=0)
    {
        p->view(iCalibrate);
        p->view(iEncoders);
        p->view(iPosition);
        p->view(iPids);
        p->view(iControlMode);
    }
    else
    {
        //yError() << deviceName << ": invalid dynamic cast to yarp::dev::PolyDriver";
        //return false;
        
        //This is to ensure backward-compatibility with iCubInterface
        yWarning() << deviceName << ": using parametricCalibrator on an old iCubInterface system. Upgrade to robotInterface is recommended."; 
        dd->view(iCalibrate);
        dd->view(iEncoders);
        dd->view(iPosition);
        dd->view(iPids);
        dd->view(iControlMode);
    }

    if (!(iCalibrate && iEncoders && iPosition && iPids && iControlMode)) {
        yError() << deviceName << ": interface not found" << iCalibrate << iPosition << iPids << iControlMode;
        return false;
    }

    if ( !iEncoders->getAxes(&nj))
    {
        yError() << deviceName << "CALIB: error getting number of axes" ;
        return false;
    }

    std::list<int>  currentSetList;
    std::list<std::list<int> >::iterator Bit=joints.begin();
    std::list<std::list<int> >::iterator Bend=joints.end();

    // count how many joints are there in the list of things to be calibrated
    std::string joints_string;
    while(Bit != Bend)
    {
        joints_string += "( ";
        currentSetList.clear();
        currentSetList = (*Bit);
        std::list<int>::iterator lit  = currentSetList.begin();
        std::list<int>::iterator lend = currentSetList.end();
        totJointsToCalibrate += currentSetList.size();

        char joints_buff [10];
        while(lit != lend)
        {
            sprintf(joints_buff, "%d ", (*lit));
            joints_string += joints_buff;
            lit++;
        }
        Bit++;
        joints_string += ") ";
    }
    yDebug() << deviceName <<("Joints calibration order:") << joints_string;

    if (totJointsToCalibrate > nj)
    {
        yError() << deviceName << ": too much axis to calibrate for this part..." << totJointsToCalibrate << " bigger than "<< nj;
        return false;
    }

    original_pid=new Pid[nj];
    limited_pid =new Pid[nj];

    if(skipCalibration)
        yWarning() << deviceName << "skipCalibration flag is on! Setting safe pid but skipping calibration.";

    Bit=joints.begin();
    while( (Bit != Bend) && (!abortCalib) )   // for each set of joints
    {
        std::list<int>::iterator lit; //iterator for joint in a set 
        
        setOfJoint_idx++;
        currentSetList.clear();
        currentSetList = (*Bit);

        // 1) set safe pid
        for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            if ( ((*lit) <0) || ((*lit) >= nj) )   // check the axes actually exists
            {
                yError() << deviceName << "Asked to calibrate joint" << (*lit) << ", which is negative OR bigger than the number of axes for this part ("<< nj << ")";
                return false;
            }

            if(!iPids->getPid((*lit), &original_pid[(*lit)]) )
            {
                yError() << deviceName << "getPid joint " << (*lit) << "failed... aborting calibration";
                abortCalib = true;
                return false;
            }
            limited_pid[(*lit)]=original_pid[(*lit)];

            if (maxPWM[(*lit)]==0)
            {
                yDebug() << deviceName << "skipping maxPwm=0 of joint " << (*lit);
                iPids->setPid((*lit),original_pid[(*lit)]);
            }
            else
            {
                limited_pid[(*lit)].max_int=maxPWM[(*lit)];
                limited_pid[(*lit)].max_output=maxPWM[(*lit)];
                iPids->setPid((*lit),limited_pid[(*lit)]);
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
            iEncoders->getEncoders(currPos);
            yDebug() <<  deviceName  << " set" << setOfJoint_idx << "j" << (*lit) << ": Calibrating... enc values AFTER calib: " << currPos[(*lit)];
        }

        if(abortCalib)
        {
            continue;
        }

        //4) check calibration result
        if(checkCalibrateJointEnded((*Bit)) ) //check calibration on entire set
        {
            yDebug() <<  deviceName  << " set" << setOfJoint_idx  << ": Calibration ended, going to zero!\n";
        }
        else    // keep pid safe  and go on
        {
            yError() <<  deviceName  << " set" << setOfJoint_idx  << ": Calibration went wrong! Disabling axes and keeping safe pid limit\n";

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
            continue;
        }

        if(goneToZero)
        {
            yDebug() <<  deviceName  << " set" << setOfJoint_idx  << ": Reached zero position!\n";
            for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
            {
                iPids->setPid((*lit),original_pid[(*lit)]);
            }
        }
        else          // keep pid safe and go on
        {
            yError() <<  deviceName  << " set" << setOfJoint_idx  << ": some axis got timeout while reaching zero position... disabling this set of axes\n";
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
        yError() << deviceName << "calibration has been aborted!I'm going to disable all joints..." ;
        for(int i=0; i<nj; i++) //for each joint of set
        {
            iControlMode->setControlMode(i,VOCAB_CM_IDLE);
        }
        return false;
    }
    calibMutex.wait();
    isCalibrated = true;
    calibMutex.post();
    return isCalibrated;
}

void parametricCalibrator::calibrateJoint(int joint)
{
    yDebug() <<  deviceName  << ": Calling calibrateJoint on joint "<< joint << " with params: " << type[joint] << param1[joint] << param2[joint] << param3[joint];
    iCalibrate->calibrate2(joint, type[joint], param1[joint], param2[joint], param3[joint]);
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
                yWarning() << deviceName  << "CALIB: aborted\n";
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
        yDebug() << deviceName << "calib joint ended";

    return calibration_ok;
}


void parametricCalibrator::goToZero(int j)
{
    if (abortCalib) return;
    yDebug() <<  deviceName  << ": Sending positionMove to joint" << j << " (desired pos: " << zeroPos[j] << "desired speed: " << zeroVel[j] <<" )";
    iPosition->setRefSpeed(j, zeroVel[j]);
    iPosition->positionMove(j, zeroPos[j]);
}

bool parametricCalibrator::checkGoneToZeroThreshold(int j)
{
    if (skipCalibration) return false;

    // wait.
    bool finished = false;
//    double ang[4];
    double angj = 0;
//    double pwm[4];
    double delta=0;
    int mode=0;
    bool done = false;

    double start_time = yarp::os::Time::now();
    while ( (!finished) && (!abortCalib))
    {
        iEncoders->getEncoder(j, &angj);
        iPosition->checkMotionDone(j, &done);
        iControlMode->getControlMode(j, &mode);

        delta = fabs(angj-zeroPos[j]);
        yDebug() << "In calib: checkGoneToZeroThreshold "<< deviceName << "joint " << j << ": curr: " << angj << "des: " << zeroPos[j] << "-> delta: " << delta << "threshold: " << zeroPosThreshold[j]  << "mode: " << yarp::os::Vocab::decode(mode).c_str();

        if (delta < zeroPosThreshold[j] && done)
        {
            yDebug() << deviceName.c_str() << "joint " << j<< " completed with delta"  << delta << "over " << zeroPosThreshold[j];
            finished=true;
            break;
        }
        if (yarp::os::Time::now() - start_time > GO_TO_ZERO_TIMEOUT)
        {
            yError() << "In calib: checkGoneToZeroThreshold " <<  deviceName.c_str() << "joint " << j << " Timeout while going to zero!";
            break;
        }
        if (mode == VOCAB_CM_IDLE)
        {
            yError() << "In calib: checkGoneToZeroThreshold " <<  deviceName.c_str() << "joint " << j << " is idle, skipping!";
            break;
        }
        if (mode == VOCAB_CM_HW_FAULT)
        {
            yError() << "In calib: checkGoneToZeroThreshold " <<  deviceName.c_str() << "hardware fault on joint " << j << ", skipping!";
            break;
        }
        if (abortCalib)
        {
            yWarning() <<  "In calib: checkGoneToZeroThreshold " << deviceName.c_str() << " joint " << j << " Aborting wait while going to zero!\n";
            break;
        }
        Time::delay(0.5);
    }
    return finished;
}

bool parametricCalibrator::park(DeviceDriver *dd, bool wait)
{
    yTrace();
    int nj=0;
    bool ret=false;
    abortParking=false;

    calibMutex.wait();
    if(!isCalibrated)
    {
        yWarning() << "Calling park without calibration... skipping";
        calibMutex.post();
        return true;
    }
    calibMutex.post();

    if ( !iEncoders->getAxes(&nj))
    {
        yError() << deviceName << ": error getting number of encoders";
        return false;
    }

    int timeout = 0;

    int * currentControlModes = new int[nj];

    bool res = iControlMode->getControlModes(currentControlModes);
    if(!res)
    {
        yError() << deviceName << ": error getting control mode during parking";
    }

    for(int i=0; i<nj; i++)
    {
        if(currentControlModes[i] != VOCAB_CM_IDLE 
           /* && currentControlModes[i] != VOCAB_HW_FAULT */)
        {
            iControlMode->setControlMode(i,VOCAB_CM_POSITION);
        }
    }

    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);     // all joints together????
    //TODO fix checkMotionDone in such a way that does not depend on timing!
    Time::delay(0.01);
    
    if(skipCalibration)
    {
        yWarning() << deviceName << "skipCalibration flag is on!! Faking park!!";
        return true;
    }

    if (wait)
    {
        yDebug() << deviceName.c_str() << ": Moving to park positions";
        bool done=false;
        while((!done) && (timeout<PARK_TIMEOUT) && (!abortParking))
        {
            iPosition->checkMotionDone(&done);
            Time::delay(1);
            timeout++;
        }
        if(!done)
        {   // In case of error do another loop trying to detect the error!!
            for(int j=0; j < nj; j++)
            {
                if (iPosition->checkMotionDone(j, &done))
                {
                    if (!done)
                        yError() << deviceName << ", joint " << j << ": not in position after timeout";
                    // else means that axes get to the position right after the timeout.... do nothing here
                }
                else	// if the CALL to checkMotionDone fails for timeout
                    yError() << deviceName << ", joint " << j << ": did not answer during park";
            }
        }
    }

    yDebug() << "Park was " << (abortParking ? "aborted" : "done");
    yError() << "PARKING-timeout "<< deviceName.c_str() << " : "<< timeout;
    for(int j=0; j < nj; j++)
    {
        iControlMode->setControlMode(j,VOCAB_CM_IDLE);
    }
// iCubInterface is already shutting down here... so even if errors occour, what else can I do?

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

// eof

