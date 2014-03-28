// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2006 Giorgio Metta, Lorenzo Natale
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
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
//    yDebug() << "Group size " << xtmp.size() << "\nvalues: " << xtmp.toString().c_str();
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

bool parametricCalibrator::calibrate(DeviceDriver *dd)  // dd dovrebbe essere il wrapper, non mc
{
    yDebug() << deviceName << "Entering parametricCalibrator::calibrate()";
    yTrace();
    abortCalib  = false;
    bool goHome_ok = true;
    int  setOfJoint_idx = 0;

    int nj=0;
    int totJointsToCalibrate = 0;

    yarp::dev::PolyDriver *p = dynamic_cast<yarp::dev::PolyDriver *>(dd);
    p->view(iCalibrate);
    p->view(iAmps);
    p->view(iEncoders);
    p->view(iPosition);
    p->view(iPids);
    p->view(iControlMode);

    if (!(iCalibrate && iAmps && iEncoders && iPosition && iPids && iControlMode)) {
        yError() << deviceName << ": interface not found" << iCalibrate << iAmps << iPosition << iPids << iControlMode;
        return false;
    }

    if ( !iEncoders->getAxes(&nj))
    {
        yError() << deviceName << "CALIB: error getting number of encoders" ;
        return false;
    }

// ok we have all interfaces


    int a = joints.size();
//     printf("List of list size %d\n", a);

    std::list<int>  tmp;

    std::list<std::list<int> >::iterator Bit=joints.begin();
    std::list<std::list<int> >::iterator Bend=joints.end();

    std::list<int>::iterator lit;
    std::list<int>::iterator lend;

// count how many joints are there in the list of things to be calibrated
    while(Bit != Bend)
    {
        tmp.clear();
        tmp = (*Bit);
        lit  = tmp.begin();
        lend = tmp.end();
        totJointsToCalibrate += tmp.size();

        printf("Joints calibration order :\n");

        while(lit != lend)
        {
            printf("%d,", (*lit));
            lit++;
        }
        printf("\n");
        Bit++;
    }

    if (totJointsToCalibrate > nj)
    {
        yError() << deviceName << ": too much axis to calibrate for this part..." << totJointsToCalibrate << " bigger than "<< nj;
        return false;
    }

    original_pid=new Pid[nj];
    limited_pid =new Pid[nj];

    if(skipCalibration)
        yWarning() << deviceName << "Vanilla flag is on!! Did the set safe pid but skipping calibration!!";
    else
        yWarning() << deviceName << "\n\nGoing to calibrate!!!!\n\n";

    Bit=joints.begin();
    while( (Bit != Bend) && (!abortCalib) )   // per ogni set di giunti
    {
        setOfJoint_idx++;
        tmp.clear();
        tmp = (*Bit);

        lit  = tmp.begin();
        lend = tmp.end();
        while( (lit != lend) && (!abortCalib) )     // per ogni giunto del set
        {
            if ( ((*lit) <0) || ((*lit) >= nj) )   // check the axes actually exists
            {
                yError() << deviceName << "Asked to calibrate joint" << (*lit) << ", which is negative OR bigger than the number of axes for this part ("<< nj << ")";
                return false;
            }

            if(!iPids->getPid((*lit), &original_pid[(*lit)]) )
            {
                yError() << deviceName << "getPid joint " << (*lit) << "failed... aborting calibration";
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
            
            lit++;
        }

        //
        // Calibrazione
        //

        if(skipCalibration)     // if this flag is on, fake calibration
        {
            Bit++;
            continue;
        }

        //------------------------------------------------
        //enable only the motors which have to test the hardware limit
        for(lit  = tmp.begin(); lit != lend; lit++)  
        {
            if (type[*lit]==0 ||
                type[*lit]==2 ||
                type[*lit]==4 ) 
            {
                yDebug() <<  deviceName  << "Enabling joint " << *lit << " to test hardware limit";
                iAmps->enableAmp(*lit); 
                iPids->enablePid(*lit);
            }
        }
        //------------------------------------------------

        Time::delay(0.1f);

        //------------------------------------------------
        for(lit  = tmp.begin(); lit != lend; lit++)      // per ogni giunto del set
        {
            // Enable amp moved into EMS class;
            // Here we just call the calibration procedure
            calibrateJoint((*lit));
        }

        Time::delay(0.1f);

        for(lit  = tmp.begin(); lit != lend; lit++)      // per ogni giunto del set
        {
            iEncoders->getEncoders(currPos);
            yDebug() <<  deviceName  << " set" << setOfJoint_idx << "j" << (*lit) << ": Calibrating... enc values AFTER calib: " << currPos[(*lit)];
        }

        if(checkCalibrateJointEnded((*Bit)) )
        {
            yDebug() <<  deviceName  << " set" << setOfJoint_idx  << ": Calibration ended, going to zero!\n";
        }
        else    // keep pid safe  and go on
        {
            yError() <<  deviceName  << " set" << setOfJoint_idx  << "j" << (*lit) << ": Calibration went wrong! Disabling axes and keeping safe pid limit\n";
            for(lit  = tmp.begin(); lit != tmp.end() && !abortCalib; lit++)   // per ogni giunto del set
            {
                iAmps->disableAmp((*lit));
            }
        }
        //------------------------------------------------

        //------------------------------------------------
        yDebug() <<  deviceName  << "Enabling PWM";
        for(lit  = tmp.begin(); lit != tmp.end() && !abortCalib; lit++)   // per ogni giunto del set
        {
            if (type[*lit]!=0 &&
                type[*lit]!=2 &&
                type[*lit]!=4 ) 
            {
                iAmps->enableAmp((*lit));
                iPids->enablePid((*lit));
                iControlMode->setPositionMode((*lit));
            }
        }
        yDebug() <<  deviceName  <<  "Enabling PWM complete";
        //------------------------------------------------

        Time::delay(0.5f);    // needed?

        //------------------------------------------------
        yDebug() <<  deviceName  << "Sending goToZero commands";
        lit  = tmp.begin();
        while(lit != lend) 
        {
            goToZero((*lit));
            lit++;
        }
        yDebug() <<  deviceName  << "Sending goToZero commands complete";
        //------------------------------------------------

        Time::delay(1.0);     // needed?

        bool goneToZero = true;
        lit  = tmp.begin();
        while(lit != lend)    // per ogni giunto del set
        {
            yDebug() << " joint" << (*lit) << " using encoder";
            goneToZero &= checkGoneToZeroThreshold(*lit);   // BLL style, use encoder position
            lit++;
        }

        if(goneToZero)
        {
            yDebug() <<  deviceName  << " set" << setOfJoint_idx  << ": Reached zero position!\n";
            for(lit  = tmp.begin(); lit != tmp.end() && !abortCalib; lit++)   // per ogni giunto del set
            {
                iPids->setPid((*lit),original_pid[(*lit)]);
            }
        }
        else          // keep pid safe and go on
        {
            //The following line causes segfault and it has been commented out. Please fix it.
            //yError() <<  deviceName  << " set" << setOfJoint_idx  << "j" << (*lit) << ": some axis got timeout while reaching zero position... disabling this set of axes (*here joint number is wrong, it's quite harmless and useless to print but I want understand ` it is wrong.\n";
            yError() <<  deviceName  << " set" << setOfJoint_idx  << "j" << "(*lit)" << ": some axis got timeout while reaching zero position... disabling this set of axes (*here joint number is wrong, it's quite harmless and useless to print but I want understand ` it is wrong.\n";
            for(lit  = tmp.begin(); lit != tmp.end() && !abortCalib; lit++)   // per ogni giunto del set
            {
                iAmps->disableAmp((*lit));
            }
        }

        // Go to the next set of joints to calibrate... if any
        Bit++;
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

// Not used anymore... EMS knows wath to do. Just ask if motion is done!! ^_^
bool parametricCalibrator::checkGoneToZeroThreshold(int j)
{
    // wait.
    bool finished = false;
//    double ang[4];
    double angj = 0;
//    double pwm[4];
    double delta=0;
    bool done = false;

    double start_time = yarp::os::Time::now();
    while ( (!finished) && (!abortCalib))
    {
        iEncoders->getEncoder(j, &angj);
        iPosition->checkMotionDone(j, &done);

        delta = fabs(angj-zeroPos[j]);
        yDebug() << deviceName << "joint " << j << ": curr: " << angj << "des: " << zeroPos[j] << "-> delta: " << delta << "threshold " << zeroPosThreshold[j];

        if (delta < zeroPosThreshold[j] && done)
        {
            yDebug() << deviceName.c_str() << "joint " << j<< " completed with delta"  << delta << "over " << zeroPosThreshold[j];
            finished=true;
            break;
        }

        if (yarp::os::Time::now() - start_time > GO_TO_ZERO_TIMEOUT)
        {
            yError() <<  deviceName.c_str() << "joint " << j << " Timeout while going to zero!";
            break;
        }
        if (abortCalib)
        {
            yWarning() <<  deviceName.c_str() << " joint " << j << " Abort wait while going to zero!\n";
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

    iPosition->setPositionMode();
    iPosition->setRefSpeeds(homeVel);
    iPosition->positionMove(homePos);     // all joints together????
    //TODO fix checkMotionDone in such a way that does not depend on timing!
    Time::delay(0.01);
    
    if(skipCalibration)
    {
        yWarning() << deviceName << "Vanilla flag is on!! Faking park!!";
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
    	iAmps->disableAmp(j);
    	iPids->disablePid(j);
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

