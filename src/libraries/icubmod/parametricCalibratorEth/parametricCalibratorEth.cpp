
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2014 iCub Facility, Istituto Italiano di Tecnologia
 * Authors: Alberto Cardellino, Marco Randazzo, Valentina Gaggero
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <yarp/os/Time.h>
#include <yarp/dev/PolyDriver.h>

#include "parametricCalibratorEth.h"
#include <math.h>
#include <algorithm>

#include <yarp/os/LogStream.h>

using namespace yarp::os;
using namespace yarp::dev;

const int       PARK_TIMEOUT            = 30;
const double    GO_TO_ZERO_TIMEOUT      = 10;
const int       CALIBRATE_JOINT_TIMEOUT = 20;

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


// helper for parsing config file
bool parametricCalibratorEth::parseSequenceGroup(yarp::os::Searchable &config, std::string sequence, std::vector<PositionSequence> &seqList)
{
    Bottle parkSeq_group = config.findGroup(sequence);
    if (parkSeq_group.isNull())
    {
        // yWarning() << "parametricCalibrator " << deviceName << "Missing <" << sequence << "> group";
        return false;
    }

    Bottle xtmp;
    int numOfSeq = 0;

    if(!extractGroup(parkSeq_group, xtmp, "numberOfSequences", "number of sequences listed ", 1))
    {
        return  false;
    }

    numOfSeq = xtmp.get(1).asInt();
    // create space in vector of sequences
    seqList.resize(numOfSeq);

    if(numOfSeq < 0)
    {
        yError() << "ParametricCalibratorEth " << deviceName << "<numberOfSequences> must be a positive integer";
        return false;
    }

    // read all sequences
    for(int seq_idx=0; seq_idx<numOfSeq; seq_idx++)
    {
        char sequence_name[80];
        snprintf(sequence_name, 80, "SEQUENCE_%d", seq_idx);

        Bottle &seq_i = parkSeq_group.findGroup(sequence_name);
        if(seq_i.isNull())
        {
            yError() << "ParametricCalibratorEth " << deviceName << "cannot find " << sequence_name;
            return false;
        }

        // 1) Seq number
        seqList.at(seq_idx).seq_num = seq_idx;

        // 2) Read positions fromn config file
        Bottle & poss = seq_i.findGroup("position", "desired parking position");
        if (poss.isNull())
        {
            yError() << "ParametricCalibratorEth " << deviceName << ": <position> parameter not found for sequence " << sequence_name;
            return false;
        }

        // 3) Read velocities fromn config file
        Bottle &vels = seq_i.findGroup("velocity", "desired parking velocities");
        if (vels.isNull())
        {
            yError() << "ParametricCalibratorEth " << deviceName << ": <velocity> parameter not found for sequence " << sequence_name;
            return false;
        }

        if(( poss.size() -1 != n_joints) || (vels.size() -1 != n_joints))
        {
            yError() << "ParametricCalibratorEth " << deviceName << ": <position> or <velocity> parameter size for sequence " << sequence_name << " doesn not match the number of joint being calibrated.\n" << \
                        "Part joint number is " << n_joints << " while <position> size is " << poss.size()-1 << " and <velocity> size is " << vels.size()-1 << "; joint number is " << n_joints;
            return false;
        }

        // Store data in memory
        seqList[seq_idx].seq_num = seq_idx;
        seqList[seq_idx].positions.reserve(n_joints);
        seqList[seq_idx].velocities.reserve(n_joints);

        for (int j = 1; j <n_joints+1; j++)
        {
            seqList[seq_idx].positions .push_back(poss.get(j).asDouble());
            seqList[seq_idx].velocities.push_back(vels.get(j).asDouble());
        }
    }
    return true;
}

parametricCalibratorEth::parametricCalibratorEth() :
    calibParams(nullptr),
    original_max_pwm(nullptr),
    limited_max_pwm(nullptr),
    startupMaxPWM(nullptr),
    currPos(nullptr),
    currVel(nullptr),
//    legacyParkingPosition(0),
    startupPosThreshold(0),
    abortCalib(false),
    isCalibrated(false),
    skipCalibration(false),
    clearHwFault(false),
    n_joints(0),
    timeout_goToZero(nullptr),
    timeout_calibration(nullptr),
    disableHomeAndPark(nullptr),
    disableStartupPosCheck(nullptr),
    totJointsToCalibrate(0),
    useLegacyParking(true),
    currentParkingSeq_step(0)
{
}

parametricCalibratorEth::~parametricCalibratorEth()
{
    yTrace();
    close();
}

bool parametricCalibratorEth::open(yarp::os::Searchable& config)
{
    yTrace();
    Property p;
    p.fromString(config.toString());

    if (p.check("GENERAL")==false)
    {
      yError() << "Parametric calibrator: missing [GENERAL] section";
      return false;
    }

    if(p.findGroup("GENERAL").check("deviceName"))
    {
      deviceName = p.findGroup("GENERAL").find("deviceName").asString();
    } 
    else
    {
      yError() << "Parametric calibrator: missing deviceName parameter"; 
      return false;
    } 

    std::string str;
    if(config.findGroup("GENERAL").find("verbose").asInt())
    {
        str=config.toString().c_str();
        yTrace() << deviceName.c_str() << str;
    }  

    // Check clearHwFaultBeforeCalibration
    Value val_clearHwFault = config.findGroup("GENERAL").find("clearHwFaultBeforeCalibration");
    if(val_clearHwFault.isNull())
    {
        clearHwFault = false;
    }
    else
    {
        if(!val_clearHwFault.isBool())
        {
            yError() << deviceName.c_str() << ": clearHwFaultBeforeCalibration bool param is different from accepted values (true / false). Assuming false";
            clearHwFault = false;
        }
        else
        {
            clearHwFault = val_clearHwFault.asBool();
            if(clearHwFault)
                yInfo() << deviceName.c_str() << ":  clearHwFaultBeforeCalibration option enabled\n";
        }
    }

    // Check useRawEncoderData, it robot is using raw data, force to skip the calibration because it will be dangerous!
    Value use_raw = config.findGroup("GENERAL").find("useRawEncoderData");
    bool useRawEncoderData;

    if(use_raw.isNull())
    {
        useRawEncoderData = false;
    }
    else
    {
        if(!use_raw.isBool())
        {
            yError() << deviceName.c_str() << ": useRawEncoderData bool param is different from accepted values (true / false). Assuming false";
            useRawEncoderData = false;
        }
        else
        {
            useRawEncoderData = use_raw.asBool();
            if(useRawEncoderData)
                yWarning() << deviceName.c_str() << ":  MotionControl is using raw data from encoders! Be careful. \n" <<
                              "\t forcing to skip the calibration";
        }
    }

//    yWarning() <<  deviceName.c_str() << ": useRawEncoderData is " << useRawEncoderData;

    if(useRawEncoderData)
    {
        skipCalibration = true;
    }
    else
    {
        // Check useRawEncoderData = skip root calibration -- use with care
        Value checkSkipCalib = config.findGroup("GENERAL").find("skipCalibration");
        if(checkSkipCalib.isNull())
        {
            skipCalibration = false;
        }
        else
        {
            if(!checkSkipCalib.isBool())
            {
                yWarning() << deviceName << ": skipCalibration bool param is different from accepted values (true / false). Assuming false";
                skipCalibration = false;
            }
            else
            {
                skipCalibration = checkSkipCalib.asBool();
                if(skipCalibration)
                {
                    yWarning() << deviceName << ": skipping calibration!! This option was set in general.xml file.";
                    yWarning() << deviceName << ": BE CAREFUL USING THE ROBOT IN THIS CONFIGURATION! See 'skipCalibration' param in config file";
                } 
           }
        }
    }

    if(p.findGroup("GENERAL").check("joints"))
    {
        n_joints = p.findGroup("GENERAL").find("joints").asInt();
    }
    else if(p.findGroup("GENERAL").check("Joints"))
    {
        // This is needed to be backward compatibile with old iCubInterface
        n_joints = p.findGroup("GENERAL").find("Joints").asInt();
    }
    else
    {
        yError() << deviceName.c_str() <<  ": missing joints parameter" ;
        return false;
    }

    calibParams = new CalibrationParameters[n_joints];
    startupMaxPWM = new int[n_joints];

    legacyStartupPosition.seq_num = 0;
    legacyStartupPosition.positions.resize(n_joints);
    legacyStartupPosition.velocities.resize(n_joints);
    currPos = new double[n_joints];
    currVel = new double[n_joints];
    legacyParkingPosition.seq_num = 0;
    legacyParkingPosition.positions.resize(n_joints);
    legacyParkingPosition.velocities.resize(n_joints);


    timeout_goToZero = new int[n_joints];
    timeout_calibration = new int[n_joints];
    startupPosThreshold = new double[n_joints];
    disableHomeAndPark = new int[n_joints];
    disableStartupPosCheck = new int[n_joints];

    for (int i = 0; i < n_joints; i++) timeout_goToZero[i] = 10;
    for (int i = 0; i < n_joints; i++) timeout_calibration[i] = 20;
    for (int i = 0; i < n_joints; i++) disableHomeAndPark[i] = false;
    for (int i = 0; i < n_joints; i++) disableStartupPosCheck[i] = false;

    int i=0;

    Bottle& xtmp = p.findGroup("CALIBRATION").findGroup("calibration1");
    if (xtmp.size()-1!=n_joints) {yError() << deviceName << ": invalid number of Calibration1 params " << xtmp.size()<< " " << n_joints; return false;}
    for (i = 1; i < xtmp.size(); i++) calibParams[i-1].param1 = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration2");
    if (xtmp.size()-1!=n_joints) {yError() << deviceName << ": invalid number of Calibration2 params"; return false;}
    for (i = 1; i < xtmp.size(); i++) calibParams[i - 1].param2 = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration3");
    if (xtmp.size()-1!=n_joints) {yError() << deviceName << ": invalid number of Calibration3 params"; return false;}
    for (i = 1; i < xtmp.size(); i++) calibParams[i - 1].param3 = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration4");
    if (xtmp.size() - 1 != n_joints) { yError() << deviceName << ": invalid number of Calibration4 params"; return false; }
    for (i = 1; i < xtmp.size(); i++) calibParams[i - 1].param4 = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibration5");
    if (xtmp.size() - 1 != n_joints) { yError() << deviceName << ": invalid number of Calibration5 params"; return false; }
    for (i = 1; i < xtmp.size(); i++) calibParams[i - 1].param5 = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibrationType");
    if (xtmp.size()-1!=n_joints) {yError() <<  deviceName << ": invalid number of Calibration3 params"; return false;}
    for (i = 1; i < xtmp.size(); i++) calibParams[i - 1].type = (unsigned char)xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibrationZero");
    if (xtmp.size() - 1 != n_joints) { yError() << deviceName << ": invalid number of calibrationZero params"; return false; }
    for (i = 1; i < xtmp.size(); i++) calibParams[i - 1].paramZero = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibrationDelta");
    if (xtmp.size() - 1 != n_joints) { yError() << deviceName << ": invalid number of calibrationDelta params"; return false; }
    for (i = 1; i < xtmp.size(); i++) calibParams[i - 1].paramZero += xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("calibrationTimeout");
    if (xtmp.size() - 1 != n_joints) { } //this parameter is optional
    else { for (i = 1; i < xtmp.size(); i++) timeout_calibration[i - 1] = (int)xtmp.get(i).asDouble(); }

    xtmp = p.findGroup("CALIBRATION").findGroup("startupPosition");
    if (xtmp.size()-1!=n_joints) {yError() <<  deviceName << ": invalid number of startupPosition params"; return false;}
    for (i = 1; i < xtmp.size(); i++) legacyStartupPosition.positions[i-1] = xtmp.get(i).asDouble();

    xtmp = p.findGroup("CALIBRATION").findGroup("startupVelocity");
    if (xtmp.size()-1!=n_joints) {yError() <<  deviceName << ": invalid number of startupVelocity params"; return false;}
    for (i = 1; i < xtmp.size(); i++) legacyStartupPosition.velocities[i - 1] = xtmp.get(i).asDouble();

    // First find new version of parking sequence. Optional right now for back compatibility
    useLegacyParking = ! parseSequenceGroup(p, "PARKING_SEQUENCE", parkingSequence);

    Bottle homeGroup = p.findGroup("HOME");
    if(useLegacyParking)
    {
        if( homeGroup.isNull())
        {
            yError() << "Parking position not found. Either <HOME> or <PARKING_SEQUENCE> must be specified in config file";
            return false;
        }
        /*
	else
        {
            yWarning() << "<HOME> group is deprecated in favour of <PARKING_SEQUENCE>";
        }
        */
        xtmp = homeGroup.findGroup("positionHome");
        if (xtmp.size()-1!=n_joints) {yError() <<  deviceName << ": invalid number of PositionHome params"; return false;}
        legacyParkingPosition.positions.resize(n_joints);
        for (i = 1; i < xtmp.size(); i++)
            legacyParkingPosition.positions[i-1] = xtmp.get(i).asDouble();

        xtmp = homeGroup.findGroup("velocityHome");
        if (xtmp.size()-1!=n_joints) {yError() <<  deviceName << ": invalid number of VelocityHome params"; return false;}
        legacyParkingPosition.velocities.resize(n_joints);
        for (i = 1; i < xtmp.size(); i++)
            legacyParkingPosition.velocities[i-1] = xtmp.get(i).asDouble();
    }

    // this parameter may be superseded by new park sequence mechanism, probably also for startup.
    xtmp = homeGroup.findGroup("disableHomeAndPark");
    if (xtmp.size() - 1 != n_joints) { } //this parameter is optional
    else { for (i = 1; i < xtmp.size(); i++) disableHomeAndPark[i - 1] = xtmp.get(i).asInt(); }

    xtmp = p.findGroup("CALIBRATION").findGroup("startupMaxPwm");
    if (xtmp.size()-1!=n_joints) {yError() <<  deviceName << ": invalid number of startupMaxPwm params"; return false;}
    for (i = 1; i < xtmp.size(); i++) startupMaxPWM[i-1] =  xtmp.get(i).asInt();

    xtmp = p.findGroup("CALIBRATION").findGroup("startupPosThreshold");
    if (xtmp.size()-1!=n_joints) {yError() <<  deviceName << ": invalid number of startupPosThreshold params"; return false;}
    for (i = 1; i < xtmp.size(); i++) startupPosThreshold[i-1] =  xtmp.get(i).asDouble();
 
    xtmp = p.findGroup("CALIBRATION").findGroup("startupDisablePosCheck");
    if (xtmp.size() - 1 != n_joints) { } //this parameter is optional
    else { for (i = 1; i < xtmp.size(); i++) disableStartupPosCheck[i - 1] = xtmp.get(i).asInt(); }
   
    xtmp = p.findGroup("CALIBRATION").findGroup("startupTimeout");
    if (xtmp.size() - 1 != n_joints) {} //this parameter is optional
    else { for (i = 1; i < xtmp.size(); i++) timeout_goToZero[i - 1] = xtmp.get(i).asDouble(); }

    xtmp = p.findGroup("CALIB_ORDER");
    int calib_order_size = xtmp.size();
    if (calib_order_size <= 1) {yError() << deviceName << ": invalid number CALIB_ORDER params"; return false;}
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

bool parametricCalibratorEth::close ()
{
    yTrace();
    if (calibParams != NULL) {
        delete[] calibParams;
        calibParams = NULL;
    }

    if (startupMaxPWM != NULL) {
        delete[] startupMaxPWM;
        startupMaxPWM = NULL;
    }
    if (original_max_pwm != NULL) {
        delete[] original_max_pwm;
        original_max_pwm = NULL;
    }
    if (limited_max_pwm != NULL) {
        delete[] limited_max_pwm;
        limited_max_pwm = NULL;
    }

    if (timeout_goToZero != NULL) {
        delete[] timeout_goToZero;
        timeout_goToZero = NULL;
    }

    if (timeout_calibration != NULL) {
        delete[] timeout_calibration;
        timeout_calibration = NULL;
    }

    if (currPos != NULL) {
        delete[] currPos;
        currPos = NULL;
    }
    if (currVel != NULL) {
        delete[] currVel;
        currVel = NULL;
    }

    if (disableHomeAndPark != NULL) {
        delete[] disableHomeAndPark;
        disableHomeAndPark = NULL;
    }

    if (disableStartupPosCheck != NULL) {
        delete[] disableStartupPosCheck;
        disableStartupPosCheck = NULL;
    }

    return true;
}

bool parametricCalibratorEth::calibrate(DeviceDriver *device)
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

bool parametricCalibratorEth::calibrate()
{
    int  setOfJoint_idx = 0;
    totJointsToCalibrate = 0;
    calibJoints.clear();

    if (dev2calibrate==0)
    {
        yError() << deviceName << ": not able to find a valid device to calibrate";
        return false;
    }

    int n_joints_board{0};
    if ( !iEncoders->getAxes(&n_joints_board))
    {
        yError() << deviceName << ": error getting number of axes" ;
        return false;
    }
    if(n_joints_board != n_joints)
    {
        yError() << "ParametricCalibratorEth: " << deviceName << ": number of joints of device to calibrate (" << n_joints_board << \
                                                                 ") does not match the number of joints in calibrator config file ("<< n_joints << ")";
        return false;
    }

    std::list<int>  currentSetList;
    std::list<std::list<int> >::iterator Bit=joints.begin();
    std::list<std::list<int> >::iterator Bend=joints.end();

    // count how many joints there are in the list of things to be calibrated
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
        yError() << deviceName << ": too much axis to calibrate for this part..." << totJointsToCalibrate << " bigger than " << n_joints;
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
            if ( ((*lit) <0) || ((*lit) >= n_joints) )   // check the axes actually exists
            {
                yError() << deviceName << ": asked to calibrate joint" << (*lit) << ", which is negative OR bigger than the number of axes for this part ("<< n_joints << ")";
                abortCalib = true;
                break;
            }

            if(!iAmp->getPWMLimit((*lit), &original_max_pwm[(*lit)]) )
            {
                yError() << deviceName << ": getPid joint " << (*lit) << "failed... aborting calibration";
                abortCalib = true;
                break;
            }

            limited_max_pwm[(*lit)] = original_max_pwm[(*lit)];

            if (startupMaxPWM[(*lit)]==0)
            {
                yDebug() << deviceName << ": skipping startupMaxPWM=0 of joint " << (*lit);
                iAmp->setPWMLimit((*lit),original_max_pwm[(*lit)]);
            }
            else
            {
                if (startupMaxPWM[(*lit)]<limited_max_pwm[(*lit)])
                {
                    limited_max_pwm[(*lit)]=startupMaxPWM[(*lit)];
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
        //      because enablePid doesn't send command because joints are not calibrated

        /*for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            if (type[*lit]==0 ||
                type[*lit]==2 ||
                type[*lit]==4 ) 
            {
                yDebug() << "In calibration " <<  deviceName  << ": enabling joint " << *lit << " to test hardware limit";
                iControlMode->setControlMode((*lit), VOCAB_CM_POSITION);
            }
        }*/
        
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

            for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
            {
               iControlMode->setControlMode((*lit),VOCAB_CM_IDLE);
            }
            Bit++;
            continue; //go to next set
        }

        // 5) if calibration finish with success enable disabled joints in order to move them to zero
        /*for(lit  = currentSetList.begin(); lit != currentSetList.end() && !abortCalib; lit++) //for each joint of set
        {
            // if the joint han not been enabled at point 1, now i enable it 
            //iAmps->enableAmp((*lit));
            if (type[*lit]!=0 &&
                type[*lit]!=2 &&
                type[*lit]!=4 ) 
            {
                iControlMode->setControlMode((*lit), VOCAB_CM_POSITION);
            }
        }*/

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
            goToStartupPosition((*lit));
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
    isCalibrated = true;
    return isCalibrated;
}

bool parametricCalibratorEth::calibrateJoint(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'calibration' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }
    yDebug() << deviceName << ": Calling calibrateJoint on joint " << j << ": type "<< calibParams[j].type << " with params: " << calibParams[j].param1 << calibParams[j].param2 << calibParams[j].param3 << calibParams[j].param4 << calibParams[j].param5;
    bool b = iCalibrate->setCalibrationParameters(j, calibParams[j]);
    return b;
}

bool parametricCalibratorEth::checkCalibrateJointEnded(std::list<int> set)
{
    bool calibration_ok = true;
    int timeout = 0;

    std::list<int>::iterator lit;
    std::list<int>::iterator lend;

    lit = set.begin();
    lend = set.end();
    
    while (lit != lend)
    {
        if (abortCalib)
        {
            yWarning() << deviceName << ": calibration aborted\n";
        }

        if (iCalibrate->calibrationDone(*lit))
        {
            yDebug() << deviceName << ": calib joint " << (*lit) << "ended";
            lit++;
            timeout = 0;
        }
        else
        {
            if (timeout > timeout_calibration[*lit])
            {
                yError() << deviceName << ": Timeout while calibrating " << (*lit);
                calibration_ok = false;
                lit++;
                timeout = 0;
            }
            
            yarp::os::Time::delay(1.0);
            timeout++;
        }
    }

    return calibration_ok;
}

bool parametricCalibratorEth::checkHwFault(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'check hardware fault' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
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

// this function may be updated in the future to use a startup sequence, like parking
bool parametricCalibratorEth::goToStartupPosition(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'go to zero' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    bool ret = true;
    if (disableStartupPosCheck[j])
    {
        yWarning() << deviceName << ": goToZero, joint " << j << " is disabled on user request";
        return true;
    }

    if (abortCalib) return true;
    yDebug() <<  deviceName  << ": Sending positionMove to joint" << j << " (desired pos: " << legacyStartupPosition.positions[j] << \
                                "desired speed: " << legacyStartupPosition.velocities[j] <<" )";
    ret = iPosition->setRefSpeed(j, legacyStartupPosition.velocities[j]);
    ret &= iPosition->positionMove(j, legacyStartupPosition.positions[j]);
    return ret;
}

bool parametricCalibratorEth::checkGoneToZeroThreshold(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'check gone to zero' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    if (disableStartupPosCheck[j])
    {
        yWarning() << deviceName << ": checkGoneToZeroThreshold, joint " << j << " is disabled on user request";
        return true;
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
        iPids->getPidOutput(VOCAB_PIDTYPE_POSITION,j, &output);
        
        delta = fabs(angj-legacyStartupPosition.positions[j]);
        yDebug("%s: checkGoneToZeroThreshold: joint: %d curr: %.3f des: %.3f -> delta: %.3f threshold: %.3f output: %.3f mode: %s" , \
               deviceName.c_str(), j, angj, legacyStartupPosition.positions[j], delta, startupPosThreshold[j], output, yarp::os::Vocab::decode(mode).c_str());

        if (delta < startupPosThreshold[j] && done)
        {
            yDebug("%s: checkGoneToZeroThreshold: joint: %d completed with delta: %.3f over: %.3f" ,deviceName.c_str(),j,delta, startupPosThreshold[j]);
            finished=true;
            break;
        }
        if (yarp::os::Time::now() - start_time > timeout_goToZero[j])
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

// called by robotinterface (during interrupt action??)  // done
bool parametricCalibratorEth::park(DeviceDriver *dd, bool wait)
{
    // parameter device driver is not used, because we already stored and got interfaces view
    // when function 'calibration' was called.
    yTrace();
    std::list<int>::iterator joint;
 
    // legacy version: can be removed when legacy will not be supported anymore
    if (useLegacyParking)
    {
        bool allJointsCanParkSimultaneously = true;
        for (int i = 0; i < n_joints; i++)
        {
            if (disableHomeAndPark[i]) allJointsCanParkSimultaneously = false;
        }

        if(allJointsCanParkSimultaneously == false)
        {
            yWarning() << deviceName << "Joints will be parked separately, since some of them have the disableHomeAndPark flag set";
            bool ret = true;
            for(joint  = calibJoints.begin(); joint != calibJoints.end() && !abortCalib; joint++) //for each joint of set
            {
                ret &= this->parkSingleJoint(*joint);
            }
            return ret;
        }
    }
    abortParking=false;

    if(!isCalibrated)
    {
        yWarning() << deviceName << ": Calling park without calibration... skipping";
        return true;
    }

    if(skipCalibration)
    {
        yWarning() << deviceName << ": skipCalibration flag is on!! Faking park!!";
        return true;
    }

    int * currentControlModes = new int[n_joints];
    std::vector<bool> cannotPark(n_joints);
    bool res = iControlMode->getControlModes(currentControlModes);
    if(!res)
    {
        yError() << deviceName << ": error getting control mode during parking";
    }

    for(joint  = calibJoints.begin(); joint != calibJoints.end() && !abortCalib; joint++) //for each joint of set
    {
        switch(currentControlModes[(*joint)])
        {
            case VOCAB_CM_IDLE:
            {
                yError() << deviceName << ": joint " << (*joint) << " is idle, skipping park";
                cannotPark[(*joint)] = true;
            }
            break;

            case VOCAB_CM_HW_FAULT:
            {
                yError() << deviceName << ": joint " << (*joint) << " has an hardware fault, skipping park";
                cannotPark[(*joint)] = true;
            }
            break;

            case VOCAB_CM_NOT_CONFIGURED:
            {
                yError() << deviceName << ": joint " << (*joint) << " is not configured, skipping park";
                cannotPark[(*joint)] = true;
            }
            break;

            case VOCAB_CM_UNKNOWN:
            {
                yError() << deviceName << ": joint " << (*joint) << " is in unknown state, skipping park";
                cannotPark[(*joint)] = true;
            }

            default:
            {
                iControlMode->setControlMode((*joint), VOCAB_CM_POSITION);
                cannotPark[(*joint)] = false;
            }
        }
    }

    bool parkSequenceDone{false};
    if(useLegacyParking)
    {
        moveAndCheck_legacy(legacyParkingPosition, cannotPark, wait);
        // for legacy version, parkSequenceDone is always true, because there is no sequence
        parkSequenceDone = true;
    }
    else
    {
        // call one step of parking sequence for each 'park' call
 
	bool stepDone = true;
	if(parkingSequence.size() > 0)
	{
	    stepDone = moveAndCheck(parkingSequence.at(currentParkingSeq_step));
        }
        if(stepDone)
        {
            yDebug() << "ParametricCalibratorEth: park sequence step num " << currentParkingSeq_step << " ended with  " << \
                                                                              (abortParking ? "failure" : "success");
        }
        else
            abortParking = true;

        currentParkingSeq_step++;
        // parking sequence is completed if all steps are completed, or aborted if any one goes bad.
        // It is not safe to continue, if one parking step fails
        if( (currentParkingSeq_step >= parkingSequence.size()) || stepDone == false)
            parkSequenceDone = true;
    }

    // when parking is done (all steps of sequence where is the case), set all joints in idle
    if(parkSequenceDone)
    {
        yDebug() << deviceName.c_str() << ": Park " << (abortParking ? "aborted" : "completed");
        for(joint  = calibJoints.begin(); joint != calibJoints.end() && !abortCalib; joint++) //for each joint of set
        {
            switch(currentControlModes[(*joint)])
            {
                case VOCAB_CM_IDLE:
                case VOCAB_CM_HW_FAULT:
                case VOCAB_CM_NOT_CONFIGURED:
                case VOCAB_CM_UNKNOWN:
                    // Do nothing.
                    break;
                default:
                {
                    iControlMode->setControlMode((*joint), VOCAB_CM_IDLE);
                }
            }
        }
    }
    return true;
}

bool parametricCalibratorEth::moveAndCheck(PositionSequence &data)
{
    iPosition->setRefSpeeds(data.velocities.data());
    iPosition->positionMove(data.positions.data());

    bool done    = false;
    int  timeout = 0;
    do
    {
        Time::delay(1);
        timeout++;
        iPosition->checkMotionDone(&done);
    }
    while((!done) && (timeout < PARK_TIMEOUT) && (!abortParking));

    if(!done)
    {
        yError() << "ParametricCalibratorEth: parking " << deviceName << " not in position after a timeout of" << PARK_TIMEOUT << " seconds";
    }

    return done;
}

bool parametricCalibratorEth::moveAndCheck_legacy(PositionSequence &data, std::vector<bool> &cannotPark, bool wait)
{
    bool done=false;
    // send references to joints that need to be parked
    for(auto joint  = calibJoints.begin(); joint != calibJoints.end() && !abortCalib; joint++) //for each joint of set
    {
        if (cannotPark[(*joint)] ==false)
        {
            iPosition->setRefSpeed((*joint), data.velocities[(*joint)]);
            iPosition->positionMove((*joint), data.positions[(*joint)]);
        }
    }

    // wait for the parking to be completed
    if (wait)
    {
        for(auto joint  = calibJoints.begin(); joint != calibJoints.end() && !abortCalib; joint++) //for each joint of set
        {
            int timeout = 0;
            if (cannotPark[(*joint)] ==false)
            {
                yDebug() << deviceName.c_str() << ": Moving to park position, joint:" << (*joint);
                done=false;
                iPosition->checkMotionDone((*joint), &done);
                while ((!done) && (timeout < PARK_TIMEOUT) && (!abortParking))
                {
                    Time::delay(1);
                    timeout++;
                    iPosition->checkMotionDone((*joint), &done);
                }
                if(!done)
                {
                    yError() << deviceName << ": joint " << (*joint) << " not in position after a timeout of" << PARK_TIMEOUT << " seconds";
                }
            }
        }
    }
    return done;
}

bool parametricCalibratorEth::quitCalibrate()
{
    yDebug() << deviceName.c_str() << ": Quitting calibrate\n";
    abortCalib = true;
    return true;
}

bool parametricCalibratorEth::quitPark()
{
    yDebug() << deviceName.c_str() << ": Quitting parking\n";
    abortParking=true;
    return true;
}

yarp::dev::IRemoteCalibrator *parametricCalibratorEth::getCalibratorDevice()
{
    return this;
}

bool parametricCalibratorEth::calibrateSingleJoint(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'calibration' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    return calibrateJoint(j);
}

bool parametricCalibratorEth::calibrateWholePart()
{
    yTrace();
    return calibrate();
}

bool parametricCalibratorEth::homingSingleJoint(int j)
{
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'homing' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    if (disableHomeAndPark[j])
    {
        yWarning() << deviceName << ": homingSingleJoint, joint " << j << " is disabled on user request";
        return true;
    }
    return goToStartupPosition(j);
}

bool parametricCalibratorEth::homingWholePart()
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

bool parametricCalibratorEth::parkSingleJoint(int j, bool _wait)
{
    // check input joint number is valid
    if(std::find(calibJoints.begin(), calibJoints.end(), j) == calibJoints.end())
    {
        yError("%s cannot perform 'park' operation because joint number %d is out of range [%s].", deviceName.c_str(), j, calibJointsString.toString().c_str());
        return false;
    }

    if(useLegacyParking) // legacy version: can be removed when legacy will not be supported anymore
    {
        if (disableHomeAndPark[j])
        {
            yWarning() << deviceName << ": parkSingleJoint, joint " << j << " is disabled on user request";
            return true;
        }
    }

    abortParking=false;

    if(!isCalibrated)
    {
        yWarning() << deviceName << ": Calling park without calibration... skipping";
        return true;
    }

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

    if(useLegacyParking) // legacy version: can be removed when legacy will not be supported anymore
    {
        iPosition->setRefSpeed (j, legacyStartupPosition.velocities[j]);
        iPosition->positionMove(j, legacyStartupPosition.positions[j]);
    }
    else
    {
        // Send the position and velocities of the last sequence step
        iPosition->setRefSpeed (j, parkingSequence.rbegin()->velocities[j]);
        iPosition->positionMove(j, parkingSequence.rbegin()->positions[j]);
    }

    if (_wait)
    {
        if (cannotPark == false)
        {
            int timeout = 0;
            yDebug() << deviceName.c_str() << ": Moving to park position, joint:" << j;
            bool done=false;
            iPosition->checkMotionDone(j, &done);
            while ((!done) && (timeout < PARK_TIMEOUT) && (!abortParking))
            {
                Time::delay(1);
                timeout++;
                iPosition->checkMotionDone(j, &done);
            }
            if(!done)
            {
                yError() << deviceName << ": joint " << j << " not in position after a timeout of" << PARK_TIMEOUT << " seconds";
            }
        }
    }

    yDebug() << deviceName.c_str() << ": Park " << (abortParking ? "aborted" : "completed");
    iControlMode->setControlMode(j,VOCAB_CM_IDLE);
    return true;
}

// called from motorgui or remote devices
bool parametricCalibratorEth::parkWholePart()
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

