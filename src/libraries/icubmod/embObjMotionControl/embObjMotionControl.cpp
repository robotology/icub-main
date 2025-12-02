// -*- Mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
* Authors: Alberto Cardellino
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/

//#include <yarp/dev/CanBusInterface.h>
// system std include
#include <string.h>
#include <iostream>
#include <cmath>

// yarp include
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include "embObjMotionControl.h"
#include <ethManager.h>
#include <FeatureInterface.h>
#include <yarp/conf/environment.h>

#include <yarp/os/LogStream.h>

#include <yarp/os/NetType.h>
#include <yarp/dev/ControlBoardHelper.h>

 #include <yarp/dev/ReturnValue.h>

// local include
#include "EoCommon.h"
#include "EOarray.h"
#include "EoProtocol.h"
#include "EoProtocolMN.h"
#include "EoProtocolMC.h"
#include "EoProtocolAS.h"
#include "motionControlDefaultValues.h"

#include "eomcUtils.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::os::impl;



using namespace yarp::dev::eomc;


// macros
#define ASK_REFERENCE_TO_FIRMWARE 1

#define PARSER_MOTION_CONTROL_VERSION   6




#define NV_NOT_FOUND    return nv_not_found();

static bool nv_not_found(void)
{
    yError () << " nv_not_found!! This may mean that this variable is not handled by this EMS\n";
    return false;
}

//static constexpr double const temperatureErrorValue_s = -5000;

std::string embObjMotionControl::getBoardInfo(void)
{
    if(nullptr == res)
    {
        return " BOARD name_unknown (IP unknown) ";
    }
    else
    {
        return ("BOARD " + res->getProperties().boardnameString +  " (IP "  + res->getProperties().ipv4addrString + ") ");
    }
}


bool embObjMotionControl::alloc(int nj)
{
    _axisMap = allocAndCheck<int>(nj);

    _encodersStamp = allocAndCheck<double>(nj);
    _gearbox_M2J = allocAndCheck<double>(nj);
    _gearbox_E2J = allocAndCheck<double>(nj);
    _deadzone = allocAndCheck<double>(nj);
    _foc_based_info= allocAndCheck<eomc::focBasedSpecificInfo_t>(nj);
    _trj_pids= new eomc::PidInfo[nj];
    _dir_pos_pids= new eomc::PidInfo[nj];
    _dir_vel_pids= new eomc::PidInfo[nj];
    _trq_pids= new eomc::TrqPidInfo [nj];
    _cur_pids= new eomc::PidInfo[nj];
    _vel_pids= new eomc::PidInfo[nj];
    _impedance_limits=allocAndCheck<eomc::impedanceLimits_t>(nj);
    checking_motiondone=allocAndCheck<bool>(nj);
    _last_position_move_time=allocAndCheck<double>(nj);

    // Reserve space for data stored locally. values are initialize to 0
    _ref_command_positions = allocAndCheck<double>(nj);
    _ref_positions = allocAndCheck<double>(nj);
    _ref_command_speeds = allocAndCheck<double>(nj);
    _ref_speeds = allocAndCheck<double>(nj);
    _ref_accs = allocAndCheck<double>(nj);

    _enabledAmp = allocAndCheck<bool>(nj);
    _enabledPid = allocAndCheck<bool>(nj);
    _calibrated = allocAndCheck<bool>(nj);
    _cacheImpedance = allocAndCheck<eOmc_impedance_t>(nj);

    _rotorsLimits.resize(nj);
    _jointsLimits.resize(nj);
    _currentLimits.resize(nj);
    _temperatureLimits.resize(nj);
    _jsets.resize(nj);
    _joint2set.resize(nj);
    _timeouts.resize(nj);
    _impedance_params.resize(nj);
    _lugre_params.resize(nj);
    _axesInfo.resize(nj);
    _jointEncs.resize(nj);
    _motorEncs.resize(nj);
    _kalman_params.resize(nj);
    _temperatureSensorsVector.resize(nj);
    _temperatureExceededLimitWatchdog.resize(nj);
    _temperatureSensorErrorWatchdog.resize(nj); 
    _temperatureSpikesFilter.resize(nj);
    
    // update threshold for watchdog parametrized on the ROP transmission rate (by default is 2ms)
    uint8_t txrate = res->getProperties().txROPratedivider;
    for(int i = 0; i < nj; ++i)
    {
        _temperatureExceededLimitWatchdog.at(i).setThreshold(txrate);
        _temperatureSensorErrorWatchdog.at(i).setThreshold(txrate);
    }
    
    return true;
}

bool embObjMotionControl::dealloc()
{
    checkAndDestroy(_axisMap);
    checkAndDestroy(_encodersStamp);
    checkAndDestroy(_gearbox_M2J);
    checkAndDestroy(_gearbox_E2J);
    checkAndDestroy(_deadzone);
    checkAndDestroy(_impedance_limits);
    checkAndDestroy(checking_motiondone);
    checkAndDestroy(_ref_command_positions);
    checkAndDestroy(_ref_positions);
    checkAndDestroy(_ref_command_speeds);
    checkAndDestroy(_ref_speeds);
    checkAndDestroy(_ref_accs);

    checkAndDestroy(_enabledAmp);
    checkAndDestroy(_enabledPid);
    checkAndDestroy(_calibrated);
    checkAndDestroy(_foc_based_info);

    if(_trj_pids)
        delete [] _trj_pids;

    if(_dir_pos_pids)
        delete [] _dir_pos_pids;

    if(_dir_vel_pids)
        delete [] _dir_vel_pids;

    if(_trq_pids)
        delete [] _trq_pids;

    if(_cur_pids)
        delete [] _cur_pids;

    if (_vel_pids)
        delete[] _vel_pids;


    return true;
}

embObjMotionControl::embObjMotionControl() :
    ImplementControlCalibration(this),
    ImplementAmplifierControl(this),
    ImplementPidControl(this),
    ImplementEncodersTimed(this),
    ImplementPositionControl(this),
    ImplementVelocityControl(this),
    ImplementVelocityDirect(this),
    ImplementControlMode(this),
    ImplementImpedanceControl(this),
    ImplementMotorEncoders(this),
#ifdef IMPLEMENT_DEBUG_INTERFACE
    ImplementDebugInterface(this),
#endif
    ImplementTorqueControl(this),
    ImplementControlLimits(this),
    ImplementPositionDirect(this),
    ImplementInteractionMode(this),
    ImplementMotor(this),
    ImplementRemoteVariables(this),
    ImplementAxisInfo(this),
    ImplementPWMControl(this),
    ImplementCurrentControl(this),
    ImplementJointFault(this),
    SAFETY_THRESHOLD(2.0),
    _rotorsLimits(0),
    _jointsLimits(0),
    _currentLimits(0),
    _temperatureLimits(0),
    _jsets(0),
    _joint2set(0),
    _timeouts(0),
    _impedance_params(0),
    _lugre_params(0),
    _axesInfo(0),
    _jointEncs(0),
    _motorEncs(0),
    _kalman_params(0),
    _temperatureSensorsVector(0),
    _temperatureExceededLimitWatchdog(0),
    _temperatureSensorErrorWatchdog(0),
    _temperatureSpikesFilter(0),
    _rawDataAuxVector(0),
    _rawValuesMetadataMap({})
{
    _gearbox_M2J  = 0;
    _gearbox_E2J  = 0;
    _deadzone     = 0;
    opened        = 0;
    _trj_pids     = NULL;
    _dir_pos_pids = NULL;
    _dir_vel_pids = NULL;
    _trq_pids     = NULL;
    _cur_pids     = NULL;
    _vel_pids     = NULL;
    res           = NULL;
    _njoints      = 0;
    _axisMap      = NULL;
    _encodersStamp = NULL;
    _foc_based_info = NULL;
    _cacheImpedance   = NULL;
    _impedance_limits = NULL;
    _ref_accs         = NULL;
    _ref_command_speeds   = NULL;
    _ref_command_positions= NULL;
    _ref_positions    = NULL;
    _ref_speeds       = NULL;
    _measureConverter = NULL;

    checking_motiondone = NULL;
    // debug connection
    //tot_packet_recv   = 0;
    //errors            = 0;
    //start             = 0;
    //end               = 0;

    // Check status of joints
    _enabledPid       = NULL;
    _enabledAmp       = NULL;
    _calibrated       = NULL;
    _last_position_move_time = NULL;

    behFlags.useRawEncoderData = false;
    behFlags.pwmIsLimited      = false;
    
    _maintenanceModeCfg.enableSkipRecalibration = false; 

    std::string tmp = yarp::conf::environment::get_string("ETH_VERBOSEWHENOK");
    if (tmp != "")
    {
        behFlags.verbosewhenok = (bool)yarp::conf::numeric::from_string(tmp, 0U);
    }
    else
    {
        behFlags.verbosewhenok = false;
    }
    parser = NULL;
    _mcparser = NULL;
    
#ifdef NETWORK_PERFORMANCE_BENCHMARK 
       /* We would like to verify if the round trimp of request and answer from embedded board is about 3 milliseconds, with a tollerance 0f 0.250 milliseconds.
       The m_responseTimingVerifier object, after 3 seconds, prints an istogram with values from 1 to 10 millisec with a step of 0.5 millisec
    */
     m_responseTimingVerifier.init(0.003, 0.00025, 0.001, 0.01, 0.0005, 30);
#endif

}


embObjMotionControl::~embObjMotionControl()
{
    yTrace() << "embObjMotionControl::~embObjMotionControl()";

    if(NULL != parser)
    {
        delete parser;
        parser = NULL;
    }

    if(NULL != _mcparser)
    {
        delete _mcparser;
        _mcparser = NULL;
    }

    dealloc();
}


bool embObjMotionControl::initialised()
{
    return opened;
}

bool embObjMotionControl::initializeInterfaces(measureConvFactors &f)
{

    ImplementControlCalibration::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementAmplifierControl::initialize(_njoints, _axisMap, f.angleToEncoder, NULL,f.ampsToSensor);
    ImplementEncodersTimed::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementMotorEncoders::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementPositionControl::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementPidControl::initialize(_njoints, _axisMap, f.angleToEncoder, NULL, f.newtonsToSensor, f.ampsToSensor, f.dutycycleToPWM);
    ImplementControlMode::initialize(_njoints, _axisMap);
    ImplementVelocityControl::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementVelocityDirect::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementControlLimits::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementImpedanceControl::initialize(_njoints, _axisMap, f.angleToEncoder, NULL, f.newtonsToSensor);
    ImplementTorqueControl::initialize(_njoints, _axisMap, f.angleToEncoder, NULL, f.newtonsToSensor, f.ampsToSensor, f.dutycycleToPWM, f.bemf2raw, f.ktau2raw);
    ImplementPositionDirect::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementInteractionMode::initialize(_njoints, _axisMap, f.angleToEncoder, NULL);
    ImplementMotor::initialize(_njoints, _axisMap);
    ImplementRemoteVariables::initialize(_njoints, _axisMap);
    ImplementAxisInfo::initialize(_njoints, _axisMap);
    ImplementCurrentControl::initialize(_njoints, _axisMap, f.ampsToSensor);
    ImplementPWMControl::initialize(_njoints, _axisMap, f.dutycycleToPWM);
    ImplementJointFault::initialize(_njoints, _axisMap);

    return true;

}

bool embObjMotionControl::open(yarp::os::Searchable &config)
{
    // - first thing to do is verify if the eth manager is available. then i parse info about the eth board.

    ethManager = eth::TheEthManager::instance();
    if(NULL == ethManager)
    {
        yFatal() << "embObjMotionControl::open() fails to instantiate ethManager";
        return false;
    }

    eOipv4addr_t ipv4addr;
    string boardIPstring;
    string boardName;
    if(false == ethManager->verifyEthBoardInfo(config, ipv4addr, boardIPstring, boardName))
    {
        yError() << "embObjMotionControl::open(): object TheEthManager fails in parsing ETH propertiex from xml file";
        return false;
    }
    // add specific info about this device ...
    
    if(false == eth::parser::read(config, bdata))
    {
        yError() << getBoardInfo() << "embObjMotionControl::open(): eth::parser fails to read board configuration data from xml file";
        return false;
    }

    if(NULL == parser)
    {
        parser = new ServiceParser;
    }


    // - now all other things

    // -- instantiate EthResource etc.

    res = ethManager->requestResource2(this, config);
    if(NULL == res)
    {
        yError() << "embObjMotionControl::open() fails because could not instantiate the ethResource for " << getBoardInfo() << " ... unable to continue";
        return false;
    }
    // READ CONFIGURATION
    if(!fromConfig(config))
    {
        yError() << getBoardInfo() << "Missing motion control parameters in config file";
        return false;
    }

    if(!res->verifyEPprotocol(eoprot_endpoint_motioncontrol))
    {
        yError() << "embObjMotionControl: failed verifyEPprotocol. Cannot continue!";
        cleanup();
        return false;
    }


    const eOmn_serv_parameter_t* servparam = &serviceConfig.ethservice;
    if(eomn_serv_MC_generic == serviceConfig.ethservice.configuration.type)
    {
        servparam = NULL;
    }

    // in here ...we open ports where to print AMO data
    mcdiagnostics.config.mode = serviceConfig.ethservice.configuration.diagnosticsmode;
    mcdiagnostics.config.par16 = serviceConfig.ethservice.configuration.diagnosticsparam;
    if(eomn_serv_diagn_mode_MC_AMOyarp == mcdiagnostics.config.mode)
    {
        // prepare the ports
        mcdiagnostics.ports.resize(2);
        for(size_t i=0; i<mcdiagnostics.ports.size(); i++)
        {
            mcdiagnostics.ports[i] = new BufferedPort<Bottle>;
            mcdiagnostics.ports[i]->open("/amo/" + res->getProperties().boardnameString + "/j" + std::to_string(i));
        }
    }

    // Initialize the downsampler timer
    
    event_downsampler = new mced::mcEventDownsampler();
    event_downsampler->config.period = 0.01;
    event_downsampler->config.threshold = 5;
    event_downsampler->config.subcomponent = "[mc.skipped-cmd-wrong-mode]";
    event_downsampler->config.info = getBoardInfo();
    event_downsampler->start();

    if(false == res->serviceVerifyActivate(eomn_serv_category_mc, servparam))
    {
        yError() << "embObjMotionControl::open() has an error in call of ethResources::serviceVerifyActivate() for" << getBoardInfo();
        cleanup();
        return false;
    }

    yDebug() << "embObjMotionControl:serviceVerifyActivate OK!";


    if(!init() )
    {
        yError() << "embObjMotionControl::open() has an error in call of embObjMotionControl::init() for" << getBoardInfo();
        return false;
    }
    else
    {
        if(behFlags.verbosewhenok)
        {
            yDebug() << "embObjMotionControl::init() has succesfully initted" << getBoardInfo();
        }
    }


    if(false == res->serviceStart(eomn_serv_category_mc))
    {
        yError() << "embObjMotionControl::open() fails to start mc service for" << getBoardInfo() << ": cannot continue";
        cleanup();
        return false;
    }
    else
    {
        if(behFlags.verbosewhenok)
        {
            yDebug() << "embObjMotionControl::open() correctly starts mc service of" << getBoardInfo();
        }
    }


    opened = true;


    if(eomn_serv_diagn_mode_MC_AMOyarp == mcdiagnostics.config.mode)
    {
        // marco.accame on 10 june 2011.
        // in here we wait some time so that yarprobotinterface may receive amo streaming before any
        // further module (e.g., the calibrator) sends an active control mode and starts moving the motor.
        //
        // how does it work?
        //
        // when the ETH board receives command from res->serviceStart() it enters the control loop and it
        // waits for a non IDLE control mode etc. but it also starts streaming AMO data. It is the thread
        // inside EthReceiver whuch process UDP pckets, so a wait inside this current thread will do the job.
        //
        // moreover, if mcdiagnostics.config.par16 is > 0, then we also have a mechanism such that the AMO
        // are streamed in a conditio of the motor not initiaized yet. in such  way we can observe what is
        // the effect of the configuration of the motor on the AMO reading.
        //
        // how does it work?
        //
        // it is the function embObjMotionControl::init() which configures the motors by sending messages
        // with tag eoprot_tag_mc_joint_config.
        // the ETH board, if it sees eomn_serv_diagn_mode_MC_AMOyarp and par16 > 0, it applies the config of
        // the motor with a delay of mcdiagnostics.config.par16 milliseconds. so, in here if we wait
        // the same amount of time, we:
        // - can read amo values with the motors off,
        // - we are sure that no other module (e.g., the calibrator) will attempt to move a motor which is
        //   not yet configured.
        // I KNOW: IT WOULD BE MUCH BETTER to wait in here until the ETH board tells us that it has configured
        // the motor (rather then just applying a delay and hoping the ETH boards works fine). but that would
        // require some effort which for now i prefer to postpone because "l'ottimo e' il nemico del bene"
        // and we need a quick solution to test icub3.

        SystemClock::delaySystem(0.001*mcdiagnostics.config.par16);
    }

    return true;
}


int embObjMotionControl::fromConfig_NumOfJoints(yarp::os::Searchable &config)
{
    //
    //  Read Configuration params from file
    //
    int jn = config.findGroup("GENERAL").check("Joints", Value(1), "Number of degrees of freedom").asInt32();

    return(jn);
}



void embObjMotionControl::debugUtil_printJointsetInfo(void)
{

    yError() << "****** DEBUG PRINTS **********";
    yError() << "joint to set:";
    for(int x=0; x< _njoints; x++)
        yError() << " /t j " << x << ": set " <<_joint2set[x];
    yError() << "jointmap:";

    yError() << " number of sets" << _jsets.size();
    for(size_t x=0; x< _jsets.size(); x++)
    {
        yError() << "set " << x<< "has size " <<_jsets[x].getNumberofJoints();
        for(int y=0; y<_jsets[x].getNumberofJoints(); y++)
            yError() << "set " << x << ": " << _jsets[x].joints[y];
    }
    yError() << "********* END ****************";

}




bool embObjMotionControl::verifyUserControlLawConsistencyInJointSet(eomc::PidInfo *pidInfo)
{

    for(size_t s=0; s<_jsets.size(); s++)
    {
       int numofjoints = _jsets[s].getNumberofJoints();

       if(numofjoints== 0 )
       {
            yError() << "embObjMC" << getBoardInfo() <<  "Jointsset " << s << "hasn't joints!!! I should be never stay here!!!";
            return false;
       }
        int firstjoint = _jsets[s].joints[0];//get firts joint of set s

        for(int k=1; k<numofjoints; k++)
        {
            int otherjoint = _jsets[s].joints[k];

            if(pidInfo[firstjoint].usernamePidSelected != pidInfo[otherjoint].usernamePidSelected)
            {
                yError() << "embObjMC "<< getBoardInfo() <<  "Joints beloning to same set must be have same control law. Joint " << otherjoint << " differs from " << firstjoint << "Set num " << s ;
                yError() << pidInfo[firstjoint].usernamePidSelected << "***" << pidInfo[otherjoint].usernamePidSelected;
                return false;
            }
        }
    }
    return true;
}





bool embObjMotionControl::verifyUserControlLawConsistencyInJointSet(eomc::TrqPidInfo *pidInfo)
{
    for(size_t s=0; s<_jsets.size(); s++)
    {
       int numofjoints = _jsets[s].getNumberofJoints();

       if(numofjoints== 0 )
       {
           yError() << "embObjMC "<< getBoardInfo() << "Jointsset " << s << "hasn't joints!!! I should be never stay here!!!";
            return false;
       }
        int firstjoint = _jsets[s].joints[0];//get firts joint of set s

        for(int k=1; k<numofjoints; k++)
        {
            int otherjoint = _jsets[s].joints[k];

            if(pidInfo[firstjoint].usernamePidSelected != pidInfo[otherjoint].usernamePidSelected)
            {
                yError() << "embObjMC"<< getBoardInfo() << "Joints beloning to same set must be have same control law. Joint " << otherjoint << " differs from " << firstjoint << "Set num " << s ;
                yError() << pidInfo[firstjoint].usernamePidSelected << "***" << pidInfo[otherjoint].usernamePidSelected;
                return false;
            }
        }
    }
    return true;
}


bool embObjMotionControl::updatedJointsetsCfgWithControlInfo()
{

    for(size_t s=0; s<_jsets.size(); s++)
    {
        if(_jsets[s].getNumberofJoints() == 0)
        {
            yError() << "embObjMC"<< getBoardInfo() << "Jointsset " << s << "hasn't joints!!! Error in configuration!!!";
            return false;
        }

        int joint = _jsets[s].joints[0];
        //eOmc_pidoutputtype_t pid_out_type = pidOutputTypeConver_eomc2fw(_trj_pids[joint].controlLaw);
        //if(eomc_pidoutputtype_unknown == pid_out_type)
        //{
        //    yError() << "embObjMC"<< getBoardInfo() << "pid output type is unknown for joint " << joint;
        //    return false;
        //}
        //_jsets[s].setPidOutputType(pid_out_type);
        //_jsets[s].setCanDoTorqueControl(isTorqueControlEnabled(joint));
        
        _jsets[s].cfg.pid_output_types.postrj_ctrl_out_type = _trj_pids[joint].out_type;
        _jsets[s].cfg.pid_output_types.veltrj_ctrl_out_type = _trj_pids[joint].out_type;
        _jsets[s].cfg.pid_output_types.mixtrj_ctrl_out_type = _trj_pids[joint].out_type;
        _jsets[s].cfg.pid_output_types.posdir_ctrl_out_type = _dir_pos_pids[joint].out_type;
        _jsets[s].cfg.pid_output_types.veldir_ctrl_out_type = _dir_vel_pids[joint].out_type;
        _jsets[s].cfg.pid_output_types.torque_ctrl_out_type = _trq_pids[joint].out_type;
        _jsets[s].cfg.pid_output_types.pwm_ctrl_out_type = eomc_ctrl_out_type_pwm;

        if (_cur_pids[joint].enabled)
        {
            _jsets[s].cfg.pid_output_types.cur_ctrl_out_type = eomc_ctrl_out_type_cur;
        }
        else
        {
            _jsets[s].cfg.pid_output_types.cur_ctrl_out_type = eomc_ctrl_out_type_n_a;
        }
    }
    return true;
}




bool embObjMotionControl::saveCouplingsData(void)
{
    eOmc_4jomo_coupling_t *jc_dest;

    static eOmc_4jomo_coupling_t dummyjomocoupling = {};

    switch(serviceConfig.ethservice.configuration.type)
    {
        case eomn_serv_MC_foc:
        {
            jc_dest = &(serviceConfig.ethservice.configuration.data.mc.foc_based.jomocoupling);
        } break;

        case eomn_serv_MC_mc4plus:
        {
            jc_dest = &(serviceConfig.ethservice.configuration.data.mc.mc4plus_based.jomocoupling);
        } break;

        case eomn_serv_MC_mc4plusmais:
        {
           jc_dest = &(serviceConfig.ethservice.configuration.data.mc.mc4plusmais_based.jomocoupling);

        } break;

        case eomn_serv_MC_mc2pluspsc:
        {
            jc_dest = &(serviceConfig.ethservice.configuration.data.mc.mc2pluspsc.jomocoupling);

        } break;

        case eomn_serv_MC_mc4plusfaps:
        {
            jc_dest = &(serviceConfig.ethservice.configuration.data.mc.mc4plusfaps.jomocoupling);

        } break;

        case eomn_serv_MC_advfoc:
        {
            jc_dest = &dummyjomocoupling;
        } break;

        case eomn_serv_MC_mc4:
        {
            return true;
        } break;

        case eomn_serv_MC_generic:
        {
            return true;
        } break;

        default:
        {
            return false;
        }
    }


    memset(jc_dest, 0, sizeof(eOmc_4jomo_coupling_t));

    //I need to initialize all elements of joint2set with "eomc_jointSetNum_none": it is used by fw to get num of setBemfParamRaw
    //4 is teh satic dimension of joint2set. see definition of type eOmc_4jomo_coupling_t
    for(int i=0; i<4; i++)
    {
        jc_dest->joint2set[i] = eomc_jointSetNum_none;
    }

    if(_joint2set.size() > 4 )
    {
        yError() << "embObjMC "<< getBoardInfo() << "Jointsset size is bigger than 4. I can't send jointset information to fw.";
        return false;
    }

    for(size_t i=0; i<_joint2set.size(); i++)
    {
        jc_dest->joint2set[i] = _joint2set[i];
    }

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            jc_dest->joint2motor[i][j] = eo_common_float_to_Q17_14((float)_couplingInfo.matrixJ2M[4*i+j]);
            jc_dest->motor2joint[i][j] = eo_common_float_to_Q17_14((float)_couplingInfo.matrixM2J[4*i+j]);
        }
    }


    for(int r=0; r<4; r++)
    {
        for(int c=0; c<6; c++)
        {
            jc_dest->encoder2joint[r][c] = eo_common_float_to_Q17_14((float)_couplingInfo.matrixE2J[6*r+c]);
        }
    }

    for(size_t s=0; s< _jsets.size(); s++)
    {
        eOmc_jointset_configuration_t* cfg_ptr = _jsets[s].getConfiguration();
        memcpy(&(jc_dest->jsetcfg[s]), cfg_ptr, sizeof(eOmc_jointset_configuration_t));
    }


    if(eomn_serv_MC_advfoc == serviceConfig.ethservice.configuration.type)
    {
        // i will copy data from jc_dest to the effective destination
        eOmc_adv4jomo_coupling_t *ajc = &serviceConfig.ethservice.configuration.data.mc.advfoc.adv4jomocoupling;
        ajc->type = eommccoupling_traditional4x4;
        // i copy some fields as they are
        std::memmove(&ajc->data.coupling4x4.joint2set[0], &jc_dest->joint2set[0], 4*sizeof(uint8_t));
        std::memmove(&ajc->data.coupling4x4.jsetcfg[0], &jc_dest->jsetcfg[0], 4*sizeof(eOmc_jointset_configuration_t));
        std::memmove(&ajc->data.coupling4x4.joint2motor, &jc_dest->joint2motor, sizeof(eOmc_4x4_matrix_t));
        std::memmove(&ajc->data.coupling4x4.motor2joint, &jc_dest->motor2joint, sizeof(eOmc_4x4_matrix_t));
        // and i will copy only 4x4 from one field
        for(uint8_t r=0; r<4; r++)
        {
            for(uint8_t c=0; c<4; c++)
            {
                ajc->data.coupling4x4.encoder2joint4x4[r][c] = jc_dest->encoder2joint[r][c];
            }
        }
    }

    return true;

}


bool embObjMotionControl::fromConfig_Step2(yarp::os::Searchable &config)
{
    Bottle xtmp;
    int i,j;

    measureConvFactors measConvFactors (_njoints);

    if(iNeedCouplingsInfo())
    {

        ////// COUPLINGS
        if(!_mcparser->parseCouplingInfo(config, _couplingInfo))
            return false;


        ////// JOINTSET_CFG
        if(!_mcparser->parseJointsetCfgGroup(config, _jsets, _joint2set))
            return false;

        //debugUtil_printJointsetInfo();
    }


    ///////// GENERAL MECHANICAL INFO


    {
        if(!_mcparser->parseAxisInfo(config, _axisMap, _axesInfo))
            return false;

         ////// measures conversion factors
        if(behFlags.useRawEncoderData)
        {
            for (i = 0; i < _njoints; i++)
            {
                measConvFactors.angleToEncoder[i] = 1;
            }
        }
        else
        {
            if(!_mcparser->parseEncoderFactor(config, measConvFactors.angleToEncoder))
                return false;
        }

        if (!_mcparser->parsefullscalePWM(config, measConvFactors.dutycycleToPWM))
            return false;

        if (!_mcparser->parseAmpsToSensor(config, measConvFactors.ampsToSensor))
            return false;
        
        //VALE: i have to parse GeneralMecGroup after parsing jointsetcfg, because inside generalmec group there is useMotorSpeedFbk that needs jointset info.

        if(!_mcparser->parseGearboxValues(config, _gearbox_M2J, _gearbox_E2J))
            return false;

        // useMotorSpeedFbk
        if(eomn_serv_MC_mc4 != (eOmn_serv_type_t)serviceConfig.ethservice.configuration.type)
        {
            int* useMotorSpeedFbk = 0;
            useMotorSpeedFbk = new int[_njoints];
            if (!_mcparser->parseMechanicalsFlags(config, useMotorSpeedFbk))
            {
                delete[] useMotorSpeedFbk;
                return false;
            }
            //Note: currently in eth protocol this parameter belongs to jointset configuration. So
            // i need to check that every joint belong to same set has the same value
            if (!verifyUseMotorSpeedFbkInJointSet(useMotorSpeedFbk))
            {
                delete[] useMotorSpeedFbk;
                return false;
            }
            delete[] useMotorSpeedFbk;
        }
        bool deadzoneIsAvailable;
        if(!_mcparser->parseDeadzoneValue(config, _deadzone, &deadzoneIsAvailable))
            return false;
        if(!deadzoneIsAvailable) // if parameter is not written in configuration files then use default values
        {
            updateDeadZoneWithDefaultValues();
        }

        if(!_mcparser->parseKalmanFilterParams(config, _kalman_params))
        {
            return false;
        }
    }


    ///// CONTROLS AND PID GROUPS
    {
        bool lowLevPidisMandatory = false;

        if((serviceConfig.ethservice.configuration.type == eomn_serv_MC_foc) || (serviceConfig.ethservice.configuration.type == eomn_serv_MC_advfoc))
        {
            lowLevPidisMandatory = true;
        }

        if(!_mcparser->parsePids(config, _trj_pids, _vel_pids, _dir_pos_pids, _dir_vel_pids, _trq_pids, _cur_pids, lowLevPidisMandatory))
            return false;

        // 1) verify joint belonging to same set has same control law
        //if(!verifyUserControlLawConsistencyInJointSet(_ppids))
        //    return false;
        //if(!verifyUserControlLawConsistencyInJointSet(_vpids))
        //    return false;
        //if(!verifyUserControlLawConsistencyInJointSet(_tpids))
        //    return false;

        //yarp::dev::PidFeedbackUnitsEnum fbk_TrqPidUnits;
        //yarp::dev::PidOutputUnitsEnum   out_TrqPidUnits;
        //if(!verifyTorquePidshasSameUnitTypes(fbk_TrqPidUnits, out_TrqPidUnits))
        //    return false;

        //2) since some joint sets configuration info is in control and ids group, get that info and save them in jointset data struct.
        updatedJointsetsCfgWithControlInfo();
    }

    for (i = 0; i < _njoints; i++)
    {
        measConvFactors.newtonsToSensor[i] = 1000000.0f; // conversion from Nm into microNm

        measConvFactors.bemf2raw[i] = measConvFactors.newtonsToSensor[i] / measConvFactors.angleToEncoder[i];
        if (_trq_pids->out_PidUnits == yarp::dev::PidOutputUnitsEnum::DUTYCYCLE_PWM_PERCENT)
        {
            measConvFactors.ktau2raw[i] = measConvFactors.dutycycleToPWM[i] / measConvFactors.newtonsToSensor[i];
        }
        else if (_trq_pids->out_PidUnits == yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS)
        {
            measConvFactors.ktau2raw[i] = 1.0 / measConvFactors.newtonsToSensor[i];
        }
        else
        {
            yError() << "Invalid ktau units"; return false;
        }
    }

    ///////////////INIT INTERFACES
    _measureConverter = new ControlBoardHelper(_njoints, _axisMap, measConvFactors.angleToEncoder, NULL, measConvFactors.newtonsToSensor, measConvFactors.ampsToSensor, nullptr, measConvFactors.dutycycleToPWM , measConvFactors.bemf2raw, measConvFactors.ktau2raw);
    _measureConverter->set_pid_conversion_units(PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, _trj_pids->fbk_PidUnits, _trj_pids->out_PidUnits);
    _measureConverter->set_pid_conversion_units(PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT, _dir_vel_pids->fbk_PidUnits, _dir_vel_pids->out_PidUnits);
    _measureConverter->set_pid_conversion_units(PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE,   _trq_pids->fbk_PidUnits, _trq_pids->out_PidUnits);
    _measureConverter->set_pid_conversion_units(PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT,  _cur_pids->fbk_PidUnits, _cur_pids->out_PidUnits);
    _measureConverter->set_pid_conversion_units(PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY, _vel_pids->fbk_PidUnits, _vel_pids->out_PidUnits);
    _measureConverter->set_pid_conversion_units(PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT, _dir_pos_pids->fbk_PidUnits, _dir_pos_pids->out_PidUnits);
    /*
    void ControlBoardHelper::set_pid_conversion_units(const PidControlTypeEnum& pidtype, const PidFeedbackUnitsEnum fbk_conv_units, const PidOutputUnitsEnum out_conv_units)
    {
        ControlBoardHelper* cb_helper = this;
        int nj = cb_helper->axes();
        for (int i = 0; i < nj; i++)
        {
            mPriv->pid_units[pidtype][i].fbk_units = fbk_conv_units;
            mPriv->pid_units[pidtype][i].out_units = out_conv_units;
        }
    }
    */
    initializeInterfaces(measConvFactors);
    ImplementPidControl::setConversionUnits(PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, _trj_pids->fbk_PidUnits, _trj_pids->out_PidUnits);
    ImplementPidControl::setConversionUnits(PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT,   _dir_pos_pids->fbk_PidUnits, _dir_pos_pids->out_PidUnits);
    ImplementPidControl::setConversionUnits(PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE,   _trq_pids->fbk_PidUnits, _trq_pids->out_PidUnits);
    ImplementPidControl::setConversionUnits(PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT,  _cur_pids->fbk_PidUnits, _cur_pids->out_PidUnits);
    ImplementPidControl::setConversionUnits(PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY, _vel_pids->fbk_PidUnits, _vel_pids->out_PidUnits);
    ImplementPidControl::setConversionUnits(PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT,   _dir_vel_pids->fbk_PidUnits, _dir_vel_pids->out_PidUnits);


    //Now save in data in structures EmbObj protocol compatible
    if(!saveCouplingsData())
        return false;


    ////// IMPEDANCE PARAMETERS
    if(! _mcparser->parseImpedanceGroup(config,_impedance_params))
    {
        yError() << "embObjMC " << getBoardInfo() << "IMPEDANCE section: error detected in parameters syntax";
        return false;
    }

    ////// LUGRE PARAMETERS
    if(! _mcparser->parseLugreGroup(config,_lugre_params))
    {
        yError() << "embObjMC " << getBoardInfo() << "LUGRE section: error detected in parameters syntax";
        
    }

    ////// IMPEDANCE LIMITS DEFAULT VALUES (UNDER TESTING)
    for(j=0; j<_njoints; j++)
    {
        // got from canBusMotionControl, ask to Randazzo Marco
        _impedance_limits[j].min_damp=  0.001;
        _impedance_limits[j].max_damp=  9.888;
        _impedance_limits[j].min_stiff= 0.002;
        _impedance_limits[j].max_stiff= 9.889;
        _impedance_limits[j].param_a=   0.011;
        _impedance_limits[j].param_b=   0.012;
        _impedance_limits[j].param_c=   0.013;
    }



    /////// LIMITS
    {
        if(!_mcparser->parseCurrentLimits(config, _currentLimits))
            return false;

        if(!_mcparser->parseTemperatureLimits(config, _temperatureLimits))
            return false;

        if(!_mcparser->parseJointsLimits(config, _jointsLimits))
            return false;

        if(!_mcparser->parseRotorsLimits(config, _rotorsLimits))
            return false;
    }

    /////// [2FOC] or [AMCBLDC] or [ADVFOC_COMMON]
    eOmn_serv_type_t servtype = static_cast<eOmn_serv_type_t>(serviceConfig.ethservice.configuration.type);

    if((eomn_serv_MC_foc == servtype) || (eomn_serv_MC_advfoc == servtype))
    {
        std::string groupName = {};

        if(eomn_serv_MC_foc == servtype)
        {
            // in here the name of the group depends on the configured board
            eObrd_type_t brd = static_cast<eObrd_type_t>(serviceConfig.ethservice.configuration.data.mc.foc_based.type);
            groupName = (eobrd_foc == brd) ? "2FOC" : "AMCBLDC";
        }
        else if(eomn_serv_MC_advfoc == servtype)
        {
            // but in here we may have multiple boards, so ... it is better to use a generic name
            // ADVFOC with multiple columns, one for each motor
            groupName = "ADVFOC";
        }

        if(!_mcparser->parseFocGroup(config, _foc_based_info, groupName, _temperatureSensorsVector))
            return false;

        for (j = 0; j < _njoints; j++)
        {
            if (((_temperatureSensorsVector.at(j)->getType() != motor_temperature_sensor_none )) && ((_temperatureLimits[j].hardwareTemperatureLimit == 0) || (_temperatureLimits[j].warningTemperatureLimit == 0)))
            {
                yError() << "In" << getBoardInfo() << "joint" << j << ": inconsistent configuration, please update it. If Temperature limits are not set then TemperatureSensorType must be NONE or not set and/or HasTempSensor must be zero. Aborting...";
                return false;
            }

            if (_temperatureSensorsVector.at(j)->getType() == motor_temperature_sensor_none)
            {
                yInfo() << "embObjMC " << getBoardInfo() << "joint " << j << " has motor not provided with any available type of temperature sensor. If needed update the configurations file accordingly";
            }
        }
    }
    else
    {
        for (j = 0; j < _njoints; j++)
        {
            _temperatureSensorsVector.at(j) = std::make_unique<eomc::TemperatureSensorNONE>();
        }
    }

    int defaultTimeout = 100;

    if (this->serviceConfig.ethservice.configuration.type == eomn_serv_MC_advfoc)
    {
        // temporary workaround
        // in this case the default timeout is 300 ms because there is some lag in AMC
        defaultTimeout = 300;
    }

    /////// [TIMEOUTS]
    if(! _mcparser->parseTimeoutsGroup(config, _timeouts, defaultTimeout))
        return false;


    return true;
}



bool embObjMotionControl::verifyUseMotorSpeedFbkInJointSet(int useMotorSpeedFbk [])
{
    for(size_t s=0; s< _jsets.size(); s++)
    {
        int numofjointsinset = _jsets[s].getNumberofJoints();
        if(numofjointsinset == 0 )
        {
            yError() << "embObjMC " << getBoardInfo() << "Jointsset " << s << "hasn't joints!!! Error in configuration!!!";
            return false;
        }

        int firstjointofset = _jsets[s].joints[0];
        for(int j=1; j<numofjointsinset; j++)
        {
            int joint = _jsets[s].joints[j];
            if(useMotorSpeedFbk[firstjointofset] != useMotorSpeedFbk[joint])
            {
                yError() << "embObjMC " << getBoardInfo() << ". Param useMotorSpeedFbk should have same value for joints belong same set. See joint " << firstjointofset << " and " << joint;
                return false;
            }
        }

        _jsets[s].setUseSpeedFeedbackFromMotors(useMotorSpeedFbk[firstjointofset]);
    }

    return true;

}

bool embObjMotionControl::verifyTorquePidshasSameUnitTypes(yarp::dev::PidFeedbackUnitsEnum  &fbk_pidunits, yarp::dev::PidOutputUnitsEnum& out_pidunits)
{
    fbk_pidunits = yarp::dev::PidFeedbackUnitsEnum::RAW_MACHINE_UNITS;
    out_pidunits = yarp::dev::PidOutputUnitsEnum::RAW_MACHINE_UNITS;
    //get first joint with enabled torque
    int firstjoint = -1;
    for(int i=0; i<_njoints; i++)
    {
        if(_trq_pids[i].enabled)
            firstjoint = i;
    }

    if(firstjoint==-1)
    {
        // no joint has torque enabed
        return true;
    }

    for(int i=firstjoint+1; i<_njoints; i++)
    {
        if(_trq_pids[i].enabled)
        {
            if(_trq_pids[firstjoint].fbk_PidUnits != _trq_pids[i].fbk_PidUnits ||
               _trq_pids[firstjoint].out_PidUnits != _trq_pids[i].out_PidUnits)
            {
                yError() << "embObjMC " << getBoardInfo() << "all joints with torque enabled should have same controlunits type. Joint " << firstjoint << " differs from joint " << i;
                return false;
            }
        }
    }

    fbk_pidunits = _trq_pids[firstjoint].fbk_PidUnits;
    out_pidunits = _trq_pids[firstjoint].out_PidUnits;
    return true;
}

// bool embObjMotionControl::isTorqueControlEnabled(int joint)
// {
//     return (_trq_pids[joint].enabled);
// }

void embObjMotionControl::updateDeadZoneWithDefaultValues(void)
{
    for(int i=0; i<_njoints; i++)
    {
        switch(_jointEncs[i].type)
        {
            case eomc_enc_aea:
                _deadzone[i] = eomc_defaultValue::DeadZone::jointWithAEA;// 0.0494;
                break;
            case eomc_enc_aea3:
                _deadzone[i] = eomc_defaultValue::DeadZone::jointWithAEA3;// TODO: temporary equal to 0.0
                break;
            case eomc_enc_aksim2:
                _deadzone[i] = eomc_defaultValue::DeadZone::jointWithAKSIM2;
            case eomc_enc_amo:
                _deadzone[i] = eomc_defaultValue::DeadZone::jointWithAMO;//  0.0055;
                break;
            case eomc_enc_roie:
            case eomc_enc_absanalog:
            case eomc_enc_mais:
            case eomc_enc_qenc:
            case eomc_enc_hallmotor:
            case eomc_enc_spichainof2:
            case eomc_enc_spichainof3:
            case eomc_enc_mrie:
            default:
                _deadzone[i] = 0.0;
            
        }
    }
}

// use this one for ... service configuration
bool embObjMotionControl::fromConfig_readServiceCfg(yarp::os::Searchable &config)
{

    if(false == parser->parseService(config, serviceConfig))
    {
        yError() << "embObjMC " << getBoardInfo() << "cannot parse service" ;
        return false;
    }

    if(eomn_serv_MC_generic == serviceConfig.ethservice.configuration.type)
    {
        yError() << "embObjMC " << getBoardInfo() << "it is no longer possible use eomn_serv_MC_generic, because firmware cannot configure itself!" ;
        return false;
    }

    //now parser read encoders' resolutions also.
    //so here I save in embObMotioncontrol memory encoders's resolution
    servMC_encoder_t * jointEncoder_ptr = NULL;
    servMC_encoder_t * motorEncoder_ptr = NULL;
    for(int i=0; i<_njoints; i++)
    {
        jointEncoder_ptr = parser->getEncoderAtJoint(i);
        motorEncoder_ptr = parser->getEncoderAtMotor(i);

        if(NULL == jointEncoder_ptr)
        {
            _jointEncs[i].resolution = 1;
            _jointEncs[i].type = eomc_enc_none;
            _jointEncs[i].tolerance  = 0;
        }
        else
        {
            _jointEncs[i].resolution  = jointEncoder_ptr->resolution;
            _jointEncs[i].type = (eOmc_encoder_t)jointEncoder_ptr->desc.type; //Here I'm sure that type belong to eOmc_encoder_t enum.It is filled by eomc_string2encoder function
            _jointEncs[i].tolerance  = jointEncoder_ptr->tolerance;
        }


        if(NULL == motorEncoder_ptr)
        {
            _motorEncs[i].resolution = 1;
            _motorEncs[i].type = eomc_enc_none;
            _motorEncs[i].tolerance = 0;
        }
        else
        {
            _motorEncs[i].resolution = motorEncoder_ptr->resolution;
            _motorEncs[i].type = (eOmc_encoder_t)motorEncoder_ptr->desc.type; //Here I'm sure that type belong to eOmc_encoder_t enum.It is filled by eomc_string2encoder function
            _motorEncs[i].tolerance = motorEncoder_ptr->tolerance;
        }


    }


    return true;
}



bool embObjMotionControl::fromConfig(yarp::os::Searchable &config)
{

    _njoints = fromConfig_NumOfJoints(config);

    if(0 == _njoints)
    {
        yError() << "embObjMC"<< getBoardInfo() << "fromConfig(): detected _njoints = " << _njoints;
        return false;
    }

    // we have number of joints inside _njoints. we allocate all required buffers
    if(!alloc(_njoints))
    {
        yError() << "embObjMC"<< getBoardInfo() << "fromConfig(): alloc() failed for _njoints = " << _njoints;
        return false;
    }


    _mcparser = new eomc::Parser(_njoints, string(res->getProperties().boardnameString));

    ////// check motion control xml files version
    int currentMCversion =0;
    if(!_mcparser->parseMotioncontrolVersion(config, currentMCversion))
        return false;

    if (currentMCversion != PARSER_MOTION_CONTROL_VERSION)
    {
        yError() << "embObjMC" << getBoardInfo() << "------ ATTENTION!!!! Wrong value of <MotioncontrolVersion> parameter !!!! ---------------------------------------------------------------------------------------";
        yError() << "embObjMC" << getBoardInfo() << "------ This means that the configuration files of this device are not compatible with my parser, so I cannot start. ";
        yError() << "embObjMC" << getBoardInfo() << "------ I need version " << PARSER_MOTION_CONTROL_VERSION << ", but in configuration files have version " << currentMCversion << ".";
        yError() << "embObjMC" << getBoardInfo() << "------ Please update configuration files in robots-configuration repository. (see https://icub-tech-iit.github.io/documentation/icub_robot_configuration/icub_robot_configuration_index/ for more information). ";
        yError() << "embObjMC" << getBoardInfo() << "------ If the problem persists contact icub-support@iit.it DO NOT DO IT BY YOURSELF.";
        yError() << "embObjMC" << getBoardInfo() << "----------------------------------------------------------------------------------------------------------------------------------------------------------------";
        return false;
    }

    //print verbose info
    if(_mcparser->isVerboseEnabled(config))
        yTrace() << config.toString().c_str();



    // first step of configuration
    if(false == fromConfig_readServiceCfg(config))
    {
        return false;
    }

    if(!_mcparser->parseBehaviourFalgs(config, behFlags.useRawEncoderData, behFlags.pwmIsLimited ))//in general info group
    {
        return false;
    }

    if (!_mcparser->parseMaintenanceModeGroup(config, _maintenanceModeCfg.enableSkipRecalibration))// in general maintenance group
    {
        return false;
    }
    
    // second step of configuration
    if(false == fromConfig_Step2(config))
    {
        return false;
    }

    // third step of configuration


    return true;
}


bool embObjMotionControl::init()
{
    eOprotID32_t protid = 0;

    /////////////////////////////////////////////////
    //SEND DISABLE TO ALL JOINTS
    /////////////////////////////////////////////////

    for(int logico=0; logico< _njoints; logico++)
    {
        int fisico = _axisMap[logico];
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, fisico, eoprot_tag_mc_joint_cmmnds_controlmode);
        eOenum08_t controlMode = eomc_controlmode_cmd_idle;

        if(false == res->setRemoteValue(protid, &controlMode))
        {
            yError() << "embObjMotionControl::init() had an error while setting eomc_controlmode_cmd_idle in "<< getBoardInfo();
            // return(false); i dont return false. because even if a failure, that is not a severe error.
            // MOREOVER: to verify we must read the status of the joint and NOT the command ... THINK OF IT
        }
    }

    SystemClock::delaySystem(0.010);


    ////////////////////////////////////////////////
    // configure the regular rops
    ////////////////////////////////////////////////

    vector<eOprotID32_t> id32v(0);
    for(int n=0; n<_njoints; n++)
    {
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, n, eoprot_tag_mc_joint_status_core);
        id32v.push_back(protid);
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, n, eoprot_tag_mc_joint_status_addinfo_multienc);
        id32v.push_back(protid);
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, n, eoprot_tag_mc_motor_status);
        id32v.push_back(protid);
    }

    if(eomn_serv_diagn_mode_MC_AMOyarp == mcdiagnostics.config.mode)
    {
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, 0, eoprot_tag_mc_joint_status_debug);
        id32v.push_back(protid);
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, 1, eoprot_tag_mc_joint_status_debug);
        id32v.push_back(protid);
    }


    if(false == res->serviceSetRegulars(eomn_serv_category_mc, id32v))
    {
        yError() << "embObjMotionControl::init() fails to add its variables to regulars in "<< getBoardInfo() << ": cannot proceed any further";
        return false;
    }
    else
    {
        if(behFlags.verbosewhenok)
        {
            yDebug() << "embObjMotionControl::init() added" << id32v.size() << "regular rops to "<< getBoardInfo();
            char nvinfo[128];
            for(unsigned int r=0; r<id32v.size(); r++)
            {
                uint32_t id32 = id32v.at(r);
                eoprot_ID2information(id32, nvinfo, sizeof(nvinfo));
                yDebug() << "\t it added regular rop for" << nvinfo;
            }
        }
    }

    SystemClock::delaySystem(0.005);


    //////////////////////////////////////////
    // sending configuration to the JOINTS   //
    //////////////////////////////////////////
    for(int logico=0; logico< _njoints; logico++)
    {
        int fisico = _axisMap[logico];
        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, fisico, eoprot_tag_mc_joint_config);

        eOmc_joint_config_t jconfig = {0};
        memset(&jconfig, 0, sizeof(eOmc_joint_config_t));
        yarp::dev::Pid tmp; 
        tmp = _measureConverter->convert_pid_to_machine(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION,_trj_pids[logico].pid, fisico);
        copyPid_iCub2eo(&tmp, &jconfig.pidtrajectory);
        tmp = _measureConverter->convert_pid_to_machine(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT, _dir_pos_pids[logico].pid, fisico);
        copyPid_iCub2eo(&tmp, &jconfig.piddirect);
        tmp = _measureConverter->convert_pid_to_machine(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE, _trq_pids[logico].pid, fisico);
        copyPid_iCub2eo(&tmp, &jconfig.pidtorque);

        //stiffness and damping read in xml file are in Nm/deg and Nm/(Deg/sec), so we need to convert before send to fw.
        jconfig.impedance.damping   = (eOmeas_damping_t) _measureConverter->impN2S(_impedance_params[logico].damping, fisico);
        jconfig.impedance.stiffness = (eOmeas_stiffness_t) _measureConverter->impN2S(_impedance_params[logico].stiffness, fisico);
        jconfig.impedance.offset    = 0;
        
        _cacheImpedance[logico].stiffness = jconfig.impedance.stiffness;
        _cacheImpedance[logico].damping   = jconfig.impedance.damping;
        _cacheImpedance[logico].offset    = jconfig.impedance.offset;

        jconfig.userlimits.max = (eOmeas_position_t) S_32(_measureConverter->posA2E(_jointsLimits[logico].posMax, fisico));
        jconfig.userlimits.min = (eOmeas_position_t) S_32(_measureConverter->posA2E(_jointsLimits[logico].posMin, fisico));

        jconfig.hardwarelimits.max = (eOmeas_position_t) S_32(_measureConverter->posA2E(_jointsLimits[logico].posHwMax, fisico));
        jconfig.hardwarelimits.min = (eOmeas_position_t) S_32(_measureConverter->posA2E(_jointsLimits[logico].posHwMin, fisico));


        jconfig.maxvelocityofjoint = S_32(_measureConverter->posA2E(_jointsLimits[logico].velMax, fisico)); //icubdeg/s
        jconfig.velocitysetpointtimeout = (eOmeas_time_t) U_16(_timeouts[logico].velocity_ref);
        jconfig.currentsetpointtimeout = (eOmeas_time_t) U_16(_timeouts[logico].current_ref);
        jconfig.openloopsetpointtimeout = (eOmeas_time_t) U_16(_timeouts[logico].pwm_ref);
        jconfig.torquesetpointtimeout = (eOmeas_time_t) U_16(_timeouts[logico].torque_ref);
        jconfig.torquefeedbacktimeout = (eOmeas_time_t) U_16(_timeouts[logico].torque_fbk);

        jconfig.jntEncoderResolution = _jointEncs[logico].resolution;
        jconfig.jntEncoderType = _jointEncs[logico].type;
        jconfig.jntEncTolerance = _jointEncs[logico].tolerance;

        jconfig.motor_params.bemf_value = _measureConverter->bemf_user2raw(_trq_pids[logico].kbemf, fisico);
        jconfig.motor_params.bemf_scale = 0;
        jconfig.motor_params.ktau_value = _measureConverter->ktau_user2raw(_trq_pids[logico].ktau, fisico);
        jconfig.motor_params.ktau_scale = 0;
        jconfig.motor_params.friction.viscous_pos_val = _measureConverter->viscousPos_user2raw(_trq_pids[logico].viscousPos, fisico);
        jconfig.motor_params.friction.viscous_neg_val = _measureConverter->viscousNeg_user2raw(_trq_pids[logico].viscousNeg, fisico);
        jconfig.motor_params.friction.coulomb_pos_val = _measureConverter->coulombPos_user2raw(_trq_pids[logico].coulombPos, fisico);
        jconfig.motor_params.friction.coulomb_neg_val = _measureConverter->coulombNeg_user2raw(_trq_pids[logico].coulombNeg, fisico);
        jconfig.motor_params.friction.velocityThres_val = _measureConverter->velocityThres_user2raw(_trq_pids[logico].velocityThres, fisico);

        jconfig.gearbox_E2J = _gearbox_E2J[logico];
        
        jconfig.deadzone = _measureConverter->posA2E(_deadzone[logico], fisico);

        jconfig.tcfiltertype=_trq_pids[logico].filterType;

        jconfig.kalman_params.enabled = _kalman_params[logico].enabled;
        for(int i=0; i<_kalman_params[logico].x0.size(); i++) jconfig.kalman_params.x0[i] = _kalman_params[logico].x0.at(i);
        for(int i=0; i<_kalman_params[logico].Q.size(); i++) jconfig.kalman_params.Q[i] = _kalman_params[logico].Q.at(i);
        jconfig.kalman_params.R = _kalman_params[logico].R;
        jconfig.kalman_params.P0 = _kalman_params[logico].P0;

        if(false == res->setcheckRemoteValue(protid, &jconfig, 10, 0.010, 0.050))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setcheckRemoteValue() for joint config fisico #" << fisico << "in "<< getBoardInfo();
            return false;
        }
        else
        {
            if(behFlags.verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured joint config fisico #" << fisico << "in "<< getBoardInfo();
            }
        }
    }


    //////////////////////////////////////////
    // sending configuration to the MOTORS  //
    //////////////////////////////////////////


    for(int logico=0; logico<_njoints; logico++)
    {
        int fisico = _axisMap[logico];

        protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, fisico, eoprot_tag_mc_motor_config);
        eOmc_motor_config_t motor_cfg = {0};
        motor_cfg.maxvelocityofmotor = 0;//_maxMotorVelocity[logico]; //unused yet!
        motor_cfg.currentLimits.nominalCurrent = _currentLimits[logico].nominalCurrent;
        motor_cfg.currentLimits.overloadCurrent = _currentLimits[logico].overloadCurrent;
        motor_cfg.currentLimits.peakCurrent = _currentLimits[logico].peakCurrent;
        motor_cfg.gearbox_M2J = _gearbox_M2J[logico];
        motor_cfg.rotorEncoderResolution = _motorEncs[logico].resolution;
        motor_cfg.rotEncTolerance = _motorEncs[logico].tolerance;
        motor_cfg.hasHallSensor = _foc_based_info[logico].hasHallSensor;
        motor_cfg.hasRotorEncoder = _foc_based_info[logico].hasRotorEncoder;
        motor_cfg.hasTempSensor = _foc_based_info[logico].hasTempSensor;
        motor_cfg.hasRotorEncoderIndex = _foc_based_info[logico].hasRotorEncoderIndex;
        motor_cfg.hasSpeedEncoder = _foc_based_info[logico].hasSpeedEncoder;
        motor_cfg.verbose = _foc_based_info[logico].verbose;
        motor_cfg.motorPoles = _foc_based_info[logico].motorPoles;
        motor_cfg.rotorIndexOffset = _foc_based_info[logico].rotorIndexOffset;
        motor_cfg.Kbemf = _foc_based_info[logico].kbemf;
        motor_cfg.rotorEncoderType = _motorEncs[logico].type;
        motor_cfg.pwmLimit =_rotorsLimits[logico].pwmMax;
        motor_cfg.temperatureLimit = (eOmeas_temperature_t) S_16(_temperatureSensorsVector.at(logico)->convertTempCelsiusToRaw(_temperatureLimits.at(logico).hardwareTemperatureLimit)); //passing raw value not in degree
	    motor_cfg.limitsofrotor.max = (eOmeas_position_t) S_32(_measureConverter->posA2E(_rotorsLimits[logico].posMax, fisico ));
        motor_cfg.limitsofrotor.min = (eOmeas_position_t) S_32(_measureConverter->posA2E(_rotorsLimits[logico].posMin, fisico ));

        motor_cfg.LuGre_params.Km     = _lugre_params[logico].Km;
        motor_cfg.LuGre_params.Kw     = _lugre_params[logico].Kw;
        motor_cfg.LuGre_params.S0     = _lugre_params[logico].S0;
        motor_cfg.LuGre_params.S1     = _lugre_params[logico].S1;
        motor_cfg.LuGre_params.Vth    = _lugre_params[logico].Vth;
        motor_cfg.LuGre_params.Fc_pos = _lugre_params[logico].Fc_pos;
        motor_cfg.LuGre_params.Fc_neg = _lugre_params[logico].Fc_neg;
        motor_cfg.LuGre_params.Fs_pos = _lugre_params[logico].Fs_pos;
        motor_cfg.LuGre_params.Fs_neg = _lugre_params[logico].Fs_neg;
        
        yarp::dev::Pid tmp;

        //current pid
        tmp = _measureConverter->convert_pid_to_machine(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT, _cur_pids[logico].pid, fisico);
        copyPid_iCub2eo(&tmp, &motor_cfg.pidcurrent);

        //velocity pid
        memset(&motor_cfg.pidvelcur, 0, sizeof(eOmc_PID_t));
        memset(&motor_cfg.pidvelpwm, 0, sizeof(eOmc_PID_t));

        tmp = _measureConverter->convert_pid_to_machine(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT, _dir_vel_pids[logico].pid, fisico);

        if(eomc_ctrl_out_type_vel == _dir_vel_pids[logico].out_type)
        {
            copyPid_iCub2eo(&tmp, &motor_cfg.pidvelpwm);
        }
        else if (eomc_ctrl_out_type_vel_cur == _dir_vel_pids[logico].out_type)
        {
            copyPid_iCub2eo(&tmp, &motor_cfg.pidvelcur);
        }
        else
        {

            yError() << "embObjMC " << getBoardInfo() << " joint " << logico << " velocity direct pid has unsupported output type " << _dir_vel_pids[logico].out_type;
            return false;
        }
        

        if (false == res->setcheckRemoteValue(protid, &motor_cfg, 10, 0.010, 0.050))
        {
            yError() << "FATAL: embObjMotionControl::init() had an error while calling setcheckRemoteValue() for motor config fisico #" << fisico << "in "<< getBoardInfo();
            return false;
        }
        else
        {
            if (behFlags.verbosewhenok)
            {
                yDebug() << "embObjMotionControl::init() correctly configured motor config fisico #" << fisico << "in "<< getBoardInfo();
            }
        }
    }

    /////////////////////////////////////////////
    // sending configuration to the CONTROLLER //
    /////////////////////////////////////////////

    protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_controller, 0, eoprot_tag_mc_controller_config);

    eOmc_controller_config_t controller_cfg = {0};
    memset(&controller_cfg, 0, sizeof(eOmc_controller_config_t));
    controller_cfg.durationofctrlloop = (uint32_t)bdata.settings.txconfig.cycletime;
    controller_cfg.enableskiprecalibration = _maintenanceModeCfg.enableSkipRecalibration;

    if(false == res->setcheckRemoteValue(protid, &controller_cfg, 10, 0.010, 0.050))
    {
        yError() << "FATAL: embObjMotionControl::init() had an error while calling setcheckRemoteValue() for the controller " << "in "<< getBoardInfo();
        return false;
    }
    else
    {
        if(behFlags.verbosewhenok)
        {
            yDebug() << "embObjMotionControl::init() correctly configured controller config " << "in "<< getBoardInfo();
        }
    }

    ///////////////////////////////////////////////
    // intialize the map of the rawValuesVectors //
    //////////////////////////////////////////////
    const char* tag = eoprot_TAG2string(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_status_addinfo_multienc);                              
    
    _rawValuesMetadataMap.insert({{tag, rawValuesKeyMetadata({}, {}, _njoints * eOmc_joint_multienc_maxnum)}});
    for (auto &[k, v] : _rawValuesMetadataMap)
    {
        std::string auxstring = "";
        
        for (int i = 0; i < _njoints; i++)
        {
            getEntityName(i, auxstring);
            if (k == tag)
            {
                v.axesNames.push_back(auxstring);
                v.rawValueNames.insert(v.rawValueNames.end(), 
                    {auxstring+"_primary_encoder_raw_value", 
                    auxstring+"_secondary_encoder_raw_value",
                    auxstring+"_primary_encoder_diagnostic"}
                );  
            }
            auxstring.clear();
        }  
    }
    yTrace() << "embObjMotionControl::init(): correctly instantiated for " << getBoardInfo();
    return true;
}



bool embObjMotionControl::close()
{
    yTrace() << " embObjMotionControl::close()";

    ImplementControlMode::uninitialize();
    ImplementEncodersTimed::uninitialize();
    ImplementMotorEncoders::uninitialize();
    ImplementPositionControl::uninitialize();
    ImplementVelocityControl::uninitialize();
    ImplementPidControl::uninitialize();
    ImplementControlCalibration::uninitialize();
    ImplementAmplifierControl::uninitialize();
    ImplementImpedanceControl::uninitialize();
    ImplementControlLimits::uninitialize();
    ImplementTorqueControl::uninitialize();
    ImplementPositionDirect::uninitialize();
    ImplementInteractionMode::uninitialize();
    ImplementRemoteVariables::uninitialize();
    ImplementAxisInfo::uninitialize();
    ImplementCurrentControl::uninitialize();
    ImplementPWMControl::uninitialize();
    ImplementJointFault::uninitialize();

    if (_measureConverter)  {delete _measureConverter; _measureConverter=0;}


    if(eomn_serv_diagn_mode_MC_AMOyarp == mcdiagnostics.config.mode)
    {
        // close the ports
        for(size_t i=0; i<mcdiagnostics.ports.size(); i++)
        {
            mcdiagnostics.ports[i]->close();
            delete mcdiagnostics.ports[i];
        }
        mcdiagnostics.ports.clear();

        mcdiagnostics.config.mode = eomn_serv_diagn_mode_NONE;
        mcdiagnostics.config.par16 = 0;
    }

    delete event_downsampler;
    // in cleanup, at date of 23feb2016 there is a call to ethManager->releaseResource() which ...
    // send to config all the boards and stops tx and rx treads.
    // thus, in here we cannot call serviceStop(mc) because there will be tx/rx activity only for the first call of ::close().
    // i termporarily put serviceStop(eomn_serv_category_all) inside releaseResource()
    // todo: later on: clear regulars of mc, stop(mc), inside releaseresource() DO NOT stop tx/rx activity and DO NOT stop all services
    // res->serviceStop(eomn_serv_category_mc);
    // #warning TODO: clear the regulars imposed by motion-control.

    cleanup();

    return true;
}

void embObjMotionControl::cleanup(void)
{
    if(ethManager == NULL) return;

    int ret = ethManager->releaseResource2(res, this);
    res = NULL;
    if(ret == -1)
        ethManager->killYourself();
}


 //////////////// IethResource INTERFACE
eth::iethresType_t embObjMotionControl::type()
{
    return eth::iethres_motioncontrol;
}

bool embObjMotionControl::update(eOprotID32_t id32, double timestamp, void *rxdata)
{
    // use this function to update the values cached in the class using data received by the remote boards via the network callbacks
    // in embObjMotionControl it is updated only the timestamp of the encoders, thuus i dont used rxdata
    size_t joint = eoprot_ID2index(id32);
    eOprotEntity_t ent = eoprot_ID2entity(id32);
    eOprotTag_t tag = eoprot_ID2tag(id32);

    // rxdata = rxdata;

    // marco.accame: pay attention using rxdata. the rxdata depends on the id32.
    // now the function update() is called with rxdata of different types.
    // if the tag is eoprot_tag_mc_joint_status, then rxdata is of type eOmc_joint_status_t*
    // if the tag is eoprot_tag_mc_joint_status_basic, then rxdata is of type eOmc_joint_status_basic_t*


    // for the case of id32 which contains an encoder value .... we refresh the timestamp of that encoder

    if(true == initialised())
    {   // do it only if we already have opened the device
        std::lock_guard<std::mutex> lck(_mutex);
        _encodersStamp[joint] = timestamp;
    }


    if(eomn_serv_diagn_mode_MC_AMOyarp == mcdiagnostics.config.mode)
    {
        char str[128] = "boh";

        eoprot_ID2information(id32, str, sizeof(str));

        if((eoprot_entity_mc_joint == ent) && (eoprot_tag_mc_joint_status_debug == tag) && (joint < mcdiagnostics.ports.size()))
        {

            eOprotID32_t id32sc = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joint, eoprot_tag_mc_joint_status_core);
            eOmc_joint_status_core_t  jcore = {};

            res->getLocalValue(id32sc, &jcore);

            int32_t *debug32 = reinterpret_cast<int32_t*>(rxdata);
            // write into relevant port

            Bottle& output = mcdiagnostics.ports[joint]->prepare();
            output.clear();
            //output.addString("[yt, amo, reg, pos]"); // but we must get the joint and the motor as well
            output.addString("[yt, amo, reg, pos]");
            output.addFloat64(timestamp);
            output.addInt32(debug32[0]);
            output.addInt32(debug32[1]);
            output.addInt32(jcore.measures.meas_position);
            mcdiagnostics.ports[joint]->write();
        }
    }

    if((eoprot_entity_mc_motor == ent) && (eoprot_tag_mc_motor_status == tag))
    {
        if(false == initialised())
            return true;

        uint8_t motor = eoprot_ID2index(id32);
        if((_temperatureSensorsVector.at(motor)->getType() == motor_temperature_sensor_none))
            return true;

        eOmc_motor_status_t *mc_motor_status = reinterpret_cast<eOmc_motor_status_t*>(rxdata);
        
        if((double)mc_motor_status->basic.mot_temperature < 0 ) //I get a invalid value
        {
            if(! _temperatureSensorErrorWatchdog.at(motor).isStarted())
            {
                yWarning() << getBoardInfo() << "At time" << (yarp::os::Time::now() - _temperatureSensorErrorWatchdog.at(motor).getAbsoluteTime()) << "In motor" << motor << "cannot read Temperature from I2C. There might be cabling problems, TDB cable might be broken or sensor unreachable";
                _temperatureSensorErrorWatchdog.at(motor).start();
            }
            else
            {
                _temperatureSensorErrorWatchdog.at(motor).increment();
                if( _temperatureSensorErrorWatchdog.at(motor).isExpired())
                {
                    yWarning()<< getBoardInfo() << "Motor" << motor << "failed to read" << _temperatureSensorErrorWatchdog.at(motor).getCount() << "temperature readings for" << yarp::os::Time::now() - _temperatureSensorErrorWatchdog.at(motor).getStartTime() << "seconds";
                    _temperatureSensorErrorWatchdog.at(motor).start();
                }
            }
            return true;
        }
        
        //if I'm here I have a valid value
        double delta_tmp = 0;
        double tmp = _temperatureSensorsVector.at(motor)->convertRawToTempCelsius((double)mc_motor_status->basic.mot_temperature);
        
        // check if this is a spike or not
        // evaluate difference between current and previous temperature
        if(!_temperatureSpikesFilter.at(motor).isStarted()) //Pre-set of the filter buffer is ready
        {
            _temperatureSpikesFilter.at(motor).start(tmp);
            return true;
        }

        // when i'm here the filter is ready.
        delta_tmp = std::abs(tmp - _temperatureSpikesFilter.at(motor).getPrevTemperature());
        
        //1. check if I have a good value (not a spike)
        if(delta_tmp > _temperatureSpikesFilter.at(motor).getTemperatureThreshold())
        {
            //it is a spike
            return true;
        }
        // this is a not spike --> can update prev temperature
        _temperatureSpikesFilter.at(motor).updatePrevTemperature(tmp);
        
        //2. tmp is good and check the limits
        if(tmp > _temperatureLimits[motor].warningTemperatureLimit)
        {
            if(! _temperatureExceededLimitWatchdog.at(motor).isStarted())
            {
                yWarning() << getBoardInfo() << "Motor" << motor << "The temperature (" << tmp << "[ ℃  ] )  exceeds the warning limit (" << _temperatureLimits[motor].warningTemperatureLimit << "[ ℃  ] ). Processes not stopped but it is strongly recommended decreasing motor usage or reducing currents and PWMs to not risk motor damaging";
                _temperatureExceededLimitWatchdog.at(motor).start();
            }
            else
            {
                if(_temperatureExceededLimitWatchdog.at(motor).isExpired())
                {
                    yWarning() << getBoardInfo() << "Motor" << motor << "The temperature (" << tmp << "[ ℃  ] )  exceeds the warning limit (" << _temperatureLimits[motor].warningTemperatureLimit << "[ ℃  ] ) again!. Processes not stopped but it is strongly recommended decreasing motor usage or reducing currents and PWMs to not risk motor damaging";
                    _temperatureExceededLimitWatchdog.at(motor).start();
                }
                _temperatureExceededLimitWatchdog.at(motor).increment();
            }
        }
        else
        {
            _temperatureExceededLimitWatchdog.at(motor).clear();
        }
    }
    return true;
}


bool embObjMotionControl::getEntityName(uint32_t entityId, std::string &entityName)
{
    bool ret = getAxisNameRaw(entityId, entityName);

    //since getAxisNameRaw set "ERROR" in entityName when an error occurred,
    //while this function has to return an empty string, I reset the entityName string
    if(!ret)
    {
        entityName.clear();
    }
    return ret;

}


bool embObjMotionControl::getEncoderTypeName(uint32_t jomoId, eOmc_position_t pos, std::string &encoderTypeName)
{
    encoderTypeName.clear();
    
    if ((jomoId >= 0) && (jomoId < _njoints))
    {
        switch (pos)
        {
        case eomc_pos_atjoint:
            encoderTypeName = eomc_encoder2string(_jointEncs[jomoId].type, eobool_true);
            break;
        case eomc_pos_atmotor:
            encoderTypeName = eomc_encoder2string(_motorEncs[jomoId].type, eobool_true);
            break;
        case eomc_pos_unknown:
            encoderTypeName = "UNKNOWN";
            break;
        case eomc_pos_none:
        default:
            encoderTypeName = "NONE";
            break;
        }
        return true;
    }
    else
    {
        encoderTypeName = "ERROR";
        return false;
    }
}

bool embObjMotionControl::getEntityControlModeName(uint32_t entityId, eOenum08_t control_mode, std::string &controlModeName, eObool_t compact_string)
{
    controlModeName.clear();
    
    if ((entityId>= 0) && (entityId< _njoints))
    {
        controlModeName = eomc_controlmode2string(control_mode, compact_string);
        return true;
    }
    else
    {
        controlModeName = eomc_controlmode2string(eomc_ctrlmval_unknownError, compact_string);
        return false;
    }
}

///////////// PID INTERFACE
ReturnValue embObjMotionControl::setPidRaw(const PidControlTypeEnum& pidtype, int j, const Pid &pid)
{
    switch (pidtype)
    {
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
            helper_setPosPidRaw(j,pid);
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
            //helper_setVelPidRaw(j,pid);
            helper_setSpdPidRaw(j, pid);
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE:
            helper_setTrqPidRaw(j, pid);
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT:
            helper_setCurPidRaw(j,pid);
        break;
        default:
            yError()<<"Invalid pidtype:"<<static_cast<int>(pidtype);
        break;
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getPidRaw(const PidControlTypeEnum& pidtype, int axis, Pid *pid)
{
    auto ret = ReturnValue_error_generic;
    switch (pidtype)
    {
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
            ret = helper_getPosPidRaw(axis,pid);
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
            //helper_getVelPidRaw(axis,pid);
            ret = helper_getSpdPidRaw(axis, pid);
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE:
            ret = helper_getTrqPidRaw(axis, pid);
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT:
            ret = helper_getCurPidRaw(axis,pid);
        break;
        default:
            yError() << getBoardInfo() << "Invalid pidtype:"<<static_cast<int>(pidtype) << "in "<< __func__;
        break;
    }
    return ret;
}

ReturnValue embObjMotionControl::helper_setPosPidRaw(int j, const Pid &pid)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtrajectory);
    eOmc_PID_t  outPid;
    Pid hwPid = pid;

    copyPid_iCub2eo(&hwPid, &outPid);

    if(false == res->setRemoteValue(protoId, &outPid))
    {
        yError() << "while setting position PIDs for " << getBoardInfo() << " joint " << j;
        return ReturnValue_error_method_failed;
    }

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setPidsRaw(const PidControlTypeEnum& pidtype, const Pid *pids)
{
    ReturnValue ret = ReturnValue_ok;
    for(int j=0; j< _njoints; j++)
    {
        ret &= setPidRaw(pidtype, j, pids[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::setPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double ref)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::setPidReferencesRaw(const PidControlTypeEnum& pidtype, const double *refs)
{
    ReturnValue ret = ReturnValue_ok;
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= setPidReferenceRaw(pidtype, j, refs[index]);
    }
    return ret;
}

ReturnValue embObjMotionControl::setPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double limit)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::setPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, const double *limits)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getPidErrorRaw(const PidControlTypeEnum& pidtype, int j, double *err)
{
    uint16_t size = 0;
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t  jcore = {0};
    *err = 0;
    if(!res->getLocalValue(id32, &jcore))
        return ReturnValue_error_method_failed;

    switch(pidtype)
    {
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
        {
            if((eomc_controlmode_torque == jcore.modes.controlmodestatus)   ||
               (eomc_controlmode_openloop == jcore.modes.controlmodestatus) ||
               (eomc_controlmode_current == jcore.modes.controlmodestatus))
                    return ReturnValue_ok;
            else
                *err = (double) jcore.ofpid.generic.error1;
        }
        break;
        
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT:
        {
            *err=0;  //not yet implemented
            YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;
        
        case PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE:
        {
            if ((eOmc_interactionmode_compliant == jcore.modes.interactionmodestatus) &&
                (eomc_controlmode_position == jcore.modes.controlmodestatus))
            {
                *err = (double) jcore.ofpid.complpos.errtrq;
            }

            if(eomc_controlmode_torque == jcore.modes.controlmodestatus)
            {
                *err = (double) jcore.ofpid.torque.errtrq;
            }
        }
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
        {
            *err = 0;  //not yet implemented
            YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT:
        {
            *err = 0;  //not yet implemented
            YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;

        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT:
        {
            *err=0;  //not yet implemented
            YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;    
        
        default:
        {
            yError()<< getBoardInfo() << "Invalid pidtype:"<<static_cast<int>(pidtype) << "in "<< __func__;
        }
        break;
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getPidErrorsRaw(const PidControlTypeEnum& pidtype, double *errs)
{
    ReturnValue ret = ReturnValue_ok;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getPidErrorRaw(pidtype, j, &errs[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::helper_getPosPidRaw(int j, Pid *pid)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtrajectory);

    uint16_t size;
    eOmc_PID_t eoPID = {0};

    
#ifdef NETWORK_PERFORMANCE_BENCHMARK  
    double start = yarp::os::Time::now();
#endif
    
    bool ret = askRemoteValue(protid, &eoPID, size);

#ifdef NETWORK_PERFORMANCE_BENCHMARK  
    double end = yarp::os::Time::now();
    m_responseTimingVerifier.tick(end-start, start);
#endif
    
     if(!ret)
     {
        yError() << "failed helper_getPosPidsRaw for" << getBoardInfo();
        return ReturnValue_error_method_failed;
     }

    copyPid_eo2iCub(&eoPID, pid);
    
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_getPosPidsRaw(Pid *pid)
{
    std::vector<eOmc_PID_t> eoPIDList(_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_config_pidtrajectory, eoPIDList);
    if(!ret)
    {
        yError() << "failed helper_getPosPidsRaw for" << getBoardInfo();
        return ReturnValue_error_method_failed;
    }
    
    for(int j=0; j<_njoints; j++)
    {
        copyPid_eo2iCub(&eoPIDList[j], &pid[j]);
    }
    return ReturnValue_ok;
}


ReturnValue embObjMotionControl::getPidsRaw(const PidControlTypeEnum& pidtype, Pid *pids)
{
    ReturnValue ret = ReturnValue_ok;
    switch (pidtype)
    {
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
            ret = helper_getPosPidsRaw(pids);
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT   :
            ret = helper_getPosPidsRaw(pids);
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT   :
            ret = helper_getVelPidsRaw(pids);
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE:
            ret = helper_getTrqPidsRaw(pids);
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT:
            ret = helper_getCurPidsRaw(pids);
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
            ret = helper_getSpdPidsRaw(pids);
            break;
        default:
            yError()<< getBoardInfo() << "Invalid pidtype:"<<static_cast<int>(pidtype) << "in "<< __func__;
             ret = ReturnValue_error_generic;
            break;
    }
    return ret;
}

ReturnValue embObjMotionControl::getPidReferenceRaw(const PidControlTypeEnum& pidtype, int j, double *ref)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t jcore = {0};
    *ref = 0;
    if(!res->getLocalValue(id32, &jcore))
        return ReturnValue_error_method_failed;

    switch (pidtype)
    {
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
        {
            if((eomc_controlmode_torque == jcore.modes.controlmodestatus) ||
            (eomc_controlmode_openloop == jcore.modes.controlmodestatus) ||
            (eomc_controlmode_current == jcore.modes.controlmodestatus))
            { *ref = 0; yError() << getBoardInfo() << "Invalid getPidReferenceRaw() request for current control mode"; return ReturnValue_ok; }
            *ref = (double) jcore.ofpid.generic.reference1;
        }
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT:
        {
            *ref=0;
             YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE:
        {
            *ref = 0;
            YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT:
        {
            *ref=0;
            YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
        {
            *ref = 0;
            YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT:
        {
            *ref=0;
             YARP_METHOD_NOT_YET_IMPLEMENTED();
        }
        break;
        default:
        {
            *ref=0;
            yError()<< getBoardInfo() << "Invalid pidtype:"<<static_cast<int>(pidtype) << "in "<< __func__;
            return ReturnValue_error_generic;
        }
        break;
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getPidReferencesRaw(const PidControlTypeEnum& pidtype, double *refs)
{
    ReturnValue ret = ReturnValue_ok;

    // just one joint at time, wait answer before getting to the next.
    // This is because otherwise too many msg will be placed into can queue
    for(int j=0; j< _njoints; j++)
    {
        ret &= getPidReferenceRaw(pidtype, j, &refs[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::getPidErrorLimitRaw(const PidControlTypeEnum& pidtype, int j, double *limit)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getPidErrorLimitsRaw(const PidControlTypeEnum& pidtype, double *limits)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::resetPidRaw(const PidControlTypeEnum& pidtype, int j)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::disablePidRaw(const PidControlTypeEnum& pidtype, int j)
{
    return YARP_METHOD_DEPRECATED();
}

ReturnValue embObjMotionControl::enablePidRaw(const PidControlTypeEnum& pidtype, int j)
{
    return YARP_METHOD_DEPRECATED();
}

ReturnValue embObjMotionControl::setPidOffsetRaw(const PidControlTypeEnum& pidtype, int j, double v)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

////////////////////////////////////////
//    Velocity control interface raw  //
////////////////////////////////////////

ReturnValue embObjMotionControl::velocityMoveRaw(int j, double sp)
{
    int mode=0;
    getControlModeRaw(j, &mode);
    if( (mode != VOCAB_CM_VELOCITY) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_VEL) &&
        (mode != VOCAB_CM_IDLE))
    {
        if(event_downsampler->canprint())
        {
            yError() << "velocityMoveRaw: skipping command because " << getBoardInfo() << " joint " << j << " is not in VOCAB_CM_VELOCITY mode";
        }
        return ReturnValue_ok;
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    _ref_command_speeds[j] = sp ;   // save internally the new value of speed.

    eOmc_setpoint_t setpoint;
    setpoint.type = eomc_setpoint_velocity;
    setpoint.to.velocity.value =  (eOmeas_velocity_t) S_32(_ref_command_speeds[j]);
    setpoint.to.velocity.withacceleration = (eOmeas_acceleration_t) S_32(_ref_accs[j]);


    if(false == res->setRemoteValue(protid, &setpoint))
    {
        yError()<< getBoardInfo() << "while setting velocity mode";
        return ReturnValue_error_method_failed;
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::velocityMoveRaw(const double *sp)
{
    ReturnValue ret = ReturnValue_ok;

    for(int j=0; j<_njoints; j++)
    {
        ret &= velocityMoveRaw(j, sp[j]);
    }

    return ret;
}


////////////////////////////////////////
//    Calibration control interface   //
////////////////////////////////////////

ReturnValue embObjMotionControl::setCalibrationParametersRaw(int j, const CalibrationParameters& params)
{
    yTrace() << "setCalibrationParametersRaw for " << getBoardInfo() << "joint" << j;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_calibration);
    eOmc_calibrator_t calib;
    memset(&calib, 0x00, sizeof(calib));
    calib.type = params.type;

    switch (calib.type)
    {
        // muove -> amp+pid, poi calib
    case eomc_calibration_type0_hard_stops:
        calib.params.type0.pwmlimit = (int16_t)S_16(params.param1);
        calib.params.type0.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type0.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        // fermo
    case eomc_calibration_type1_abs_sens_analog:
        calib.params.type1.position = (int16_t)S_16(params.param1);
        calib.params.type1.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type1.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        // muove
    case eomc_calibration_type2_hard_stops_diff:
        calib.params.type2.pwmlimit = (int16_t)S_16(params.param1);
        calib.params.type2.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type2.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        // muove
    case eomc_calibration_type3_abs_sens_digital:
        calib.params.type3.position = (int16_t)S_16(params.param1);
        calib.params.type3.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type3.offset = (int32_t)S_32(params.param3);
        calib.params.type3.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        // muove
    case eomc_calibration_type4_abs_and_incremental:
        calib.params.type4.position = (int16_t)S_16(params.param1);
        calib.params.type4.velocity = (eOmeas_velocity_t)S_32(params.param2);
        calib.params.type4.maxencoder = (int32_t)S_32(params.param3);
        calib.params.type4.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        // muove
    case eomc_calibration_type5_hard_stops:
        calib.params.type5.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type5.final_pos = (int32_t) S_32(params.param2);
        calib.params.type5.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        // muove
    case eomc_calibration_type6_mais:
        calib.params.type6.position = (int32_t)S_32(params.param1);
        calib.params.type6.velocity = (int32_t)S_32(params.param2);
        calib.params.type6.current = (int32_t)S_32(params.param3);
        calib.params.type6.vmin = (int32_t)S_32(params.param4);
        calib.params.type6.vmax = (int32_t)S_32(params.param5);
        calib.params.type6.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        // muove
    case eomc_calibration_type7_hall_sensor:
        calib.params.type7.position = (int32_t)S_32(params.param1);
        calib.params.type7.velocity = (int32_t)S_32(params.param2);
        //param3 is not used
        calib.params.type7.vmin = (int32_t)S_32(params.param4);
        calib.params.type7.vmax = (int32_t)S_32(params.param5);
        calib.params.type7.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

        //muove
    case eomc_calibration_type8_tripod_internal_hard_stop:
        calib.params.type8.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type8.max_delta  = (int32_t) S_32(params.param2);
        calib.params.type8.calibrationZero = (int32_t)S_32(params.paramZero /* * _angleToEncoder[j] */);
        break;

    case eomc_calibration_type9_tripod_external_hard_stop:
        calib.params.type9.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type9.max_delta  = (int32_t) S_32(params.param2);
        calib.params.type9.calibrationZero = (int32_t)S_32(params.paramZero /* * _angleToEncoder[j] */);
        break;

    case eomc_calibration_type10_abs_hard_stop:
        calib.params.type10.pwmlimit   = (int32_t) S_32(params.param1);
        calib.params.type10.calibrationZero = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

    case eomc_calibration_type11_cer_hands:
        calib.params.type11.offset0     = (int32_t)S_32(params.param1);
        calib.params.type11.offset1     = (int32_t)S_32(params.param2);
        calib.params.type11.offset2     = (int32_t)S_32(params.param3);
        calib.params.type11.cable_range = (int32_t)S_32(params.param4);
        calib.params.type11.pwm         = (int32_t)S_32(params.param5);
        //calib.params.type11.calibrationZero = 32767;//(int32_t)S_32(params.paramZero * _angleToEncoder[j]);
        break;

    case eomc_calibration_type12_absolute_sensor:
        calib.params.type12.rawValueAtZeroPos  = (int32_t)S_32(params.param1);
        calib.params.type12.calibrationDelta = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));
        break;

    case eomc_calibration_type13_cer_hands_2:
        calib.params.type13.rawValueAtZeroPos0     = (int32_t)S_32(params.param1);
        calib.params.type13.rawValueAtZeroPos1     = (int32_t)S_32(params.param2);
        calib.params.type13.rawValueAtZeroPos2     = (int32_t)S_32(params.param3);
        calib.params.type13.rawValueAtZeroPos3     = (int32_t)S_32(params.param4);
        break;
    
    case eomc_calibration_type14_qenc_hard_stop_and_fap:
        calib.params.type14.pwmlimit               = (int32_t)S_32(params.param1);
        calib.params.type14.final_pos              = (int32_t)S_32(params.param2);
        calib.params.type14.invertdirection        = (uint8_t)U_32(params.param3);
        calib.params.type14.rotation               = (int32_t)S_32(params.param4);

        if (calib.params.type14.invertdirection != 0 && calib.params.type14.invertdirection != 1)
        {
            yError() <<  getBoardInfo() << "Error in param3 of calibartion type 14 for joint " << j << "Admitted values are: 0=FALSE and 1=TRUE";
            return ReturnValue_error_generic;
        }
        

        if(!checkCalib14RotationParam(calib.params.type14.rotation))
        {
            yError() <<  getBoardInfo() << "Error in param4 of calibartion type 14 for joint " << j << "Admitted values are: 0, 32768, 16384, -16384 [0, 180, 90, -90] in iCubDegree";
            return ReturnValue_error_generic;
        }
        calib.params.type14.offset                 = (int32_t)S_32(params.param5);
        calib.params.type14.calibrationZero        = (int32_t)S_32(_measureConverter->posA2E(params.paramZero, j));

        break;

    default:
        yError() <<  getBoardInfo() << "joint" << j << "Calibration type unknown!! (embObjMotionControl)\n";
        return ReturnValue_error_generic;
        break;
    }

    if (false == res->setRemoteValue(protid, &calib))
    {
        yError() <<  getBoardInfo() << "joint" << j << "while setting remote calibration parameters";
        return ReturnValue_error_method_failed;
    }

    _calibrated[j] = true;

    return ReturnValue_ok;
}

bool embObjMotionControl::checkCalib14RotationParam(int32_t calib_param4)
{
    eOmc_calib14_ROT_t urotation = eomc_int2calib14_ROT(calib_param4);
    
    if (urotation == eOmc_calib14_ROT_zero || 
        urotation == eOmc_calib14_ROT_plus180 ||
        urotation == eOmc_calib14_ROT_plus090 ||
        urotation == eOmc_calib14_ROT_minus090)
    {
        return ReturnValue_ok;
    }

    return false;
}

ReturnValue embObjMotionControl::calibrateAxisWithParamsRaw(int j, unsigned int type, double p1, double p2, double p3)
{
    yTrace() << "calibrateRaw for" << getBoardInfo() << "joint" << j;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_calibration);
    eOmc_calibrator_t calib;
    memset(&calib, 0x00, sizeof(calib));
    calib.type = type;

    switch(type)
    {
        // muove -> amp+pid, poi calib
    case eomc_calibration_type0_hard_stops:
        calib.params.type0.pwmlimit = (int16_t) S_16(p1);
        calib.params.type0.velocity = (eOmeas_velocity_t) S_32(p2);
        break;

        // fermo
    case eomc_calibration_type1_abs_sens_analog:
        calib.params.type1.position = (int16_t) S_16(p1);
        calib.params.type1.velocity = (eOmeas_velocity_t)  S_32(p2);
        break;

        // muove
    case eomc_calibration_type2_hard_stops_diff:
        calib.params.type2.pwmlimit = (int16_t) S_16(p1);
        calib.params.type2.velocity = (eOmeas_velocity_t)  S_32(p2);
        break;

        // muove
    case eomc_calibration_type3_abs_sens_digital:
        calib.params.type3.position = (int16_t) S_16(p1);
        calib.params.type3.velocity = (eOmeas_velocity_t)  S_32(p2);
        calib.params.type3.offset   = (int32_t) S_32(p3);
        break;

        // muove
    case eomc_calibration_type4_abs_and_incremental:
        calib.params.type4.position   = (int16_t) S_16(p1);
        calib.params.type4.velocity   = (eOmeas_velocity_t)  S_32(p2);
        calib.params.type4.maxencoder = (int32_t) S_32(p3);
        break;

   default:
        yError() <<  getBoardInfo() << "joint" << j << "Calibration type unknown!! (embObjMotionControl)\n";
        return ReturnValue_error_generic;
        break;
    }

    if (false == res->setRemoteValue(protid, &calib))
    {
        yError() <<  getBoardInfo() << "joint" << j << "while setting remote calibration parameters";
        return ReturnValue_error_method_failed;
    }
    _calibrated[j ] = true;

    return ReturnValue_ok;
}


std::string embObjMotionControl::controlModeType2String(eOmc_controlmode_t type)
{
    switch(type)
    {
        case eomc_controlmode_idle:
            return "IDLE";
        case eomc_controlmode_calib:
            return "CALIBRATION";
        case eomc_controlmode_notConfigured:
            return "NOT CONFIGURED";
        case eomc_controlmode_hwFault:
            return "HARDWARE FAULT";
        case eomc_controlmode_unknownError:
            return "UNKNOWN ERROR";
        case eomc_controlmode_configured:
            return "CONFIGURED";
        default:
            return "OTHER THAN CALIBRATION";
    }
}

ReturnValue embObjMotionControl::calibrationDoneRaw(int axis)
{
    eOmc_joint_status_core_t jcore = {0};
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_core);
    if(!res->getLocalValue(id32, &jcore))
    {
        yError() << getBoardInfo() << "Failure of getLocalValue() for axis" << axis << "in "<< __YFUNCTION__;
        return ReturnValue_error_method_failed;
    }

    eOmc_controlmode_t type = (eOmc_controlmode_t) jcore.modes.controlmodestatus;
    ReturnValue result = ReturnValue_error_generic;
    switch(type)
    {
        case eomc_controlmode_idle:
            return ((_maintenanceModeCfg.enableSkipRecalibration) ? ReturnValue_ok : ReturnValue_error_generic);
        case eomc_controlmode_calib:
        case eomc_controlmode_hwFault:
        case eomc_controlmode_notConfigured:
        case eomc_controlmode_unknownError:
        case eomc_controlmode_configured:
        {   
            yError() << getBoardInfo()<< "Unable to complete calibration: joint" << axis << " is in " << controlModeType2String(type) << " status in "<<  __YFUNCTION__;
            return ReturnValue_error_generic;
        } ;

        default:  // if the control mode is no longer a calibration type, it means calibration ended
            return ReturnValue_ok;
    }

}

////////////////////////////////////////
//     Position control interface     //
////////////////////////////////////////

ReturnValue embObjMotionControl::getAxes(int *ax)
{
    *ax=_njoints;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::positionMoveRaw(int j, double ref)
{
    if (yarp::os::Time::now()-_last_position_move_time[j]<MAX_POSITION_MOVE_INTERVAL)
    {
        yWarning() << getBoardInfo() << "Performance warning: You are using positionMove commands at high rate (<"<< MAX_POSITION_MOVE_INTERVAL*1000.0 <<" ms). Probably position control mode is not the right control mode to use.";
    }
    _last_position_move_time[j] = yarp::os::Time::now();

    int mode = 0;
    getControlModeRaw(j, &mode);
    if( (mode != VOCAB_CM_POSITION) &&
        (mode != VOCAB_CM_MIXED) &&
        (mode != VOCAB_CM_IMPEDANCE_POS) &&
        (mode != VOCAB_CM_IDLE))
    {
        if (event_downsampler->canprint())
        {
            yError() << "positionMoveRaw: skipping command because " << getBoardInfo() << " joint " << j << " is not in VOCAB_CM_POSITION mode";
        }
        return ReturnValue_ok;
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    _ref_command_positions[j] = ref;   // save internally the new value of pos.

    eOmc_setpoint_t setpoint;

    setpoint.type = (eOenum08_t) eomc_setpoint_position;
    setpoint.to.position.value =  (eOmeas_position_t) S_32(_ref_command_positions[j]);
    setpoint.to.position.withvelocity = (eOmeas_velocity_t) S_32(_ref_speeds[j]);

    return res->setRemoteValue(protid, &setpoint) ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::positionMoveRaw(const double *refs)
{
    bool ret = true;

    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        ret &= positionMoveRaw(j, refs[index]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::relativeMoveRaw(int j, double delta)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::relativeMoveRaw(const double *deltas)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}


ReturnValue embObjMotionControl::checkMotionDoneRaw(int j, bool *flag)
{
    eObool_t ismotiondone = eobool_false;
    uint16_t size = 0;

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core_modes_ismotiondone);
    if(false == askRemoteValue(id32, &ismotiondone, size))
    {
        yError () << "Failure of askRemoteValue() inside embObjMotionControl::checkMotionDoneRaw(j=" << j << ") for " << getBoardInfo();
        return ReturnValue_error_generic;
    }


    *flag = ismotiondone; // eObool_t can have values only amongst: eobool_true (1) or eobool_false (0).

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::checkMotionDoneRaw(bool *flag)
{
    std::vector <eObool_t> ismotiondoneList(_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_status_core_modes_ismotiondone, ismotiondoneList);
    if(false == ret)
    {
        yError () << "Failure of askRemoteValues() inside embObjMotionControl::checkMotionDoneRaw for all joints of" << getBoardInfo();
        return ReturnValue_error_generic;
    }
    *flag=true;
    for(int j=0; j<_njoints; j++)
    {
        *flag &= ismotiondoneList[j]; // eObool_t can have values only amongst: eobool_true (1) or eobool_false (0).
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setTrajSpeedRaw(int j, double sp)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    _ref_speeds[j] = sp;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setTrajSpeedsRaw(const double *spds)
{
    // Velocity is expressed in iDegrees/s
    // save internally the new value of speed; it'll be used in the positionMove
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        setTrajSpeedRaw(j, spds[index]);
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setTrajAccelerationRaw(int j, double acc)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command

    if (acc > 1e6)
    {
        _ref_accs[j ] =  1e6;
    }
    else if (acc < -1e6)
    {
        _ref_accs[j ] = -1e6;
    }
    else
    {
        _ref_accs[j ] = acc;
    }

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setTrajAccelerationsRaw(const double *accs)
{
    // Acceleration is expressed in iDegrees/s^2
    // save internally the new value of the acceleration; it'll be used in the velocityMove command
    for(int j=0, index=0; j< _njoints; j++, index++)
    {
        if (accs[j] > 1e6)
        {
            _ref_accs[index] =  1e6;
        }
        else if (accs[j] < -1e6)
        {
            _ref_accs[index] = -1e6;
        }
        else
        {
            _ref_accs[index] = accs[j];
        }
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getTrajSpeedRaw(int j, double *spd)
{
    if (j<0 || j>_njoints) return ReturnValue_error_generic; //TODO: is necessary this check?
#if ASK_REFERENCE_TO_FIRMWARE
    *spd = _ref_speeds[j];
    //return YARP_METHOD_NOT_YET_IMPLEMENTED();
#else
    *spd = _ref_speeds[j];
#endif
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getTrajSpeedsRaw(double *spds)
{
    memcpy(spds, _ref_speeds, sizeof(double) * _njoints);
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getTrajAccelerationRaw(int j, double *acc)
{
    *acc = _ref_accs[j];
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getTrajAccelerationsRaw(double *accs)
{
    memcpy(accs, _ref_accs, sizeof(double) * _njoints);
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::stopRaw(int j)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_stoptrajectory);

    eObool_t stop = eobool_true;

    return res->setRemoteValue(protid, &stop) ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::stopRaw()
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= stopRaw(j);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}
///////////// END Position Control INTERFACE  //////////////////

////////////////////////////////////////
//     Position control2 interface    //
////////////////////////////////////////

ReturnValue embObjMotionControl::positionMoveRaw(const int n_joint, const int *joints, const double *refs)
{
    ReturnValue ret = ReturnValue_ok;
    for(int j=0; j<n_joint; j++)
    {
        ret &= positionMoveRaw(joints[j], refs[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::relativeMoveRaw(const int n_joint, const int *joints, const double *deltas)
{
    ReturnValue ret = ReturnValue_ok;
    for(int j=0; j<n_joint; j++)
    {
        ret &= relativeMoveRaw(joints[j], deltas[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::checkMotionDoneRaw(const int n_joint, const int *joints, bool *flag)
{

    //1) first of all, check if all joints number are ok
    for(int j=0; j<n_joint; j++)
    {
        if(joints[j] >= _njoints)
        {
            yError() << getBoardInfo() << ":checkMotionDoneRaw required for not existing joint ( " << joints[j] << ")";
            return ReturnValue_error_generic;
        }
    } //TODO: is necessary this check?

    //2) ask check motion done for all my joints
    std::vector <eObool_t> ismotiondoneList(_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_status_core_modes_ismotiondone, ismotiondoneList);
    if(false == ret)
    {
        yError () << getBoardInfo() << "Failure of askRemoteValues() inside embObjMotionControl::checkMotionDoneRaw for a group of joint"; getBoardInfo();
        return ReturnValue_error_generic;
    }

    //3) verify only the given joints
    bool tot_val = true;
    for(int j=0; j<n_joint; j++)
    {
        tot_val &= ismotiondoneList[joints[j]];
    }

    *flag = tot_val;
    return ReturnValue_ok;

}

ReturnValue embObjMotionControl::setTrajSpeedsRaw(const int n_joint, const int *joints, const double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&setTrajSpeedRaw(joints[j], spds[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::setTrajAccelerationsRaw(const int n_joint, const int *joints, const double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&setTrajAccelerationRaw(joints[j], accs[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getTrajSpeedsRaw(const int n_joint, const int *joints, double *spds)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getTrajSpeedRaw(joints[j], &spds[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getTrajAccelerationsRaw(const int n_joint, const int *joints, double *accs)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret && getTrajAccelerationRaw(joints[j], &accs[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::stopRaw(const int n_joint, const int *joints)
{
    bool ret = true;
    for(int j=0; j<n_joint; j++)
    {
        ret = ret &&stopRaw(joints[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

///////////// END Position Control INTERFACE  //////////////////

// ControlMode

ReturnValue embObjMotionControl::getControlModeRaw(int j, int *v)
{
    eOmc_joint_status_core_t jcore = {0};
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    if(! res->getLocalValue(protid, &jcore))
        return ReturnValue_error_generic;

    eOmc_controlmode_t type = (eOmc_controlmode_t) jcore.modes.controlmodestatus;

    *v = controlModeStatusConvert_embObj2yarp(type);
    return ReturnValue_ok;
}

// IControl Mode 2
ReturnValue embObjMotionControl::getControlModesRaw(int* v)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret = ret && getControlModeRaw(j, &v[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        ret = ret && getControlModeRaw(joints[j], &modes[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}



// marco.accame: con alberto cardellino abbiamo parlato della correttezza di effettuare la verifica di quanto imposto (in setControlModeRaw() ed affini)
// andando a rileggere il valore nella scheda eth fino a che esso non sia quello atteso. si deve fare oppure no?
// con il control mode il can ora lo fa ma e' giusto? era cosi' anche in passato?
ReturnValue embObjMotionControl::setControlModeRaw(const int j, const int _mode)
{
    bool ret = true;
    eOenum08_t controlmodecommand = 0;

    if((_mode == VOCAB_CM_TORQUE) && (_trq_pids[j].enabled  == false))
    {
        yError()<<"Torque control is disabled. Check your configuration parameters";
        return ReturnValue_error_generic;
    }

    if(!controlModeCommandConvert_yarp2embObj(_mode, controlmodecommand) )
    {
        yError() << "SetControlMode: received unknown control mode for " << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(_mode);
        return ReturnValue_error_generic;
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_controlmode);
    if(false == res->setRemoteValue(protid, &controlmodecommand) )
    {
        yError() << "setControlModeRaw failed for " << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(_mode);
        return ReturnValue_error_generic;
    }


    ret = checkRemoteControlModeStatus(j, _mode);

    if(false == ret)
    {
        yError() << "In embObjMotionControl::setControlModeRaw(j=" << j << ", mode=" << yarp::os::Vocab32::decode(_mode).c_str() << ") for " << getBoardInfo() << " has failed checkRemoteControlModeStatus()";
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}


ReturnValue embObjMotionControl::setControlModesRaw(const int n_joint, const int *joints, int *modes)
{
    bool ret = true;
    eOenum08_t controlmodecommand = 0;


    for(int i=0; i<n_joint; i++)
    {
        if ((modes[i] == VOCAB_CM_TORQUE) && (_trq_pids[i].enabled  == false)) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        if(!controlModeCommandConvert_yarp2embObj(modes[i], controlmodecommand) )
        {
            yError() << "SetControlModesRaw(): received unknown control mode for " << getBoardInfo() << " joint " << joints[i] << " mode " << Vocab32::decode(modes[i]);

            return ReturnValue_error_generic;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joints[i], eoprot_tag_mc_joint_cmmnds_controlmode);
        if(false == res->setRemoteValue(protid, &controlmodecommand) )
        {
            yError() << "setControlModesRaw() could not send set<cmmnds_controlmode> for " << getBoardInfo() << " joint " << joints[i] << " mode " << Vocab32::decode(modes[i]);

            return ReturnValue_error_generic;
        }

        bool tmpresult = checkRemoteControlModeStatus(joints[i], modes[i]);
        if(false == tmpresult)
        {
            yError() << "setControlModesRaw(const int n_joint, const int *joints, int *modes) could not check with checkRemoteControlModeStatus() for " << getBoardInfo() << " joint " << joints[i] << " mode " << Vocab32::decode(modes[i]);
        }

        ret = ret && tmpresult;

    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::setControlModesRaw(int *modes)
{
    bool ret = true;
    eOenum08_t controlmodecommand = 0;

    for(int i=0; i<_njoints; i++)
    {

        if ((modes[i] == VOCAB_CM_TORQUE) && (_trq_pids[i].enabled  == false))
        {
            yError()<<"Torque control is disabled. Check your configuration parameters";
            continue;
        }

        if(!controlModeCommandConvert_yarp2embObj(modes[i], controlmodecommand) )
        {
            yError() << "SetControlMode: received unknown control mode for" << getBoardInfo() << " joint " << i << " mode " << Vocab32::decode(modes[i]);
            return ReturnValue_error_generic;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, i, eoprot_tag_mc_joint_cmmnds_controlmode);
        if(false == res->setRemoteValue(protid, &controlmodecommand) )
        {
            yError() << "setControlModesRaw failed for " << getBoardInfo() << " joint " << i << " mode " << Vocab32::decode(modes[i]);
            return ReturnValue_error_generic;
        }

        bool tmpresult = checkRemoteControlModeStatus(i, modes[i]);
        if(false == tmpresult)
        {
            yError() << "setControlModesRaw(int *modes) could not check with checkRemoteControlModeStatus() for" << getBoardInfo() << " joint " << i << " mode " << Vocab32::decode(modes[i]);
        }

        ret = ret && tmpresult;

    }


    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}


//////////////////////// BEGIN EncoderInterface

ReturnValue embObjMotionControl::setEncoderRaw(int j, double val)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::setEncodersRaw(const double *vals)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::resetEncoderRaw(int j)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::resetEncodersRaw()
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getEncoderRaw(int j, double *value)
{
    eOmc_joint_status_core_t core;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);

    bool ret = res->getLocalValue(protid, &core);

    if(ret)
    {
        *value = (double) core.measures.meas_position;
    }
    else
    {
        yError() << "embObjMotionControl while reading encoder";
        *value = 0;
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getEncodersRaw(double *encs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderRaw(j, &encs[j]);

    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getEncoderSpeedRaw(int j, double *sp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t core;
    *sp = 0;
    if(!res->getLocalValue(protid, &core))
    {
        return ReturnValue_error_generic;
    }
    // extract requested data from status
    *sp = (double) core.measures.meas_velocity;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getEncoderSpeedsRaw(double *spds)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderSpeedRaw(j, &spds[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getEncoderAccelerationRaw(int j, double *acc)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t core;
    *acc = 0;
    if(! res->getLocalValue(protid, &core))
    {
        return ReturnValue_error_generic;
    }
    *acc = (double) core.measures.meas_acceleration;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getEncoderAccelerationsRaw(double *accs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getEncoderAccelerationRaw(j, &accs[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

///////////////////////// END Encoder Interface

ReturnValue embObjMotionControl::getEncodersTimedRaw(double *encs, double *stamps)
{
    bool ret = getEncodersRaw(encs);
    std::lock_guard<std::mutex> lck(_mutex);
    for(int i=0; i<_njoints; i++)
        stamps[i] = _encodersStamp[i];
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getEncoderTimedRaw(int j, double *encs, double *stamp)
{
    bool ret = getEncoderRaw(j, encs);
    std::lock_guard<std::mutex> lck(_mutex);
    *stamp = _encodersStamp[j];
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

//////////////////////// BEGIN EncoderInterface

ReturnValue embObjMotionControl::getNumberOfMotorEncodersRaw(int* num)
{
    *num=_njoints;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setMotorEncoderRaw(int m, const double val)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::setMotorEncodersRaw(const double *vals)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::setMotorEncoderCountsPerRevolutionRaw(int m, const double cpr)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getMotorEncoderCountsPerRevolutionRaw(int m, double *cpr)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::resetMotorEncoderRaw(int mj)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::resetMotorEncodersRaw()
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getMotorEncoderRaw(int m, double *value)
{
    eOmc_motor_status_basic_t status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);

    bool ret = res->getLocalValue(protid, &status);
    if(ret)
    {
        *value = (double) status.mot_position;
    }
    else
    {
        yError() << "embObjMotionControl while reading motor encoder position";
        *value = 0;
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getMotorEncodersRaw(double *encs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getMotorEncoderRaw(j, &encs[j]);

    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getMotorEncoderSpeedRaw(int m, double *sp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->getLocalValue(protid, &tmpMotorStatus);
    if(ret)
    {
        *sp = (double) tmpMotorStatus.mot_velocity;
    }
    else
    {
        yError() << "embObjMotionControl while reading motor encoder speed";
        *sp = 0;
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getMotorEncoderSpeedsRaw(double *spds)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getMotorEncoderSpeedRaw(j, &spds[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getMotorEncoderAccelerationRaw(int m, double *acc)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->getLocalValue(protid, &tmpMotorStatus);
    if(ret)
    {
        *acc = (double) tmpMotorStatus.mot_acceleration;
    }
    else
    {
        yError() << "embObjMotionControl while reading motor encoder acceleration";
        *acc = 0;
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getMotorEncoderAccelerationsRaw(double *accs)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getMotorEncoderAccelerationRaw(j, &accs[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getMotorEncodersTimedRaw(double *encs, double *stamps)
{
    bool ret = getMotorEncodersRaw(encs);
    std::lock_guard<std::mutex> lck(_mutex);
    for(int i=0; i<_njoints; i++)
        stamps[i] = _encodersStamp[i];
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getMotorEncoderTimedRaw(int m, double *encs, double *stamp)
{
    bool ret = getMotorEncoderRaw(m, encs);
    std::lock_guard<std::mutex> lck(_mutex);
    *stamp = _encodersStamp[m];
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}
///////////////////////// END Motor Encoder Interface

////// Amplifier interface

ReturnValue embObjMotionControl::enableAmpRaw(int j)
{
    return YARP_METHOD_DEPRECATED();
}

ReturnValue embObjMotionControl::disableAmpRaw(int j)
{
    return YARP_METHOD_DEPRECATED();
}

ReturnValue embObjMotionControl::getCurrentRaw(int j, double *value)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);
    eOmc_motor_status_basic_t  tmpMotorStatus;
    bool ret = res->getLocalValue(protid, &tmpMotorStatus);

    *value = (double) tmpMotorStatus.mot_current;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getCurrentsRaw(double *vals)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getCurrentRaw(j, &vals[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::setMaxCurrentRaw(int j, double val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_currentlimits);
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    
    if(!askRemoteValue(protid, &currentlimits, size))
    {
        yError() << "embObjMotionControl::setMaxCurrentRaw() could not read max current for " << getBoardInfo() << "joint " << j;
        return ReturnValue_error_generic;
    }

    //set current overload
    currentlimits.overloadCurrent = (eOmeas_current_t) S_16(val);

    //send new values
    return res->setRemoteValue(protid, &currentlimits) ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getMaxCurrentRaw(int j, double *val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_currentlimits);
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    *val = 0;

    if(!askRemoteValue(protid, &currentlimits, size))
    {
        yError() << "embObjMotionControl::getMaxCurrentRaw() could not read max current for " << getBoardInfo() << "joint " << j;
        return ReturnValue_error_generic;
    }

    *val = (double) currentlimits.overloadCurrent;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getAmpStatusRaw(int j, int *st)
{
 //VALE: can i set this func like YARP_METHOD_DEPRECATED? none sets _enabledAmp!!
    (_enabledAmp[j ]) ? *st = 1 : *st = 0;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getAmpStatusRaw(int *sts)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
    {
        sts[j] = _enabledAmp[j];
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

#ifdef IMPLEMENT_DEBUG_INTERFACE
//----------------------------------------------\\
//    Debug interface
//----------------------------------------------\\

bool embObjMotionControl::setParameterRaw(int j, unsigned int type, double value)   {       return YARP_METHOD_NOT_YET_IMPLEMENTED(); }
bool embObjMotionControl::getParameterRaw(int j, unsigned int type, double* value)  {       return YARP_METHOD_NOT_YET_IMPLEMENTED(); }
bool embObjMotionControl::getDebugParameterRaw(int j, unsigned int index, double* value)  { return YARP_METHOD_NOT_YET_IMPLEMENTED(); }
bool embObjMotionControl::setDebugParameterRaw(int j, unsigned int index, double value)   { return YARP_METHOD_NOT_YET_IMPLEMENTED(); }
bool embObjMotionControl::setDebugReferencePositionRaw(int j, double value)         {       return YARP_METHOD_NOT_YET_IMPLEMENTED(); }
bool embObjMotionControl::getDebugReferencePositionRaw(int j, double* value)        {       return YARP_METHOD_NOT_YET_IMPLEMENTED();}

#endif //IMPLEMENT_DEBUG_INTERFACE

// Limit interface
ReturnValue embObjMotionControl::setPosLimitsRaw(int j, double min, double max)
{
    bool ret = true;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_userlimits);

    eOmeas_position_limits_t    limits;
    limits.max = (eOmeas_position_t) S_32(max);
    limits.min = (eOmeas_position_t) S_32(min);

    ret = res->setRemoteValue(protid, &limits);


    if(!ret)
    {
        yError() << "while setting position limits for joint" << j << " \n";
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getPosLimitsRaw(int j, double *min, double *max)
{
    eOmeas_position_limits_t limits;
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_userlimits);
    uint16_t size;
    
    if(! askRemoteValue(protoid, &limits, size))
        return ReturnValue_error_method_failed;

    *min = (double)limits.min + SAFETY_THRESHOLD;
    *max = (double)limits.max - SAFETY_THRESHOLD;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getGearboxRatioRaw(int j, double *gearbox)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return ReturnValue_error_generic;

    // refresh cached value when reading data from the EMS
    *gearbox = (double)motor_cfg.gearbox_M2J;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setGearboxRatioRaw(int j, const double val)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t motor_cfg = {0};

    
    if(!askRemoteValue(protoid, &motor_cfg, size))
    {
        yError() << "setGearboxRatioRaw: failed to GET motor config for joint" << j;
        return ReturnValue_error_method_failed;
    }

    motor_cfg.gearbox_M2J = (double)val;

    // Now, write the modified configuration back
    if(false == res->setRemoteValue(protoid, &motor_cfg))
    {
        yError() << "setGearboxRatioRaw: failed to SET motor config for joint" << j;
        return ReturnValue_error_method_failed;
    }
    
    return ReturnValue_ok;
}

bool embObjMotionControl::getRotorLimitsRaw(int j, double *rotorMin, double *rotorMax)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;
    *rotorMax = (double)( motor_cfg.limitsofrotor.max);
    *rotorMin = (double)( motor_cfg.limitsofrotor.min);
    return true;
}

bool embObjMotionControl::getTorqueControlFilterType(int j, int& type)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config);
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    if(! askRemoteValue(protoid, &joint_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    type = (int)joint_cfg.tcfiltertype;
    return true;
}

bool embObjMotionControl::getRotorEncoderResolutionRaw(int j, double &rotres)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    rotres = (double)motor_cfg.rotorEncoderResolution;

    return ReturnValue_ok;
}

bool embObjMotionControl::getJointEncoderResolutionRaw(int j, double &jntres)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config);
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    if(! askRemoteValue(protoid, &joint_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    jntres = (double)joint_cfg.jntEncoderResolution;

    return true;
}

bool embObjMotionControl::getJointEncoderTypeRaw(int j, int &type)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config);
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    if(! askRemoteValue(protoid, &joint_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    type = (int)joint_cfg.jntEncoderType;

    return true;
}

bool embObjMotionControl::getRotorEncoderTypeRaw(int j, int &type)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    type = (int)motor_cfg.rotorEncoderType;

    return true;
}

bool embObjMotionControl::getKinematicMJRaw(int j, double &rotres)
{
    yError("getKinematicMJRaw not yet  implemented");
    return false;
}

bool embObjMotionControl::getTemperatureSensorTypeRaw(int j, std::string& ret)
{
    // refresh cached value when reading data from the EMS
    ret = "NONE";
    if (_temperatureSensorsVector.at(j)->getType() == motor_temperature_sensor_pt100)
    {
        ret = "PT100";
    }
    else if (_temperatureSensorsVector.at(j)->getType() == motor_temperature_sensor_pt1000)
    {
        ret = "PT1000";
    }
    else
    {
        ret = "NONE";
    }
    
    return true;
}

bool embObjMotionControl::getHasTempSensorsRaw(int j, int& ret) 
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasTempSensor;

    return true;
}

bool embObjMotionControl::getHasHallSensorRaw(int j, int& ret)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;
    
    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasHallSensor;

    return true;
}

bool embObjMotionControl::getHasRotorEncoderRaw(int j, int& ret)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasRotorEncoder;

    return true;
}

bool embObjMotionControl::getHasRotorEncoderIndexRaw(int j, int& ret)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    ret = (int)motor_cfg.hasRotorEncoderIndex;

    return true;
}

bool embObjMotionControl::getMotorPolesRaw(int j, int& poles)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);

    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;
    
    // refresh cached value when reading data from the EMS
    poles = (int)motor_cfg.motorPoles;

    return true;
}

bool embObjMotionControl::getRotorIndexOffsetRaw(int j, double& rotorOffset)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    rotorOffset = (double)motor_cfg.rotorIndexOffset;

    return true;
}

ReturnValue embObjMotionControl::getAxisNameRaw(int axis, std::string& name)
{
    if (axis >= 0 && axis < _njoints)
    {
        name = _axesInfo[axis].name;
        return ReturnValue_ok;
    }
    else
    {
        name = "ERROR";
        return ReturnValue::return_code::return_value_error_input_out_of_bounds;
    }
}

ReturnValue embObjMotionControl::getJointTypeRaw(int axis, yarp::dev::JointTypeEnum& type)
{
    if (axis >= 0 && axis < _njoints)
    {
        type = _axesInfo[axis].type;
        return ReturnValue_ok;
    }
    else
    {
        return ReturnValue_error_generic;
    }
}

bool embObjMotionControl::getJointDeadZoneRaw(int j, double &jntDeadZone)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config);
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    if(! askRemoteValue(protoid, &joint_cfg, size))
        return false;

    // refresh cached value when reading data from the EMS
    jntDeadZone = _measureConverter->posE2A((double)joint_cfg.deadzone, _axisMap[j]);
    
    return true;
}

// IRemoteVariables
ReturnValue embObjMotionControl::getRemoteVariableRaw(std::string key, yarp::os::Bottle& val)
{
    val.clear();
    if (key == "kinematic_mj")
    {
        // Return the reduced kinematic_mj matrix considering only the joints actually exposed to the user
        Bottle& ret = val.addList();

        eOmn_serv_type_t mc_serv_type = (eOmn_serv_type_t)serviceConfig.ethservice.configuration.type;
        if(iNeedCouplingsInfo())
            {
            for (int r=0; r<_njoints; r++)
            {
                for (int c = 0; c < _njoints; c++)
                {
                    // matrixJ2M is stored as row major in the  eomc_couplingInfo_t,
                    // and kinematic_mj is returned as a row major serialization as well
                    ret.addFloat64(_couplingInfo.matrixJ2M[4 * r + c]);
                }
            }
        }
        else
        {
            ret.addFloat64(0.0);
        }
        return ReturnValue_ok;
    }
    else if (key == "encoders")
    {
        Bottle& r = val.addList(); for (int i = 0; i < _njoints; i++) { r.addFloat64(_measureConverter->posA2E(1.0, i)); }
        return ReturnValue_ok;
    }
    else if (key == "rotorEncoderResolution")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorEncoderResolutionRaw(i, tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "jointEncoderResolution")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getJointEncoderResolutionRaw(i, tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "gearbox_M2J")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp=0; getGearboxRatioRaw(i, &tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "gearbox_E2J")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp=0; getGerabox_E2J(i, &tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "hasHallSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i < _njoints; i++) { int tmp = 0; getHasHallSensorRaw(i, tmp); r.addInt32(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "hasTempSensor")
    {
        Bottle& r = val.addList(); for (int i = 0; i < _njoints; i++) { int tmp = 0; getHasTempSensorsRaw(i, tmp); r.addInt32(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "TemperatureSensorType")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { std::string tmp = ""; getTemperatureSensorTypeRaw(i, tmp); r.addString(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "hasRotorEncoder")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { int tmp = 0; getHasRotorEncoderRaw(i, tmp); r.addInt32(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "hasRotorEncoderIndex")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { int tmp = 0; getHasRotorEncoderIndexRaw(i, tmp); r.addInt32(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "rotorIndexOffset")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorIndexOffsetRaw(i, tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "motorPoles")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { int tmp = 0; getMotorPolesRaw(i, tmp); r.addInt32(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "pidCurrentKp")
    {
        Bottle& r = val.addList(); for (int i = 0; i < _njoints; i++) { Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT, i, &p); r.addFloat64(p.kp); }
        return ReturnValue_ok;
    }
    else if (key == "pidCurrentKi")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT, i, &p); r.addFloat64(p.ki); }
        return ReturnValue_ok;
    }
    else if (key == "pidCurrentShift")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)  { Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT, i, &p); r.addFloat64(p.scale); }
        return ReturnValue_ok;
    }
    else if (key == "pidCurrentOutput")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)  { Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT, i, &p); r.addFloat64(p.max_output); }
        return ReturnValue_ok;
    }
    else if (key == "jointEncoderType")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)
        {
            int t; string s;
            getJointEncoderTypeRaw(i, t); uint8_t tt = t; bool b = EncoderType_eo2iCub(&tt, &s);
            if (b == false)
            {
                yError("Invalid jointEncoderType");
            }
            r.addString(s);
        }
        return ReturnValue_ok;
    }
    else if (key == "rotorEncoderType")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)
        {
            int t; string s;
            getRotorEncoderTypeRaw(i, t); uint8_t tt = t; bool b = EncoderType_eo2iCub(&tt, &s);
            if (b == false)
            {
                yError("Invalid motorEncoderType");
            }
            r.addString(s);
        }
        return ReturnValue_ok;
    }
    else if (key == "coulombThreshold")
    {
        val.addString("not implemented yet");
        return ReturnValue_ok;
    }
    else if (key == "torqueControlFilterType")
    {
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++)  { int t; getTorqueControlFilterType(i, t); r.addFloat64(t); }
        return ReturnValue_ok;
    }
    else if (key == "torqueControlEnabled")
    {

        Bottle& r = val.addList();
        for(int i = 0; i<_njoints; i++)
        {
            r.addInt32((int)_trq_pids[i].enabled );
        }
        return ReturnValue_ok;
    }
    else if (key == "PWMLimit")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getPWMLimitRaw(i, &tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "motOverloadCurr")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getMaxCurrentRaw(i, &tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "motNominalCurr")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getNominalCurrentRaw(i, &tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "motPeakCurr")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getPeakCurrentRaw(i, &tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "PowerSuppVoltage")
    {
        Bottle& r = val.addList(); for (int i = 0; i< _njoints; i++) { double tmp = 0; getPowerSupplyVoltageRaw(i, &tmp);  r.addFloat64(tmp); }
        return ReturnValue_ok;
    }
    else if (key == "rotorMax")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorLimitsRaw(i, &tmp1, &tmp2);  r.addFloat64(tmp2); }
        return ReturnValue_ok;
    }
    else if (key == "rotorMin")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getRotorLimitsRaw(i, &tmp1, &tmp2);  r.addFloat64(tmp1); }
        return ReturnValue_ok;
    }
    else if (key == "jointMax")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getPosLimitsRaw(i, &tmp1, &tmp2);  r.addFloat64(tmp2); }
        return ReturnValue_ok;
    }
    else if (key == "jointMin")
    {
        double tmp1, tmp2;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getPosLimitsRaw(i, &tmp1, &tmp2);  r.addFloat64(tmp1); }
        return ReturnValue_ok;
    }
    else if (key == "jointEncTolerance")
    {
        double tmp1;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getJointEncTolerance(i, &tmp1);  r.addFloat64(tmp1); }
        return ReturnValue_ok;
    }
    else if (key == "motorEncTolerance")
    {
        double tmp1;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getMotorEncTolerance(i, &tmp1);  r.addFloat64(tmp1); }
        return ReturnValue_ok;
    }
    else if (key == "jointDeadZone")
    {
        double tmp1;
        Bottle& r = val.addList(); for (int i = 0; i<_njoints; i++) { double tmp = 0; getJointDeadZoneRaw(i, tmp1);  r.addFloat64(tmp1); }
        return ReturnValue_ok;
    }
    else if (key == "readonly_position_PIDraw")
    {
        Bottle& r = val.addList();
        for (int i = 0; i < _njoints; i++)
        { Pid p;
          getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_POSITION, i, &p);
          char buff[1000];
          snprintf(buff, 1000, "J %d : kp %+3.3f ki %+3.3f kd %+3.3f maxint %+3.3f maxout %+3.3f off %+3.3f scale %+3.3f up %+3.3f dwn %+3.3f kff %+3.3f", i, p.kp, p.ki, p.kd, p.max_int, p.max_output, p.offset, p.scale, p.stiction_up_val, p.stiction_down_val, p.kff);
          r.addString(buff);
        }
        return ReturnValue_ok;
    }
    else if (key == "readonly_velocity_PIDraw")
    {
        Bottle& r = val.addList();
        for (int i = 0; i < _njoints; i++)
        { Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY, i, &p);
          char buff[1000];
          snprintf(buff, 1000, "J %d : kp %+3.3f ki %+3.3f kd %+3.3f maxint %+3.3f maxout %+3.3f off %+3.3f scale %+3.3f up %+3.3f dwn %+3.3f kff %+3.3f", i, p.kp, p.ki, p.kd, p.max_int, p.max_output, p.offset, p.scale, p.stiction_up_val, p.stiction_down_val, p.kff);
          r.addString(buff);
        }
        return ReturnValue_ok;
    }
    else if (key == "readonly_torque_PIDraw")
    {
        Bottle& r = val.addList();
        for (int i = 0; i < _njoints; i++)
        { Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE, i, &p);
         char buff[1000];
         snprintf(buff, 1000, "J %d : kp %+3.3f ki %+3.3f kd %+3.3f maxint %+3.3f maxout %+3.3f off %+3.3f scale %+3.3f up %+3.3f dwn %+3.3f kff %+3.3f", i, p.kp, p.ki, p.kd, p.max_int, p.max_output, p.offset, p.scale, p.stiction_up_val, p.stiction_down_val, p.kff);
         r.addString(buff);
        }
        return ReturnValue_ok;
    }
    else if (key == "readonly_current_PIDraw")
    {
        Bottle& r = val.addList();
        for (int i = 0; i < _njoints; i++)
        { Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT, i, &p);
         char buff[1000];
         snprintf(buff, 1000, "J %d : kp %+3.3f ki %+3.3f kd %+3.3f maxint %+3.3f maxout %+3.3f off %+3.3f scale %+3.3f up %+3.3f dwn %+3.3f kff %+3.3f", i, p.kp, p.ki, p.kd, p.max_int, p.max_output, p.offset, p.scale, p.stiction_up_val, p.stiction_down_val, p.kff);
         r.addString(buff);
        }
        return ReturnValue_ok;
    }
    else if (key == "readonly_llspeed_PIDraw")
    {
        Bottle& r = val.addList();
        for (int i = 0; i < _njoints; i++)
        {
            Pid p; getPidRaw(PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY, i, &p);
            char buff[1000];
            snprintf(buff, 1000, "J %d : kp %+3.3f ki %+3.3f kd %+3.3f maxint %+3.3f maxout %+3.3f off %+3.3f scale %+3.3f up %+3.3f dwn %+3.3f kff %+3.3f", i, p.kp, p.ki, p.kd, p.max_int, p.max_output, p.offset, p.scale, p.stiction_up_val, p.stiction_down_val, p.kff);
            r.addString(buff);
        }
        return ReturnValue_ok;
    }
    else if (key == "readonly_motor_torque_params_raw")
    {
        Bottle& r = val.addList();
        for (int i = 0; i < _njoints; i++)
        {
            MotorTorqueParameters params;
            getMotorTorqueParamsRaw(i, &params);
            char buff[1000];
            snprintf(buff, 1000, "J %d : bemf %+3.3f bemf_scale %+3.3f ktau %+3.3f ktau_scale %+3.3f viscousPos %+3.3f viscousNeg %+3.3f coulombPos %+3.3f coulombNeg %+3.3f velocityThres %+3.3f", i, params.bemf, params.bemf_scale, params.ktau, params.ktau_scale, params.viscousPos, params.viscousNeg, params.coulombPos, params.coulombNeg, params.velocityThres);
            r.addString(buff);
        }
        return ReturnValue_ok;
    }
    yWarning("getRemoteVariable(): Unknown variable %s", key.c_str());
    return ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::setRemoteVariableRaw(std::string key, const yarp::os::Bottle& val)
{
    string s1 = val.toString();
    if (val.size() != _njoints)
    {
        yWarning("setRemoteVariable(): Protocol error %s", s1.c_str());
        return ReturnValue_error_generic;
    }

    if (key == "kinematic_mj")
    {
        yWarning("setRemoteVariable(): Impossible to set kinematic_mj parameter at runtime.");
        return ReturnValue_error_generic;
    }
//     else if (key == "rotor")
//     {
//         for (int i = 0; i < _njoints; i++) _rotorEncoderRes[i] = val.get(i).asInt32();//this operation has none effect on motor controlelr, so i remove it
//         return true;
//     }
//     else if (key == "gearbox_M2J")
//     {
//         for (int i = 0; i < _njoints; i++) _gearbox_M2J[i] = val.get(i).asFloat64();//this operation has none effect on motor controlelr, so i remove it
//         return true;
//     }
    else if (key == "PWMLimit")
    {
        for (int i = 0; i < _njoints; i++) setPWMLimitRaw(i, val.get(i).asFloat64());
        return ReturnValue_ok;
    }
    //disabled for used safety
#if 0
    else if (key == "jointMax")
    {
        double min, max;
        for (int i = 0; i < _njoints; i++)
        {
            getPosLimitsRaw(i, &min, &max);
            setLimitsRaw(i, min, val.get(i).asFloat64());
        }
        return ReturnValue_ok;
    }
    else if (key == "jointMin")
    {
        double min, max;
        for (int i = 0; i < _njoints; i++)
        {
            getPosLimitsRaw(i, &min, &max);
            setLimitsRaw(i, val.get(i).asFloat64(), max);
        }
    }
#endif
    yWarning("setRemoteVariable(): Unknown variable %s", key.c_str());
    return ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getRemoteVariablesListRaw(yarp::os::Bottle* listOfKeys)
{
    listOfKeys->clear();
    listOfKeys->addString("kinematic_mj");
    listOfKeys->addString("encoders");
    listOfKeys->addString("gearbox_M2J");
    listOfKeys->addString("gearbox_E2J");
    listOfKeys->addString("hasHallSensor");
    listOfKeys->addString("hasTempSensor");
    listOfKeys->addString("TemperatureSensorType");
    listOfKeys->addString("hasRotorEncoder");
    listOfKeys->addString("hasRotorEncoderIndex");
    listOfKeys->addString("rotorIndexOffset");
    listOfKeys->addString("rotorEncoderResolution");
    listOfKeys->addString("jointEncoderResolution");
    listOfKeys->addString("motorPoles");
    listOfKeys->addString("pidCurrentKp");
    listOfKeys->addString("pidCurrentKi");
    listOfKeys->addString("pidCurrentShift");
    listOfKeys->addString("pidCurrentOutput");
    listOfKeys->addString("coulombThreshold");
    listOfKeys->addString("torqueControlFilterType");
    listOfKeys->addString("jointEncoderType");
    listOfKeys->addString("rotorEncoderType");
    listOfKeys->addString("PWMLimit");
    listOfKeys->addString("motOverloadCurr");
    listOfKeys->addString("motNominalCurr");
    listOfKeys->addString("motPeakCurr");
    listOfKeys->addString("PowerSuppVoltage");
    listOfKeys->addString("rotorMax");
    listOfKeys->addString("rotorMin");
    listOfKeys->addString("jointMax");
    listOfKeys->addString("jointMin");
    listOfKeys->addString("jointEncTolerance");
    listOfKeys->addString("motorEncTolerance");
    listOfKeys->addString("jointDeadZone");
    listOfKeys->addString("readonly_position_PIDraw");
    listOfKeys->addString("readonly_velocity_PIDraw");
    listOfKeys->addString("readonly_current_PIDraw");
    listOfKeys->addString("readonly_torque_PIDraw");
    listOfKeys->addString("readonly_motor_torque_params_raw");
    return ReturnValue_ok;
}

// IControlLimits2
ReturnValue embObjMotionControl::setVelLimitsRaw(int axis, double min, double max)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getVelLimitsRaw(int axis, double *min, double *max)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_config);
    uint16_t size;
    eOmc_joint_config_t    joint_cfg;
    if(! askRemoteValue(protoid, &joint_cfg, size))
        return ReturnValue_error_method_failed;

    *max = joint_cfg.maxvelocityofjoint;
    *min = 0;

    return ReturnValue_ok;
}


/*
 * IVirtualAnalogSensor Interface
 *
 *  YARP_METHOD_DEPRECATED!! WILL BE REMOVED IN THE NEAR FUTURE!!
 *
 */

yarp::dev::VAS_status embObjMotionControl::getVirtualAnalogSensorStatus(int ch)
{
    return VAS_status::VAS_OK;
};

int embObjMotionControl::getVirtualAnalogSensorChannels()
{
    return _njoints;
};

bool embObjMotionControl::updateVirtualAnalogSensorMeasure(yarp::sig::Vector &fTorques)
{
    bool ret = true;

    for(int j=0; j< _njoints; j++)
    {
        ret = ret && updateVirtualAnalogSensorMeasure(j, fTorques[j]);
    }
    return ret;
}

bool embObjMotionControl::updateVirtualAnalogSensorMeasure(int userLevel_jointNumber, double &fTorque)
{
    int j = _axisMap[userLevel_jointNumber];

    eOmeas_torque_t meas_torque = 0;
    static double curr_time = Time::now();
    static int count_saturation=0;

    meas_torque = (eOmeas_torque_t) S_32(_measureConverter->trqN2S(fTorque, j));

    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_inputs_externallymeasuredtorque);
//  We don't need anymore to cache locally because ems board broadcast its torque value in joint status core
//     // i write also locally because i want to read it back later on inside getTorqueRaw()
//     res->setLocalValue(protoid, &meas_torque);
    
    // and i want also to send it to the board
    return res->setRemoteValue(protoid, &meas_torque);
}

// end  IVirtualAnalogSensor //


// Torque control
ReturnValue embObjMotionControl::getTorqueRaw(int j, double *t)
{
    eOmc_joint_status_core_t jstatus;
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    bool ret = res->getLocalValue(protoid, &jstatus);
    *t = (double) _measureConverter->trqS2N(jstatus.measures.meas_torque, j);
    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::getTorquesRaw(double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getTorqueRaw(j, &t[j]);
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getTorqueRangeRaw(int j, double *min, double *max)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getTorqueRangesRaw(double *min, double *max)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::setRefTorquesRaw(const double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= setRefTorqueRaw(j, t[j]);
    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::setRefTorqueRaw(int j, double t)
{
    eOmc_setpoint_t setpoint;
    setpoint.type = (eOenum08_t) eomc_setpoint_torque;
    setpoint.to.torque.value =  (eOmeas_torque_t) S_32(t);

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    return res->setRemoteValue(protid, &setpoint) ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::setRefTorquesRaw(const int n_joint, const int *joints, const double *t)
{
    bool ret = true;
    for(int j=0; j< n_joint; j++)
    {
        ret &= setRefTorqueRaw(joints[j], t[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::getRefTorquesRaw(double *t)
{
    bool ret = true;
    for(int j=0; j<_njoints && ret; j++)
        ret &= getRefTorqueRaw(j, &t[j]);
    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::getRefTorqueRaw(int j, double *t)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t jcore = {0};
    *t =0 ;


    if(!res->getLocalValue(id32, &jcore))
    {
        yError() << "embObjMotionControl::getRefTorqueRaw() could not read pid torque reference pos for " << getBoardInfo() << "joint " << j;
        return ReturnValue_error_method_failed;
    }

    if ((eOmc_interactionmode_compliant == jcore.modes.interactionmodestatus) &&
        (eomc_controlmode_position == jcore.modes.controlmodestatus))
    {
        *t = (double) jcore.ofpid.complpos.reftrq;
    }

    if(eomc_controlmode_torque == jcore.modes.controlmodestatus)
    {
        *t = (double) jcore.ofpid.torque.reftrq;
    }

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_setTrqPidRaw(int j, const Pid &pid)
{
    eOmc_PID_t  outPid;
    Pid hwPid = pid;

    copyPid_iCub2eo(&hwPid, &outPid);
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtorque);
    return res->setRemoteValue(protid, &outPid) ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::helper_getTrqPidRaw(int j, Pid *pid)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_pidtorque);

    uint16_t size;
    eOmc_PID_t eoPID;
    if(! askRemoteValue(protoid, &eoPID, size))
        return ReturnValue_error_method_failed;
    
    copyPid_eo2iCub(&eoPID, pid);
    //printf("DEBUG getTorquePidRaw: %f %f %f %f %f\n",pid->kp , pid->ki, pid->kd , pid->stiction_up_val , pid->stiction_down_val );

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_getTrqPidsRaw(Pid *pid)
{
    std::vector<eOmc_PID_t> eoPIDList (_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_config_pidtorque, eoPIDList);
    if(! ret)
        return ReturnValue_error_method_failed;
    for(int j=0; j< _njoints; j++)
    {    
        copyPid_eo2iCub(&eoPIDList[j], &pid[j]);
        //printf("DEBUG getTorquePidRaw: %f %f %f %f %f\n",pid->kp , pid->ki, pid->kd , pid->stiction_up_val , pid->stiction_down_val );
    }
    return ReturnValue_ok;
}


ReturnValue embObjMotionControl::getImpedanceRaw(int j, double *stiffness, double *damping)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs
    eOmc_impedance_t val;

    if(!getWholeImpedanceRaw(j, val))
        return ReturnValue_error_method_failed;

    *stiffness = (double) (val.stiffness);
    *damping = (double) (val.damping);
    return ReturnValue_ok;
}

bool embObjMotionControl::getWholeImpedanceRaw(int j, eOmc_impedance_t &imped)
{
    // first set is done in the open function because the whole joint config is sent to the EMSs

    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);
    uint16_t size;
    if(! askRemoteValue(protoid, &imped, size))
        return false;

    // refresh cached value when reading data from the EMS
    _cacheImpedance->damping   =  imped.damping;
    _cacheImpedance->stiffness =  imped.stiffness;
    _cacheImpedance->offset    =  imped.offset;
    return true;
}

ReturnValue embObjMotionControl::setImpedanceRaw(int j, double stiffness, double damping)
{
    bool ret = true;
    eOmc_impedance_t val;

    // Need to read the whole struct and modify just 2 of them -> now aching the old values and re-using them.
    // first set is done in the open function because the whole joint config is sent to the EMSs
    // cleaner solution, split the impedance structure into 2 separeted nework variables
//    if(!getWholeImpedanceRaw(j, val))
//        return false;

    _cacheImpedance[j].stiffness = (eOmeas_stiffness_t) stiffness;
    _cacheImpedance[j].damping   = (eOmeas_damping_t) damping;

    val.stiffness   = _cacheImpedance[j].stiffness;
    val.damping     = _cacheImpedance[j].damping;
    val.offset      = _cacheImpedance[j].offset;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);


    ret &= res->setRemoteValue(protid, &val);
    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::setImpedanceOffsetRaw(int j, double offset)
{
    bool ret = true;
    eOmc_impedance_t val;

    // first set is done in the open function because the whole joint config is sent to the EMSs
//    if(!getWholeImpedanceRaw(j, val))
//        return false;

    _cacheImpedance[j].offset   = (eOmeas_torque_t) S_32(offset);
    val.stiffness     = _cacheImpedance[j].stiffness;
    val.damping     = _cacheImpedance[j].damping;
    val.offset      = _cacheImpedance[j].offset;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_impedance);


    ret &= res->setRemoteValue(protid, &val);

    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::getImpedanceOffsetRaw(int j, double *offset)
{
    eOmc_impedance_t val;

    if(!getWholeImpedanceRaw(j, val))
        return ReturnValue_error_generic;

    *offset = val.offset;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getCurrentImpedanceLimitRaw(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp)
{
    *min_stiff = _impedance_limits[j].min_stiff;
    *max_stiff = _impedance_limits[j].max_stiff;
    *min_damp  = _impedance_limits[j].min_damp;
    *max_damp  = _impedance_limits[j].max_damp;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getMotorTorqueParamsRaw(int j, MotorTorqueParameters *params)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_motor_params);

    uint16_t size;
    eOmc_motor_params_t eo_params = {0};
    if(! askRemoteValue(protoid, &eo_params, size))
        return ReturnValue_error_generic;

    params->bemf =       eo_params.bemf_value;
    params->bemf_scale = eo_params.bemf_scale;
    params->ktau       = eo_params.ktau_value;
    params->ktau_scale = eo_params.ktau_scale;
    params->viscousPos = eo_params.friction.viscous_pos_val;
    params->viscousNeg = eo_params.friction.viscous_neg_val ;
    params->coulombPos = eo_params.friction.coulomb_pos_val;
    params->coulombNeg = eo_params.friction.coulomb_neg_val;
    params->velocityThres  = eo_params.friction.velocityThres_val;

    //printf("debug getMotorTorqueParamsRaw %f %f %f %f %f %f %f %f\n",  params->bemf, params->bemf_scale, params->ktau,params->ktau_scale, params->viscousPos, params->viscousNeg, params->coulombPos, params->coulombNeg, params->threshold);

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setMotorTorqueParamsRaw(int j, const MotorTorqueParameters params)
{
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_motor_params);
    eOmc_motor_params_t eo_params = {0};

    //printf("setMotorTorqueParamsRaw for j %d(INPUT): benf=%f ktau=%f viscousPos=%f viscousNeg=%f coulombPos=%f coulombNeg=%f\n",j, params.bemf, params.ktau, params.viscousPos, params.viscousNeg, params.coulombPos, params.coulombNeg, params.threshold);

    eo_params.bemf_value  = (float)   params.bemf;
    eo_params.bemf_scale  = (uint8_t) params.bemf_scale;
    eo_params.ktau_value  = (float)   params.ktau;
    eo_params.ktau_scale  = (uint8_t) params.ktau_scale;
    eo_params.friction.viscous_pos_val = static_cast<float32_t>(params.viscousPos);
    eo_params.friction.viscous_neg_val = static_cast<float32_t>(params.viscousNeg);
    eo_params.friction.coulomb_pos_val = static_cast<float32_t>(params.coulombPos);
    eo_params.friction.coulomb_neg_val = static_cast<float32_t>(params.coulombNeg);
    eo_params.friction.velocityThres_val = static_cast<float32_t>(params.velocityThres);


    if(false == res->setRemoteValue(id32, &eo_params))
    {
        yError() << "embObjMotionControl::setMotorTorqueParamsRaw() could not send set message for" << getBoardInfo() << "joint " << j;
        return ReturnValue_error_generic;
    }

    return ReturnValue_ok;
}

// IVelocityControl2
ReturnValue embObjMotionControl::velocityMoveRaw(const int n_joint, const int *joints, const double *spds)
{
    ReturnValue ret = ReturnValue_ok;

    for(int j=0; j< n_joint; j++)
    {
        ret &= velocityMoveRaw(joints[j], spds[j]);
    }
    return ret;
}

/*
bool embObjMotionControl::helper_setVelPidRaw(int j, const Pid &pid)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_piddirect);
    eOmc_PID_t  outPid;
    Pid hwPid = pid;

    if(!_dir_pids[j].enabled)
    {
        yError() << "eoMc " << getBoardInfo() << ": it is not possible set direct pid for joint " << j <<", because velocity pid is enabled in xml files";
        return false;
    }

    copyPid_iCub2eo(&hwPid, &outPid);

    if (false == res->setRemoteValue(protoId, &outPid))
    {
        yError() << "while setting direct PIDs for" << getBoardInfo() << " joint " << j;
        return false;
    }

    return true;

    //return YARP_METHOD_NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}
*/

ReturnValue embObjMotionControl::helper_getVelPidRaw(int j, Pid *pid)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_config_piddirect);
    uint16_t size;
    eOmc_PID_t eoPID;
    if(! askRemoteValue(protoid, &eoPID, size))
        return ReturnValue_error_method_failed;

    copyPid_eo2iCub(&eoPID, pid);

    return ReturnValue_ok;

    //return YARP_METHOD_NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

ReturnValue embObjMotionControl::helper_getVelPidsRaw(Pid *pid)
{
    std::vector <eOmc_PID_t> eoPIDList (_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_config_piddirect, eoPIDList);
    if(!ret)
        return ReturnValue_error_method_failed;
    
    for(int j=0; j<_njoints; j++)
    {
        copyPid_eo2iCub(&eoPIDList[j], &pid[j]);
    }

    return ReturnValue_ok;

    //return YARP_METHOD_NOT_YET_IMPLEMENTED("Our boards do not have a Velocity Pid");
}

// PositionDirect Interface
ReturnValue embObjMotionControl::setPositionRaw(int j, double ref)
{
    int mode = 0;
    getControlModeRaw(j, &mode);
    if (mode != VOCAB_CM_POSITION_DIRECT &&
        mode != VOCAB_CM_IDLE)
    {
        if(event_downsampler->canprint())
        {
            yError() << "setReferenceRaw: skipping command because" << getBoardInfo() << " joint " << j << " is not in VOCAB_CM_POSITION_DIRECT mode";
        }
        return ReturnValue_ok;
    }

    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);
    eOmc_setpoint_t setpoint = {0};

    _ref_positions[j] = ref;   // save internally the new value of pos.
    setpoint.type = (eOenum08_t) eomc_setpoint_positionraw;
    setpoint.to.position.value = (eOmeas_position_t) S_32(ref);
    setpoint.to.position.withvelocity = 0;

    return res->setRemoteValue(protoId, &setpoint) ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::setPositionsRaw(const int n_joint, const int *joints, const double *refs)
{
    ReturnValue ret = ReturnValue_ok;
    for(int i=0; i<n_joint; i++)
    {
        ret &= setPositionRaw(joints[i], refs[i]);
    }
    return ret;
}

ReturnValue embObjMotionControl::setPositionsRaw(const double *refs)
{
    ReturnValue ret = ReturnValue_ok;
    for (int i = 0; i<_njoints; i++)
    {
        ret &= setPositionRaw(i, refs[i]);
    }
    return ret;
}


ReturnValue embObjMotionControl::getTargetPositionRaw(int axis, double *ref)
{
    if (axis<0 || axis>_njoints) return ReturnValue_error_generic;
#if ASK_REFERENCE_TO_FIRMWARE
   eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_target);
   *ref = 0;

    // Get the value
    uint16_t size;
    eOmc_joint_status_target_t  target = {0};
    if(!askRemoteValue(id32, &target, size))
    {
        yError() << "embObjMotionControl::getTargetPositionRaw() could not read reference pos for " << getBoardInfo() << "joint " << axis;
        return ReturnValue_error_generic;
    }

    *ref = (double) target.trgt_position;
    //yError() << "embObjMotionControl::getTargetPositionRaw()  BOARD" << _fId.boardNumber << "joint " << axis << "pos=" << target.trgt_position;
    return ReturnValue_ok;
#else
    *ref = _ref_command_positions[axis];
    return ReturnValue_ok;
#endif
}

ReturnValue embObjMotionControl::getTargetPositionsRaw(double *refs)
{
    bool ret = true;
    for (int i = 0; i<_njoints; i++)
    {
        ret &= getTargetPositionRaw(i, &refs[i]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getTargetPositionsRaw(int nj, const int * jnts, double *refs)
{
    bool ret = true;
    for (int i = 0; i<nj; i++)
    {
        ret &= getTargetPositionRaw(jnts[i], &refs[i]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}


ReturnValue  embObjMotionControl::getTargetVelocityRaw(int axis, double *ref)
{
    if (axis<0 || axis>_njoints) return ReturnValue_error_generic;
#if ASK_REFERENCE_TO_FIRMWARE
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_target);
    *ref = 0;

    // Get the value
    uint16_t size;
    eOmc_joint_status_target_t  target = {0};
    if(!askRemoteValue(id32, &target, size))
    {
        yError() << "embObjMotionControl::getTargetVelocityRaw() could not read reference vel for " << getBoardInfo() << "joint " << axis;
        return ReturnValue_error_generic;
    }
    *ref = (double) target.trgt_velocity;
    return ReturnValue_ok;
#else
    *ref = _ref_command_speeds[axis];
    return ReturnValue_ok;
#endif
}

ReturnValue  embObjMotionControl::getTargetVelocitiesRaw(double *refs)
{
    #if ASK_REFERENCE_TO_FIRMWARE
    std::vector <eOmc_joint_status_target_t> targetList(_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_status_target, targetList);
    if(!ret)
    {
        yError() << "embObjMotionControl::getTargetVelocitiesRaw() could not read reference vel for " << getBoardInfo() ;
        return ReturnValue_error_generic;
    }
    // Get the value
    for(int j=0; j<_njoints; j++)
    {
        refs[j] = (double) targetList[j].trgt_velocity;
    }
    return ReturnValue_ok;
    #else
    for(int j=0; j<_njoints; j++)
    {
        refs[j] = _ref_command_speeds[j];
    }
    return ReturnValue_ok;
    #endif
}

ReturnValue  embObjMotionControl::getTargetVelocitiesRaw(int nj, const int * jnts, double *refs)
{
    std::vector <double> refsList(_njoints);
    if(!getTargetVelocitiesRaw(refsList.data()))
        return ReturnValue_error_generic;
    
    for (int i = 0; i<nj; i++)
    {
        if(jnts[i]>= _njoints)
        {
            yError() << getBoardInfo() << "getTargetVelocitiesRaw: joint " << jnts[i] << "doesn't exist";
            return ReturnValue_error_generic;
        }
        refs[i] = refsList[jnts[i]];
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getRefPositionRaw(int axis, double *ref)
{
    if (axis<0 || axis>_njoints) return ReturnValue_error_generic;
#if ASK_REFERENCE_TO_FIRMWARE
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, axis, eoprot_tag_mc_joint_status_target);
    *ref = 0;
    // Get the value
    uint16_t size;
    eOmc_joint_status_target_t  target = {0};
    if(!askRemoteValue(id32, &target, size))
    {
        yError() << "embObjMotionControl::getRefPositionRaw() could not read reference pos for " << getBoardInfo() << "joint " << axis;
        return ReturnValue_error_generic;
    }

    *ref = (double) target.trgt_positionraw;
    return ReturnValue_ok;
#else
    *ref = _ref_positions[axis];
    return ReturnValue_ok;
#endif
}

ReturnValue embObjMotionControl::getRefPositionsRaw(double *refs)
{
    #if ASK_REFERENCE_TO_FIRMWARE
    std::vector <eOmc_joint_status_target_t> targetList(_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_status_target, targetList);
    if(!ret)
    {
        yError() << "embObjMotionControl::getRefPositionsRaw() could not read reference pos for " << getBoardInfo();
        return ReturnValue_error_generic;
    }
    // Get the value
    for(int j=0; j< _njoints; j++)
        refs[j] = (double) targetList[j].trgt_positionraw;
    return ReturnValue_ok;
    #else
    for(int j=0; j< _njoints; j++)
        refs[j] = _ref_positions[j];
    return ReturnValue_ok;
    #endif
}

ReturnValue embObjMotionControl::getRefPositionsRaw(int nj, const int * jnts, double *refs)
{
    ReturnValue ret = ReturnValue_ok;
    for (int i = 0; i<nj; i++)
    {
        ret &= getRefPositionRaw(jnts[i], &refs[i]);
    }
    return ret;
}

// InteractionMode



ReturnValue embObjMotionControl::getInteractionModeRaw(int j, yarp::dev::InteractionModeEnum* _mode)
{
    eOenum08_t interactionmodestatus;
//    std::cout << "eoMC getInteractionModeRaw SINGLE joint " << j << std::endl;

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core_modes_interactionmodestatus);
    if(! res->getLocalValue(protid, &interactionmodestatus)) // it is broadcasted toghether with the jointStatus full
        return ReturnValue_error_generic;

    int tmp = (int) *_mode;
    if(!interactionModeStatusConvert_embObj2yarp(interactionmodestatus, tmp) )
        return ReturnValue_error_generic;

    *_mode = (yarp::dev::InteractionModeEnum) tmp;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
//    std::cout << "eoMC getInteractionModeRaw GROUP joints" << std::endl;
    bool ret = true;
    for(int idx=0; idx<n_joints; idx++)
    {
        ret =  getInteractionModeRaw(joints[idx], &modes[idx]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{
//    std::cout << "eoMC getInteractionModeRaw ALL joints" << std::endl;
    bool ret = true;
    for(int j=0; j<_njoints; j++)
        ret = ret && getInteractionModeRaw(j, &modes[j]);
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

// marco.accame: con alberto cardellino abbiamo parlato della correttezza di effettuare la verifica di quanto imposto (in setInteractionModeRaw() ed affini)
// andando a rileggere il valore nella scheda eth fino a che esso non sia quello atteso. si deve fare oppure no?
// con il interaction mode il can ora non lo fa. mentre lo fa per il control mode. perche' diverso?
ReturnValue embObjMotionControl::setInteractionModeRaw(int j, yarp::dev::InteractionModeEnum _mode)
{
    eOenum08_t interactionmodecommand = 0;


    //    yDebug() << "received setInteractionModeRaw command (SINGLE) for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(_mode);

    if (_mode == VOCAB_IM_COMPLIANT && _trq_pids[j].enabled  == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; return ReturnValue_error_generic;}

    if(!interactionModeCommandConvert_yarp2embObj(_mode, interactionmodecommand) )
    {
        yError() << "setInteractionModeRaw: received unknown mode for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(_mode);
    }

    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);

    if(false == res->setRemoteValue(protid, &interactionmodecommand) )
    {
        yError() << "setInteractionModeRaw failed for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(_mode);
        return ReturnValue_error_generic;
    }

    // marco.accame: use the following if you want to check the value of interactionmode on the remote board
#if 0
    eOenum08_t interactionmodestatus = 0;
    uint16_t size = 0;
    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_interactionmodestatus);
    bool ret = askRemoteValue(id32, &interactionmodestatus, size);

    if((false == ret) || (interactionmodecommand != interactionmodestatus))
    {
    yError() << "check of embObjMotionControl::setInteractionModeRaw() failed for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(_mode);
        return ReturnValue_error_generic;
    }
#endif

    return ReturnValue_ok;
}


ReturnValue embObjMotionControl::setInteractionModesRaw(int n_joints, int *joints, yarp::dev::InteractionModeEnum* modes)
{
//    std::cout << "setInteractionModeRaw GROUP " << std::endl;

    eOenum08_t interactionmodecommand = 0;

    for(int j=0; j<n_joints; j++)
    {
        if (modes[j] == VOCAB_IM_COMPLIANT && _trq_pids[j].enabled  == false) {yError()<<"Torque control is disabled. Check your configuration parameters"; continue;}

        if(!interactionModeCommandConvert_yarp2embObj(modes[j], interactionmodecommand) )
        {
            yError() << "embObjMotionControl::setInteractionModesRaw(): received unknown interactionMode for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(modes[j]) << " " << modes[j];
            return ReturnValue_error_generic;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);
        if(false == res->setRemoteValue(protid, &interactionmodecommand) )
        {
            yError() << "embObjMotionControl::setInteractionModesRaw() failed for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(modes[j]);
            return ReturnValue_error_generic;
        }

        // marco.accame: use the following if you want to check the value of interactionmode on the remote board
#if 0
        eOenum08_t interactionmodestatus = 0;
        uint16_t size = 0;
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_interactionmodestatus);
        bool ret = askRemoteValue(id32, &interactionmodestatus, size);

        if((false == ret) || (interactionmodecommand != interactionmodestatus))
        {
            if(false == ret)
            {
            yError() << "check of embObjMotionControl::setInteractionModesRaw() failed for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(modes[j]);
                return ReturnValue_error_generic;
            }

            int tmp;
            if(interactionModeStatusConvert_embObj2yarp(interactionmodestatus, tmp) )
                yError() << "setInteractionModeRaw failed for" << getBoardInfo() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab32::decode(modes[j]) << " Got " << Vocab32::decode(tmp);
            else
                yError() << "setInteractionModeRaw failed for" << getBoardInfo() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab32::decode(modes[j]) << " Got an unknown value!";
            return ReturnValue_error_generic;
        }
#endif

    }

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setInteractionModesRaw(yarp::dev::InteractionModeEnum* modes)
{

    eOenum08_t interactionmodecommand = 0;

    for(int j=0; j<_njoints; j++)
    {
        if ((modes[j] == VOCAB_IM_COMPLIANT) && (_trq_pids[j].enabled  == false))
        {
            yError()<<"Torque control is disabled. Check your configuration parameters";
            continue;
        }

        if(!interactionModeCommandConvert_yarp2embObj(modes[j], interactionmodecommand) )
        {
            yError() << "setInteractionModeRaw: received unknown interactionMode for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(modes[j]);
            return ReturnValue_error_generic;
        }

        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_interactionmode);
        if(false == res->setRemoteValue(protid, &interactionmodecommand) )
        {
            yError() << "setInteractionModeRaw failed for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(modes[j]);
            return ReturnValue_error_generic;
        }

        // marco.accame: use the following if you want to check the value of interactionmode on the remote board
#if 0
        eOenum08_t interactionmodestatus = 0;
        uint16_t size = 0;
        eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_interactionmodestatus);
        bool ret = askRemoteValue(id32, &interactionmodestatus, size);

        if((false == ret) || (interactionmodecommand != interactionmodestatus))
        {
            if(false == ret)
            {
            yError() << "check of embObjMotionControl::setInteractionModesRaw() failed for" << getBoardInfo() << " joint " << j << " mode " << Vocab32::decode(modes[j]);
                return ReturnValue_error_generic;
            }

            int tmp;
            if(interactionModeStatusConvert_embObj2yarp(interactionmodestatus, tmp) )
                yError() << "setInteractionModeRaw failed for" << getBoardInfo() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab32::decode(modes[j]) << " Got " << Vocab32::decode(tmp);
            else
                yError() << "setInteractionModeRaw failed for" << getBoardInfo() << " joint " << j << " because of interactionMode mismatching \n\tSet " \
                         << Vocab32::decode(modes[j]) << " Got an unknown value!";
            return ReturnValue_error_generic;
        }
#endif

    }

    return ReturnValue_ok;
}


ReturnValue embObjMotionControl::getPidOutputRaw(const PidControlTypeEnum& pidtype, int j, double *out)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_core);
    eOmc_joint_status_core_t jcore = {0};
    *out = 0;
    if(!res->getLocalValue(protoId, &jcore) )
        return ReturnValue_error_method_failed;

    switch (pidtype)
    {
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION:
            if((eomc_controlmode_torque == jcore.modes.controlmodestatus)   ||  (eomc_controlmode_current == jcore.modes.controlmodestatus))
                *out=0;
            else
                *out = (double) jcore.ofpid.generic.output;
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_POSITION_DIRECT:
            *out=0;
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_TORQUE:
            if ((eomc_controlmode_torque == jcore.modes.controlmodestatus) ||
                ((eomc_controlmode_position == jcore.modes.controlmodestatus) && (eOmc_interactionmode_compliant == jcore.modes.interactionmodestatus)))
                *out = jcore.ofpid.generic.output;
            else
                *out = 0;
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_CURRENT:
            *out=0;
        break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY:
            *out = 0;
            break;
        case PidControlTypeEnum::VOCAB_PIDTYPE_VELOCITY_DIRECT:
            *out=0; //TODO
        break;
        default:
            yError()<<"Invalid pidtype:"<<static_cast<int>(pidtype);
        break;
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getPidOutputsRaw(const PidControlTypeEnum& pidtype, double *outs)
{
    ReturnValue ret = ReturnValue_ok;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getPidOutputRaw(pidtype, j, &outs[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::getNumberOfMotorsRaw(int* num)
{
    *num=_njoints;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getTemperatureRaw(int m, double* val)
{
    eOmc_motor_status_basic_t status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_status_basic);

    *val = NAN;
    if (_temperatureSensorsVector.at(m)->getType() == motor_temperature_sensor_none)
        return ReturnValue_ok;
    

    bool ret = res->getLocalValue(protid, &status);
    if(!ret)
    {
        yError() << getBoardInfo() << "At timestamp" << yarp::os::Time::now() << "In motor" << m << "embObjMotionControl::getTemperatureRaw failed to complete getLocalValue()";
        return ret ? ReturnValue_ok : ReturnValue_error_generic;
    }
    
    *val = _temperatureSensorsVector.at(m)->convertRawToTempCelsius((double)status.mot_temperature);
    
    
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getTemperaturesRaw(double *vals)
{
    bool ret = true;
    for(int j=0; j< _njoints; j++)
    {
        ret &= getTemperatureRaw(j, &vals[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getTemperatureLimitRaw(int m, double *temp)
{
    *temp= _temperatureLimits[m].warningTemperatureLimit;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setTemperatureLimitRaw(int m, const double temp)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_temperaturelimit);
    eOmeas_temperature_t  temperatureLimit = (eOmeas_pwm_t) S_16(temp);

    return res->setRemoteValue(protid, &temperatureLimit) ? ReturnValue_ok : ReturnValue_error_generic;

}

ReturnValue embObjMotionControl::getPeakCurrentRaw(int m, double *val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    *val = 0;
    if(!askRemoteValue(protid, &currentlimits, size))
    {
        yError() << "embObjMotionControl::getPeakCurrentRaw() can't read current limits  for" << getBoardInfo() << " motor " << m;
        return ReturnValue_error_generic;
    }

    *val = (double) currentlimits.peakCurrent ;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setPeakCurrentRaw(int m, const double val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);
    //get current limit params
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    if(!askRemoteValue(protid, &currentlimits, size))
    {
        yError() << "embObjMotionControl::setPeakCurrentRaw can't read current limits for" << getBoardInfo() << " motor " << m ;
        return ReturnValue_error_generic;
    }

    //set current overload
    currentlimits.peakCurrent = (eOmeas_current_t) S_16(val);

    //send new values
    bool ret = res->setRemoteValue(protid, &currentlimits);
    if(!ret)
    {
        yError() << "embObjMotionControl::setPeakCurrentRaw failed sending new value for" << getBoardInfo() << " motor " << m ;
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getNominalCurrentRaw(int m, double *val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    *val = 0;
    if(!askRemoteValue(protid, &currentlimits, size))
    {
        yError() << "embObjMotionControl::getNominalCurrentRaw() can't read current limits  for" << getBoardInfo() << " motor " << m;
        return ReturnValue_error_generic;
    }

    *val = (double) currentlimits.nominalCurrent ;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setNominalCurrentRaw(int m, const double val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, m, eoprot_tag_mc_motor_config_currentlimits);

    //get current limit params
    uint16_t size;
    eOmc_current_limits_params_t currentlimits = {0};
    if(!askRemoteValue(protid, &currentlimits, size))
    {
        yError() << "embObjMotionControl::setNominalCurrentRaw can't read current limits for" << getBoardInfo() << " motor " << m ;
        return ReturnValue_error_generic;
    }

    //set current overload
    currentlimits.nominalCurrent = (eOmeas_current_t) S_16(val);

    //send new values
    bool ret = res->setRemoteValue(protid, &currentlimits);
    if(!ret)
    {
        yError() << "embObjMotionControl::setNominalCurrentRaw failed sending new value for" << getBoardInfo() << " motor " << m ;
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getPWMRaw(int j, double* val)
{
    eOmc_motor_status_basic_t status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);

    bool ret = res->getLocalValue(protid, &status);
    if(ret)
    {
        *val = (double) status.mot_pwm;
    }
    else
    {
        yError() << "embObjMotionControl::getPWMRaw failed for" << getBoardInfo() << " motor " << j ;
        *val = 0;
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getPWMLimitRaw(int j, double* val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_pwmlimit);
    uint16_t size;
    eOmeas_pwm_t  motorPwmLimit;

    bool ret = askRemoteValue(protid, &motorPwmLimit, size);
    if(ret)
    {
        *val = (double) motorPwmLimit;
    }
    else
    {
        yError() << "embObjMotionControl::getPWMLimitRaw failed for" << getBoardInfo() << " motor " << j ;
        *val = 0;
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::setPWMLimitRaw(int j, const double val)
{
    if (val < 0)
    {
        yError() << "embObjMotionControl::setPWMLimitRaw failed because pwmLimit is negative for" << getBoardInfo() << " motor " << j ;
        return ReturnValue_ok; //return true because the error ios not due to communication error
    }
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_pwmlimit);
    eOmeas_pwm_t  motorPwmLimit = (eOmeas_pwm_t) S_16(val);

    return res->setRemoteValue(protid, &motorPwmLimit) ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getPowerSupplyVoltageRaw(int j, double* val)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_controller, 0, eoprot_tag_mc_controller_status);
    uint16_t size;
    eOmc_controller_status_t  controllerStatus;

    bool ret = askRemoteValue(protid, &controllerStatus, size);
    if(ret)
    {
        *val = (double) controllerStatus.supplyVoltage;
    }
    else
    {
        yError() << "embObjMotionControl::getPowerSupplyVoltageRaw failed for" << getBoardInfo() << " motor " << j ;
        *val = 0;
    }

    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

bool embObjMotionControl::askRemoteValue(eOprotID32_t id32, void* value, uint16_t& size)
{   
    return res->getRemoteValue(id32, value, 0.200, 0);
}


template <class T> 
bool embObjMotionControl::askRemoteValues(eOprotEndpoint_t ep, eOprotEntity_t entity, eOprotTag_t tag, std::vector<T>& values)
{
    std::vector<eOprotID32_t> idList;
    std::vector<void*> valueList;
    idList.clear();
    valueList.clear();
    for(int j=0; j<_njoints; j++)
    {
        eOprotID32_t protoId = eoprot_ID_get(ep, entity, j, tag);
        idList.push_back(protoId);
        valueList.push_back((void*)&values[j]);
    }
    
    bool ret = res->getRemoteValues(idList, valueList);
    if(!ret)
    {
        yError() << "embObjMotionControl::askRemoteValues failed for all joints of" << getBoardInfo();
    }
    
    return ret;
}




bool embObjMotionControl::checkRemoteControlModeStatus(int joint, int target_mode)
{
    bool ret = false;
    eOenum08_t temp = 0;
    uint16_t size = 0;

    eOprotID32_t id32 = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joint, eoprot_tag_mc_joint_status_core_modes_controlmodestatus);
    const double timeout = 0.250;  // 250 msec
    const int maxretries = 25;
    const double delaybetweenqueries = 0.010; // 10 msec

    // now i repeat the query until i am satisfied. how many times? for maximum time timeout seconds and with a gap of delaybetweenqueries

    double timeofstart = yarp::os::Time::now();
    int attempt = 0;

    for( attempt = 0; attempt < maxretries; attempt++)
    {
        ret = askRemoteValue(id32, &temp, size);
        if(ret == false)
        {
            yError ("An error occurred inside embObjMotionControl::checkRemoteControlModeStatus(j=%d, targetmode=%s) for BOARD %s IP %s", joint, yarp::os::Vocab32::decode(target_mode).c_str(), res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str());
            break;
        }
        int current_mode = controlModeStatusConvert_embObj2yarp(temp);
        if(current_mode == target_mode)
        {
            ret = true;
            break;
        }
        if((current_mode == VOCAB_CM_IDLE) && (target_mode == VOCAB_CM_FORCE_IDLE))
        {
            ret = true;
            break;
        }
        if(current_mode == VOCAB_CM_HW_FAULT)
        {
            if(target_mode != VOCAB_CM_FORCE_IDLE) { yError ("embObjMotionControl::checkRemoteControlModeStatus(%d, %d) is unable to check the control mode of BOARD %s IP %s because it is now in HW_FAULT", joint, target_mode, res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str()); }
            ret = true;
            break;
        }

        if((yarp::os::Time::now()-timeofstart) > timeout)
        {
            ret = false;
            yError ("A %f sec timeout occured in embObjMotionControl::checkRemoteControlModeStatus(), BOARD %s IP %s, joint %d, current mode: %s, requested: %s", timeout, res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str(), joint, yarp::os::Vocab32::decode(current_mode).c_str(), yarp::os::Vocab32::decode(target_mode).c_str());
            break;
        }
        if(attempt > 0)
        {   // i print the warning only after at least one retry.
            yWarning ("embObjMotionControl::checkRemoteControlModeStatus() has done %d attempts and will retry again after a %f sec delay. (BOARD %s IP %s, joint %d) -> current mode = %s, requested = %s", attempt+1, delaybetweenqueries, res->getProperties().boardnameString.c_str() , res->getProperties().ipv4addrString.c_str(), joint, yarp::os::Vocab32::decode(current_mode).c_str(), yarp::os::Vocab32::decode(target_mode).c_str());
        }
        SystemClock::delaySystem(delaybetweenqueries);
    }

    if(false == ret)
    {
        yError("failure of embObjMotionControl::checkRemoteControlModeStatus(j=%d, targetmode=%s) for BOARD %s IP %s after %d attempts and %f seconds", joint, yarp::os::Vocab32::decode(target_mode).c_str(), res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str(), attempt, yarp::os::Time::now()-timeofstart);
    }


    return ret;
}

//the device needs coupling info if it manages joints controlled by 2foc and mc4plus.
bool embObjMotionControl::iNeedCouplingsInfo(void)
{
    eOmn_serv_type_t mc_serv_type = (eOmn_serv_type_t)serviceConfig.ethservice.configuration.type;
    if( (mc_serv_type == eomn_serv_MC_foc) ||
        (mc_serv_type == eomn_serv_MC_mc4plus) ||
        (mc_serv_type == eomn_serv_MC_mc4plusmais) ||
        (mc_serv_type == eomn_serv_MC_mc2pluspsc) ||
        (mc_serv_type == eomn_serv_MC_mc4plusfaps) ||
        (mc_serv_type == eomn_serv_MC_advfoc)
      )
        return true;
    else
        return false;
}

//PWM interface
ReturnValue embObjMotionControl::setRefDutyCycleRaw(int j, double v)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    eOmc_setpoint_t setpoint;

    setpoint.type = (eOenum08_t)eomc_setpoint_openloop;
    setpoint.to.openloop.value = (eOmeas_pwm_t)S_16(v);

    return res->setRemoteValue(protid, &setpoint) ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::setRefDutyCyclesRaw(const double *v)
{
    bool ret = true;
    for (int j = 0; j<_njoints; j++)
    {
        ret = ret && setRefDutyCycleRaw(j, v[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getRefDutyCycleRaw(int j, double *v)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_target);
    uint16_t size = 0;
    *v = 0;
    eOmc_joint_status_target_t  target = { 0 };


    if (!askRemoteValue(protoId, &target, size))
    {
        yError() << "embObjMotionControl::getRefDutyCycleRaw() could not read openloop reference for " << getBoardInfo() << "joint " << j;
        return ReturnValue_error_generic;
    }

    *v = (double)target.trgt_pwm;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getRefDutyCyclesRaw(double *v)
{
    std::vector <eOmc_joint_status_target_t> targetList(_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_status_target, targetList);
    if(!ret)
    {
        yError() << "embObjMotionControl::getDutyCyclesRaw failed for all joints of" << getBoardInfo();
    }
    
    for (int j = 0; j<_njoints; j++)
    {
        v[j]= targetList[j].trgt_pwm;
    }
    return ret ? ReturnValue_ok : ReturnValue_error_generic;
}

ReturnValue embObjMotionControl::getDutyCycleRaw(int j, double *v)
{
    eOmc_motor_status_basic_t status;
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_status_basic);

    bool ret = res->getLocalValue(protid, &status);
    if (ret)
    {
        *v = (double)status.mot_pwm;
    }
    else
    {
        yError() << "embObjMotionControl::getDutyCycleRaw failed for" << getBoardInfo() << " motor " << j;
        *v = 0;
    }

    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::getDutyCyclesRaw(double *v)
{
    bool ret = true;
    for (int j = 0; j< _njoints; j++)
    {
        ret &= getDutyCycleRaw(j, &v[j]);
    }
    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;;
}

// Current interface

ReturnValue embObjMotionControl::getCurrentRangeRaw(int j, double *min, double *max)
{
    //this should be completed with numbers obtained from configuration files.
    //some caveats: currently current limits are expressed in robot configuration files in milliAmperes. Amperes should be used instead.
    //yarp does not perform any conversion on these numbers. Should it?
    *min = -10000.0;
    *max = 10000.0;
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getCurrentRangesRaw(double *min, double *max)
{
    ReturnValue ret = ReturnValue_ok;
    for (int j = 0; j< _njoints; j++)
    {
        ret &= getCurrentRangeRaw(j, &min[j], &max[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::setRefCurrentsRaw(const double *t)
{
    ReturnValue ret = ReturnValue_ok;
    for (int j = 0; j<_njoints; j++)
    {
        ret = ret && setRefCurrentRaw(j, t[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::setRefCurrentRaw(int j, double t)
{
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_cmmnds_setpoint);

    eOmc_setpoint_t setpoint;

    setpoint.type = (eOenum08_t)eomc_setpoint_current;
    setpoint.to.current.value = (eOmeas_pwm_t)S_16(t);

    return res->setRemoteValue(protid, &setpoint) ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::setRefCurrentsRaw(const int n_joint, const int *joints, const double *t)
{
    ReturnValue ret = ReturnValue_ok;
    for (int j = 0; j<n_joint; j++)
    {
        ret = ret && setRefCurrentRaw(joints[j], t[j]);
    }
    return ret;
}

ReturnValue embObjMotionControl::getRefCurrentsRaw(double *t)
{
    std::vector <eOmc_joint_status_target_t> targetList(_njoints);
    bool ret = askRemoteValues(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, eoprot_tag_mc_joint_status_target, targetList);
    if (!ret)
    {
        yError() << "embObjMotionControl::getDutyCyclesRaw failed for all joints of" << getBoardInfo();
    }

    for (int j = 0; j<_njoints; j++)
    {
        t[j] = targetList[j].trgt_current;
    }
    return ret ? ReturnValue_ok : ReturnValue_error_method_failed;
}

ReturnValue embObjMotionControl::getRefCurrentRaw(int j, double *t)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_target);
    uint16_t size = 0;
    *t = 0;
    eOmc_joint_status_target_t  target = { 0 };


    if (!askRemoteValue(protoId, &target, size))
    {
        yError() << "embObjMotionControl::getRefDutyCycleRaw() could not read openloop reference for " << getBoardInfo() << "joint " << j;
        return ReturnValue_error_method_failed;
    }

    *t = (double)target.trgt_current;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_setCurPidRaw(int j, const Pid &pid)
{
        eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_pidcurrent);
        eOmc_PID_t  outPid;
        Pid hwPid = pid;

        if (!_cur_pids[j].enabled)
        {
            yError() << "eoMc " << getBoardInfo() << ": it is not possible set current pid for motor " << j << ", because current pid is not enabled in xml files";
            return ReturnValue_error_generic;
        }

        copyPid_iCub2eo(&hwPid, &outPid);

        if (false == res->setRemoteValue(protoId, &outPid))
        {
            yError() << "while setting velocity PIDs for" << getBoardInfo() << " joint " << j;
            return ReturnValue_error_method_failed;
        }

        return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_setSpdPidRaw(int j, const Pid &pid)
{
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config_pidvelpwm);
    eOmc_PID_t  outPid;
    Pid hwPid = pid;

    if (!_cur_pids[j].enabled)
    {
        yError() << "eoMc " << getBoardInfo() << ": it is not possible set speed pid for motor " << j << ", because speed pid is not enabled in xml files";
        return ReturnValue_error_generic;
    }

    copyPid_iCub2eo(&hwPid, &outPid);

    if (false == res->setRemoteValue(protoId, &outPid))
    {
        yError() << "while setting velocity PIDs for" << getBoardInfo() << " joint " << j;
        return ReturnValue_error_method_failed;
    }

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_getCurPidRaw(int j, Pid *pid)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if(! askRemoteValue(protoid, &motor_cfg, size))
        return ReturnValue_error_method_failed;

    // refresh cached value when reading data from the EMS
    eOmc_PID_t tmp = (eOmc_PID_t)motor_cfg.pidcurrent;
    copyPid_eo2iCub(&tmp, pid);

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_getSpdPidRaw(int j, Pid *pid)
{
    eOprotID32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, j, eoprot_tag_mc_motor_config);
    uint16_t size;
    eOmc_motor_config_t    motor_cfg;
    if (!askRemoteValue(protoid, &motor_cfg, size))
        return ReturnValue_error_method_failed;

    // refresh cached value when reading data from the EMS
    eOmc_PID_t tmp = (eOmc_PID_t)motor_cfg.pidvelpwm;
    copyPid_eo2iCub(&tmp, pid);

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_getCurPidsRaw(Pid *pid)
{
    std::vector <eOmc_motor_config_t> motor_cfg_list(_njoints);
    bool ret = askRemoteValues<eOmc_motor_config_t>(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, eoprot_tag_mc_motor_config, motor_cfg_list);
    if(! ret)
        return ReturnValue_error_method_failed;
    
    for(int j=0; j<_njoints; j++)
    {
        eOmc_PID_t tmp = (eOmc_PID_t)motor_cfg_list[j].pidcurrent;
        copyPid_eo2iCub(&tmp, &pid[j]);
    }
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::helper_getSpdPidsRaw(Pid *pid)
{
    std::vector <eOmc_motor_config_t> motor_cfg_list(_njoints);
    bool ret = askRemoteValues<eOmc_motor_config_t>(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, eoprot_tag_mc_motor_config, motor_cfg_list);
    if (!ret)
        return ReturnValue_error_method_failed;

    for (int j = 0; j<_njoints; j++)
    {
        eOmc_PID_t tmp = (eOmc_PID_t)motor_cfg_list[j].pidvelpwm;
        copyPid_eo2iCub(&tmp, &pid[j]);
    }
    return ReturnValue_ok;
}

bool embObjMotionControl::getJointConfiguration(int joint, eOmc_joint_config_t *jntCfg_ptr)
{
    uint32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, joint, eoprot_tag_mc_joint_config);
    uint16_t size;
    if(!askRemoteValue(protoid, jntCfg_ptr, size))
    {
        yError ("Failure of askRemoteValue() inside embObjMotionControl::getJointConfiguration(axis=%d) for BOARD %s IP %s", joint, res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str());
        return false;
    }
    return true;
}

bool embObjMotionControl::getMotorConfiguration(int axis, eOmc_motor_config_t *motCfg_ptr)
{
    uint32_t protoid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_motor, axis, eoprot_tag_mc_motor_config);
    uint16_t size;
    if(!askRemoteValue(protoid, motCfg_ptr, size))
    {
        yError ("Failure of askRemoteValue() inside embObjMotionControl::getMotorConfiguration(axis=%d) for BOARD %s IP %s", axis, res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str());
        return false;
    }
    return true;
}


bool embObjMotionControl::getGerabox_E2J(int joint, double *gearbox_E2J_ptr)
{
    eOmc_joint_config_t jntCfg;

    if(!getJointConfiguration(joint, &jntCfg))
    {
        yError ("Failure embObjMotionControl::getGerabox_E2J(axis=%d) for BOARD %s IP %s", joint, res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str());
        return false;
    }
    *gearbox_E2J_ptr = jntCfg.gearbox_E2J;
    return true;
}

bool embObjMotionControl::getJointEncTolerance(int joint, double *jEncTolerance_ptr)
{
    eOmc_joint_config_t jntCfg;

    if(!getJointConfiguration(joint, &jntCfg))
    {
        yError ("Failure embObjMotionControl::getJointEncTolerance(axis=%d) for BOARD %s IP %s", joint, res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str());
        return false;
    }
    *jEncTolerance_ptr = jntCfg.jntEncTolerance;
    return true;
}

bool embObjMotionControl::getMotorEncTolerance(int axis, double *mEncTolerance_ptr)
{
    eOmc_motor_config_t motorCfg;
    if(!getMotorConfiguration(axis, &motorCfg))
    {
        yError ("Failure embObjMotionControl::getMotorEncTolerance(axis=%d) for BOARD %s IP %s", axis, res->getProperties().boardnameString.c_str(), res->getProperties().ipv4addrString.c_str());
        return false;
    }
    *mEncTolerance_ptr = motorCfg.rotEncTolerance;
    return true;
}

ReturnValue embObjMotionControl::getLastJointFaultRaw(int j, int& fault, std::string& message)
{
    eOmc_motor_status_t status;
    
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, 
                                        eoprot_entity_mc_motor, j, 
                                        eoprot_tag_mc_motor_status);
    
    bool ret = res->getLocalValue(protid, &status);

    message.clear();

    if (!ret)
    {
        fault = -1;
        message = "Could not retrieve the fault state.";
        return ReturnValue_error_method_failed;
    }

    if (status.mc_fault_state == EOERROR_CODE_DUMMY)
    {
        fault = EOERROR_CODE_DUMMY;
        message = "No fault detected.";

        return ReturnValue_ok;
    }

    fault = eoerror_code2value(status.mc_fault_state);
    message = eoerror_code2string(status.mc_fault_state);

    return ReturnValue_ok;
}

bool embObjMotionControl::getRawData_core(std::string key, std::vector<std::int32_t> &data)
{
    // Here I need to be sure 100% the key exists!!! 
    // It must exists since the call is made while iterating over the map
    data.clear();
    for(int j=0; j< _njoints; j++)
    {
        eOmc_joint_status_additionalInfo_t addinfo;
        eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, j, eoprot_tag_mc_joint_status_addinfo_multienc);
        if(!res->getLocalValue(protid, &addinfo))
        {
            return false;
        }
        for (int k = 0; k < std::size(addinfo.multienc); k++)
        {
            data.push_back((int32_t)addinfo.multienc[k]);
        }
        
    }
    return true;
}

bool embObjMotionControl::getRawDataMap(std::map<std::string, std::vector<std::int32_t>> &map)
{
    for (auto it = _rawValuesMetadataMap.begin(); it != _rawValuesMetadataMap.end(); it++)
    {
        if(!getRawData_core(it->first, _rawDataAuxVector))
        {
            yError() << getBoardInfo() << "getRawData failed. Cannot retrieve all raw data from local memory";
            return false;
        }
        map.insert({it->first, _rawDataAuxVector});
    }
    
    return true;
}

bool embObjMotionControl::getRawData(std::string key, std::vector<std::int32_t> &data)
{  
    if(_rawValuesMetadataMap.find(key) != _rawValuesMetadataMap.end())
    {
        getRawData_core(key, data);
    }
    else
    {
        yError() << getBoardInfo() << "Request key:" << key << "is not available. Cannot retrieve get raw data.";
        return false;
    }

    return true;
}

bool embObjMotionControl::getKeys(std::vector<std::string> &keys)
{
    for (const auto &p : _rawValuesMetadataMap)
    {
        keys.push_back(p.first);
    }
    
    return true;
}

int  embObjMotionControl::getNumberOfKeys()
{
    return _rawValuesMetadataMap.size();
}

bool embObjMotionControl::getMetadataMap(rawValuesKeyMetadataMap &metamap)
{
    if (_rawValuesMetadataMap.empty())
    {
        yError() << getBoardInfo() << "embObjMotionControl Map is empty. Closing...";
        return false;
    }
    
    metamap.metadataMap = _rawValuesMetadataMap;
    return true;
}
bool embObjMotionControl::getKeyMetadata(std::string key, rawValuesKeyMetadata &meta)
{
    if(_rawValuesMetadataMap.find(key) != _rawValuesMetadataMap.end())
    {
        meta = _rawValuesMetadataMap[key];
    }
    else
    {
        yError() << getBoardInfo() << "Requested key" << key << "is not available in the map. Closing...";
        return false;
    }
    

    return true;
}

bool embObjMotionControl::getAxesNames(std::string key, std::vector<std::string> &axesNames)
{
    axesNames.clear();
    if (_rawValuesMetadataMap.find(key) != _rawValuesMetadataMap.end())
    {
        axesNames.assign(_rawValuesMetadataMap[key].axesNames.begin(), _rawValuesMetadataMap[key].axesNames.end());
    }
    else
    {
        yError() << getBoardInfo() << "Requested key" << key << "is not available in the map. Exiting";
        return false;
    }
    return true;
}

ReturnValue embObjMotionControl::getAxes(size_t& axes)
{
        axes=_njoints;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setRefVelocityRaw(int jnt, double vel)
{
    int mode=0;
    getControlModeRaw(jnt, &mode);
    if( (mode != VOCAB_CM_VELOCITY_DIRECT) &&
        (mode != VOCAB_CM_IDLE)) //TODO: remove VOCAB_CM_IDLE exception when eoMc boards will properly support switching to VOCAB_CM_VELOCITY_DIRECT from other control modes different from VOCAB_CM_IDLE
    {
        if(event_downsampler->canprint())
        {
            yError() << "setRefVelocityRaw: skipping command because " << getBoardInfo() << " joint " << jnt << " is not in VOCAB_CM_VELOCITY_DIRECT mode";
        }
        return ReturnValue_ok;
    }
    
    eOmc_setpoint_t setpoint;
    setpoint.type = eomc_setpoint_velocityraw;
      
    setpoint.to.velocityraw.value =  (eOmeas_velocity_t) S_32(vel);
    
    eOprotID32_t protid = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, jnt, eoprot_tag_mc_joint_cmmnds_setpoint);
    
    if(false == res->setRemoteValue(protid, &setpoint))
    {
        yError() << getBoardInfo() << "while setting velocity direct target for"  << "joint " << jnt;
        return ReturnValue_error_method_failed;
    }
    
    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::setRefVelocityRaw(const std::vector<double>& vels)
{
    // Velocity is expressed in iDegrees/s
    ReturnValue ret = ReturnValue_ok;
    for(int j=0; j< _njoints; j++)
    {
        ret &= setRefVelocityRaw(j, vels[j]);

        if (!ret)
            return ret;
    }
    return ret;
}

ReturnValue embObjMotionControl::setRefVelocityRaw(const std::vector<int>& jnts, const std::vector<double>& vels)
{
    // Velocity is expressed in iDegrees/s
    ReturnValue ret = ReturnValue_ok;

    if (jnts.size() != vels.size())
    {
         yError() << getBoardInfo() << "while setting velocity direct target: size of joints and velocities vectors do not match";
        return ReturnValue_error_method_failed;
    }

    for (int j=0; j < jnts.size(); j++)
    {
        ret &= setRefVelocityRaw(jnts[j], vels[j]);
        if (!ret)
            return ret;
    }
    return ret;
}

ReturnValue embObjMotionControl::getRefVelocityRaw(const int jnt, double& vel)
{
    // Using joint related datatypes, instead of motor related datatypes
    eOprotID32_t protoId = eoprot_ID_get(eoprot_endpoint_motioncontrol, eoprot_entity_mc_joint, jnt,  eoprot_tag_mc_joint_status_target);
    
    uint16_t size;
    
    eOmc_joint_status_target_t target;

    if (!askRemoteValue(protoId, &target, size))
    {
        yError() << "embObjMotionControl::getRefVelocityRaw() could not read velocity direct reference for " << getBoardInfo() << "joint " << jnt;
        return ReturnValue_error_method_failed;
    }

    vel = (double)target.trgt_velocityraw;

    return ReturnValue_ok;
}

ReturnValue embObjMotionControl::getRefVelocityRaw(std::vector<double>& vels)
{
    ReturnValue ret = ReturnValue_ok;

    for (int j = 0; j< _njoints; j++)
    {
        ret &= getRefVelocityRaw(j, vels[j]);
        if (!ret)
        return ret;

    }
    return ret;
}

ReturnValue embObjMotionControl::getRefVelocityRaw(const std::vector<int>& jnts, std::vector<double>& vels)
{
    ReturnValue ret = ReturnValue_ok;

    for (int j = 0; j< jnts.size(); j++)
    {
        ret &= getRefVelocityRaw(jnts[j], vels[j]);
        if (!ret)
            return ret;
    }
    return ret;
}

ReturnValue embObjMotionControl::setPidFeedforwardRaw(const PidControlTypeEnum& pidtype,int j, double v)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getPidOffsetRaw(const PidControlTypeEnum& pidtype,int j, double& v)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getPidFeedforwardRaw(const PidControlTypeEnum& pidtype,int j, double& v)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::isPidEnabledRaw(const PidControlTypeEnum& pidtype, int j, bool& enabled)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getPidExtraInfoRaw(const PidControlTypeEnum& pidtype, int j, yarp::dev::PidExtraInfo& units)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}

ReturnValue embObjMotionControl::getPidExtraInfosRaw(const PidControlTypeEnum& pidtype, std::vector<yarp::dev::PidExtraInfo>& units)
{
    return YARP_METHOD_NOT_YET_IMPLEMENTED();
}
// eof
