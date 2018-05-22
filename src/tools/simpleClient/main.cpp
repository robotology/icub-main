// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2006 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Giorgio Metta and Lorenzo Natale
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

/**
 * @ingroup icub_tools
 *
 * \defgroup icub_simple_client simpleClient
 *
 * A basic remote interface to a controller.
 * Call with no arguments for usage information.
 *
 * \author Giorgio Metta and Lorenzo Natale
 *
 */

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Vocab.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::os;

#define VOCAB_HELP VOCAB4('h','e','l','p')
#define VOCAB_QUIT VOCAB4('q','u','i','t')
#define VOCAB_ICONTROLMODE_DEBUG VOCAB4('i','c','d','d')

void handleTorqueMsg(ITorqueControl *itq, IPidControl *ipid, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);
void handleImpedanceMsg(IImpedanceControl *iimp, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);
void handleControlModeMsg(IControlMode *icm, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);
void handleControlModeMsg_DEBUG(IControlMode *icm, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);
void handleInteractionModeMsg(IInteractionMode *_iInteract, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);

//
int jnts = 0;  // joint number

int main(int argc, char *argv[]) 
{
    // just list the devices if no argument given
    if (argc <= 2) {
        printf("You can call %s like this:\n", argv[0]);
        printf("   %s --robot ROBOTNAME --OPTION VALUE ...\n", argv[0]);
        printf("For example:\n");
        printf("   %s --robot icub --local /talkto/james --remote /controlboard/rpc\n", argv[0]);
        printf("Here are devices listed for your system:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    // get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("robot") || !options.check("part")) {
        printf("Missing either --robot or --part options\n");
        return 1;
    }

    Network::init();
    Time::turboBoost();
    
    std::string name;
    Value& v = options.find("robot");
    Value& part = options.find("part");

    Value *val;
    if (!options.check("device", val)) {
        options.put("device", "remote_controlboard");
    }
    if (!options.check("local", val)) {
        name="/"+std::string(v.asString().c_str())+"/"+std::string(part.asString().c_str())+"/simpleclient";
        //sprintf(&name[0], "/%s/%s/client", v.asString().c_str(), part.asString().c_str());
        options.put("local", name.c_str());
    }
    if (!options.check("remote", val)) {
        name="/"+std::string(v.asString().c_str())+"/"+std::string(part.asString().c_str());    
        //sprintf(&name[0], "/%s/%s", v.asString().c_str(), part.asString().c_str());
        options.put("remote", name.c_str());
    }

    fprintf(stderr, "%s", options.toString().c_str());

    
    // create a device 
    PolyDriver dd(options);
    if (!dd.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 1;
    }

    IPositionControl *ipos=0;
    IPositionDirect  *iposDir=0;
    IVelocityControl *vel=0;
    IEncoders *enc=0;
    IPidControl *pid=0;
    IAmplifierControl *amp=0;
    IControlLimits *lim=0;
    IControlMode *iMode2=0;
    IMotor *imot=0;
    ITorqueControl *itorque=0;
    IPWMControl *ipwm=0;
    IImpedanceControl *iimp=0;
    IInteractionMode *iInteract=0;
    IMotorEncoders *iMotEnc=0;
    IAxisInfo *iInfo = 0;

    bool ok;
    ok = dd.view(ipos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);
//    ok &= dd.view(icm);
    ok &= dd.view(itorque);
    ok &= dd.view(ipwm);
    ok &= dd.view(iimp);
    ok &= dd.view(iposDir);
    ok &= dd.view(iMode2);
    ok &= dd.view(iInteract);

    if (!ok) {
        yError("Problems acquiring mandatory interfaces, quitting\n");
        return 1;
    }

    ok &=dd.view(iMotEnc); 
    if (!ok) {
        yWarning("Problems acquiring optional interface IMotorEncoders\n");
    }
    ok &=dd.view(imot); 
    if (!ok) {
        yWarning("Problems acquiring optional interface iMotor\n");
    }
    ok &= dd.view(iInfo);
    if (!ok) {
        yWarning("Problems acquiring optional interface iInfo\n");
    }

    ipos->getAxes(&jnts);
    printf("Working with %d axes\n", jnts);
    double *tmp = new double[jnts];
    bool   *btmp = new bool[jnts];
    int    *jtmp = new int[jnts];
    for (int i=0; i<jnts; i++) jtmp[i]=i;

    printf("Device active...\n");
    while (dd.isValid()) {
        std::string s;
        s.resize(1024);
        
        printf("-> ");
        char c = 0;
        int i = 0;
        while (c != '\n') {
            c = (char)fgetc(stdin);
            s[i++] = c;
        }
        s[i-1] = s[i] = 0;

        Bottle p;
        Bottle response;
        bool ok=false;
        bool rec=false;
        p.fromString(s.c_str());
        printf("Bottle: %s\n", p.toString().c_str());

        switch(p.get(0).asVocab()) {      
        case VOCAB_HELP:
            printf("\n\n");
            printf("Available commands:\n");
            printf("-------------------\n\n");

            printf("IPWMControl:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_PWMCONTROL_INTERFACE).c_str());
            printf("    [set] [%s] <int> <float>\n", Vocab::decode(VOCAB_PWMCONTROL_REF_PWM).c_str());
            printf("    [get] [%s] <int>\n", Vocab::decode(VOCAB_PWMCONTROL_REF_PWM).c_str());
            printf("    [get] [%s]\n", Vocab::decode(VOCAB_PWMCONTROL_PWM_OUTPUT).c_str());
            printf("\n");

            printf("IControlMode:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_ICONTROLMODE).c_str());
            printf("    [set] [%s]|[%s]|[%s]|[%s]|[%s]|[%s]|[%s]|[%s]|[%s][%s]|[%s]\n",
                    Vocab::decode(VOCAB_CM_POSITION).c_str(),
                    Vocab::decode(VOCAB_CM_POSITION_DIRECT).c_str(),
                    Vocab::decode(VOCAB_CM_VELOCITY).c_str(),
                    Vocab::decode(VOCAB_CM_MIXED).c_str(),
                    Vocab::decode(VOCAB_CM_TORQUE).c_str(),
                    Vocab::decode(VOCAB_CM_PWM).c_str(),
                    Vocab::decode(VOCAB_CM_CURRENT).c_str(),
                    Vocab::decode(VOCAB_CM_IDLE).c_str(),
                    Vocab::decode(VOCAB_CM_FORCE_IDLE).c_str(),
                    Vocab::decode(VOCAB_CM_IMPEDANCE_POS).c_str(),
                    Vocab::decode(VOCAB_CM_IMPEDANCE_VEL).c_str());
            printf("    [get] [%s] <int>\n", Vocab::decode(VOCAB_CM_CONTROL_MODE).c_str());
            printf("\n");

            printf("ITorqueControl:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_TORQUE).c_str());
            printf("    [get] [%s] <int> to read the measured torque for a single axis\n",                  Vocab::decode(VOCAB_TRQ).c_str());
            printf("    [get] [%s]  to read the measured torque for all axes\n",                      Vocab::decode(VOCAB_TRQS).c_str());
            printf("    [set] [%s] <int> <float> to set the reference torque for a single axis\n",          Vocab::decode(VOCAB_REF).c_str());
            printf("    [set] [%s] <float list> to set the reference torque for all axes\n",        Vocab::decode(VOCAB_REFS).c_str());
            printf("    [set] [%s] int '('<int list>')' '('<float list>')' to set the reference torque for a subset of axes axes\n",   Vocab::decode(VOCAB_REFG).c_str());
            printf("    [get] [%s] <int> to read the reference torque for a single axis\n",                  Vocab::decode(VOCAB_REF).c_str());
            printf("    [get] [%s] to read the reference torque for all axes\n",                      Vocab::decode(VOCAB_REFS).c_str());
            printf("\n");

            printf("IImpedanceControl:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_IMPEDANCE).c_str());
            printf("    [set] [%s] <int> <float> <float> \n", Vocab::decode(VOCAB_IMP_PARAM).c_str());
            printf("    [set] [%s] <int> <float>\n\n",  Vocab::decode(VOCAB_IMP_OFFSET).c_str());
            printf("    [get] [%s] <int>\n", Vocab::decode(VOCAB_IMP_PARAM).c_str());
            printf("    [get] [%s] <int>\n", Vocab::decode(VOCAB_IMP_OFFSET).c_str());
            printf("\n");

            printf("IInteractionMode:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_INTERFACE_INTERACTION_MODE).c_str());
            printf("    [set] [%s]|[%s] <int>\n", Vocab::decode(VOCAB_IM_STIFF).c_str(), Vocab::decode(VOCAB_IM_COMPLIANT).c_str());
            printf("    [get] [%s] <int>\n", Vocab::decode(VOCAB_INTERACTION_MODE).c_str());
            printf("    [get] [%s] \n", Vocab::decode(VOCAB_INTERACTION_MODES).c_str());
            printf("\n");

            printf("IMotor Interfaces:\n");
            printf("type [get] and one of the following:\n");
            printf("    [%s] to read get the number of motors\n", Vocab::decode(VOCAB_MOTORS_NUMBER).c_str());
            printf("    [%s] <int> to read the temperature value for a single motor\n", Vocab::decode(VOCAB_TEMPERATURE).c_str());
            printf("    [%s] <int> to read the temperatures of all motors\n", Vocab::decode(VOCAB_TEMPERATURES).c_str());
            printf("    [%s] <int> to read the temperature limit for a single motor\n", Vocab::decode(VOCAB_TEMPERATURE_LIMIT).c_str());
            printf("type [set] and one of the following:\n");
            printf("    [%s] <int> to set the temperature limit for a single motor\n", Vocab::decode(VOCAB_TEMPERATURE_LIMIT).c_str());
            printf("\n");

            printf("IMotorEncoder Interfaces:\n");
            printf("type [get] and one of the following:\n");
            printf("    [%s] <int> to read the cpr value for a single motor\n", Vocab::decode(VOCAB_MOTOR_CPR).c_str());
            printf("    [%s] to read get the number of motor encoders\n", Vocab::decode(VOCAB_MOTOR_ENCODER_NUMBER).c_str());
            printf("    [%s] to read the motor encoder positions for all motors\n", Vocab::decode(VOCAB_MOTOR_ENCODERS).c_str());
            printf("    [%s] to read the motor encoder speeds for all motors\n", Vocab::decode(VOCAB_MOTOR_ENCODER_SPEEDS).c_str());
            printf("    [%s] to read the motor encoder accelerations for all motors\n", Vocab::decode(VOCAB_MOTOR_ENCODER_ACCELERATIONS).c_str());
            printf("\n");

            printf("Standard Interfaces:\n");
            printf("type [get] and one of the following:\n");
            printf("    [%s] to read the number of controlled axes\n", Vocab::decode(VOCAB_AXES).c_str());
            printf("    [%s] <int> to read the name of a single axis\n", Vocab::decode(VOCAB_INFO_NAME).c_str());
            printf("    [%s] <int> to read target position for a single axis (iPositionControl)\n", Vocab::decode(VOCAB_POSITION_MOVE).c_str());
            printf("    [%s] to read target positions for all axes (iPositionControl)\n", Vocab::decode(VOCAB_POSITION_MOVES).c_str());
            printf("    [%s] <int> to read reference position for single axis (iPositionDirect)\n", Vocab::decode(VOCAB_POSITION_DIRECT).c_str());
            printf("    [%s] to read reference position for all axes (iPositionDirect)\n", Vocab::decode(VOCAB_POSITION_DIRECTS).c_str());
            printf("    [%s] <int> to read reference velocity for single axis (for velocityMove)\n", Vocab::decode(VOCAB_VELOCITY_MOVE).c_str());
            printf("    [%s] to read reference velocities for all axes (for velocityMove)\n", Vocab::decode(VOCAB_VELOCITY_MOVES).c_str());
            printf("    [%s] to read the encoder value for all axes\n", Vocab::decode(VOCAB_ENCODERS).c_str());
            printf("    [%s] to read the PID values for all axes\n", Vocab::decode(VOCAB_PIDS).c_str());
            printf("    [%s] <int> to read the PID values for a single axis\n", Vocab::decode(VOCAB_PID).c_str());
            printf("    [%s] <int> to read the position limits for a single axis\n", Vocab::decode(VOCAB_LIMITS).c_str());
            printf("    [%s] <int> to read the velocity limits for a single axis\n", Vocab::decode(VOCAB_VEL_LIMITS).c_str());
            printf("    [%s] to read the PID error for all axes\n", Vocab::decode(VOCAB_ERRS).c_str());
            printf("    [%s] to read the PID output for all axes\n", Vocab::decode(VOCAB_OUTPUTS).c_str());
            printf("    [%s] to read the reference position for all axes\n", Vocab::decode(VOCAB_REFERENCES).c_str());
            printf("    [%s] <int> to read the reference position for a single axis\n", Vocab::decode(VOCAB_REFERENCE).c_str());
            printf("    [%s] to read the reference speed for all axes\n", Vocab::decode(VOCAB_REF_SPEEDS).c_str());
            printf("    [%s] <int> to read the reference speed for a single axis\n", Vocab::decode(VOCAB_REF_SPEED).c_str());
            printf("    [%s] to read the reference acceleration for all axes\n", Vocab::decode(VOCAB_REF_ACCELERATIONS).c_str());
            printf("    [%s] <int> to read the reference acceleration for a single axis\n", Vocab::decode(VOCAB_REF_ACCELERATION).c_str());
            printf("    [%s] to read the current consumption for all axes\n", Vocab::decode(VOCAB_AMP_CURRENTS).c_str());
            printf("    [%s] <int> to get the nominal current for a motor\n", Vocab::decode(VOCAB_AMP_NOMINAL_CURRENT).c_str());
            printf("    [%s] <int> to get the peak current for a motor\n", Vocab::decode(VOCAB_AMP_PEAK_CURRENT).c_str());
            printf("    [%s] <int> to get the PWM output for a motor\n", Vocab::decode(VOCAB_AMP_PWM).c_str());
            printf("    [%s] <int> to get the PWM limit for a motor\n", Vocab::decode(VOCAB_AMP_PWM_LIMIT).c_str());
            printf("    [%s] <int> to get the power supply voltage for a single motor\n", Vocab::decode(VOCAB_AMP_VOLTAGE_SUPPLY).c_str());
            printf("    [%s] <int> to get the current limit of single motor\n", Vocab::decode(VOCAB_AMP_MAXCURRENT).c_str());
            printf("    [%s] <int> to check motionDone on a single motor\n", Vocab::decode(VOCAB_MOTION_DONE).c_str());
            printf("    [%s] to check motionDone on all motors \n", Vocab::decode(VOCAB_MOTION_DONES).c_str());
            printf("    [%s] to check motionDone on all motors using multiple functions\n", Vocab::decode(VOCAB_MOTION_DONE_GROUP).c_str());
            printf("\n");

            printf("type [set] and one of the following:\n");
            printf("    [%s] <int> <double> to move a single axis\n", Vocab::decode(VOCAB_POSITION_MOVE).c_str());
            printf("    [%s] <int> <double> to accelerate a single axis to a given speed\n", Vocab::decode(VOCAB_VELOCITY_MOVE).c_str());            
            printf("    [%s] <int> <double> to set the reference speed for a single axis\n", Vocab::decode(VOCAB_REF_SPEED).c_str());
            printf("    [%s] <int> <double> to set the reference acceleration for a single axis\n", Vocab::decode(VOCAB_REF_ACCELERATION).c_str());
            printf("    [%s] <list> to move multiple axes\n", Vocab::decode(VOCAB_POSITION_MOVES).c_str());
            printf("    [%s] <list> to accelerate multiple axes to a given speed\n", Vocab::decode(VOCAB_VELOCITY_MOVES).c_str());
            printf("    [%s] <list> to set the reference speed for all axes\n", Vocab::decode(VOCAB_REF_SPEEDS).c_str());
            printf("    [%s] <list> to set the reference acceleration for all axes\n", Vocab::decode(VOCAB_REF_ACCELERATIONS).c_str());          
            printf("    [%s] <int> to stop a single axis\n", Vocab::decode(VOCAB_STOP).c_str());
            printf("    [%s] <int> to stop all axes\n", Vocab::decode(VOCAB_STOPS).c_str());
            printf("    [%s] <int> <double> to move single axis using position direct\n", Vocab::decode(VOCAB_POSITION_DIRECT).c_str());
            printf("    [%s] <list> to move multiple axes using position direct\n", Vocab::decode(VOCAB_POSITION_DIRECTS).c_str());
            printf("    [%s] <int> <list> to set the PID values for a single axis\n", Vocab::decode(VOCAB_PID).c_str());
            printf("    [%s] <int> <list> to set the limits for a single axis\n", Vocab::decode(VOCAB_LIMITS).c_str());
            printf("    [%s] <int> to disable the PID control for a single axis\n", Vocab::decode(VOCAB_DISABLE).c_str());
            printf("    [%s] <int> to enable the PID control for a single axis\n", Vocab::decode(VOCAB_ENABLE).c_str());
            printf("    [%s] <int> <double> to set the encoder value for a single axis\n", Vocab::decode(VOCAB_ENCODER).c_str());
            printf("    [%s] <list> to set the encoder value for all axes\n", Vocab::decode(VOCAB_ENCODERS).c_str());
            printf("    [%s] <int> <double> to set the current limit for single motor\n", Vocab::decode(VOCAB_AMP_MAXCURRENT).c_str());
            printf("    [%s] <int> <double> to set the peak current for a motor\n", Vocab::decode(VOCAB_AMP_PEAK_CURRENT).c_str());
            printf("    [%s] <int> <double> to set the PWM limit for a motor\n", Vocab::decode(VOCAB_AMP_PWM_LIMIT).c_str());
            printf("\n");

            printf("NOTES: - A list is a sequence of numbers in parenthesis, e.g. (10 2 1 10)\n");
            printf("       - Pids are expressed as a list of 7 numbers, type get pid <int> to see an example\n");
            printf("\n");

            if (options.check("debug"))
            {
                //#define VOCAB_CM_HW_FAULT           VOCAB4('h','w','f','a')
                //#define VOCAB_CM_CALIBRATING        VOCAB3('c','a','l')     // the joint is calibrating
                //#define VOCAB_CM_CALIB_DONE         VOCAB4('c','a','l','d') // calibration succesfully completed
                //#define VOCAB_CM_NOT_CONFIGURED     VOCAB4('c','f','g','n') // missing initial configuration (default value at start-up)
                //#define VOCAB_CM_CONFIGURED         VOCAB4('c','f','g','y') // initial configuration completed, if any
                printf("DEBUG NOTES: (hidden debug commands wich may break the robot! do not use!\n");
                printf("- icdd set hwfa 1 : will try to force joint 1 in hw fault\n");
                printf("- icdd set cal  1 : \n");
                printf("- icdd set cald 1 : \n");
                printf("- icdd set cfgy 1 : \n");
                printf("- icdd set cfgn 1 : \n");
                printf("- icdd set cmuk 1 : \n");
            }
            break;

        case VOCAB_QUIT:
            goto ApplicationCleanQuit;
            break;

        case VOCAB_ICONTROLMODE:
            {
                handleControlModeMsg(iMode2, p, response, &rec, &ok);
                printf("%s\n", response.toString().c_str());
                break;
            }

        case VOCAB_ICONTROLMODE_DEBUG:
            {
                handleControlModeMsg_DEBUG(iMode2, p, response, &rec, &ok);
                printf("%s\n", response.toString().c_str());
                break;
            }

        case VOCAB_IMPEDANCE:
            {
                handleImpedanceMsg(iimp, p, response, &rec, &ok);
                printf("%s\n", response.toString().c_str());
                break;
            }

        case VOCAB_TORQUE:
            {
                handleTorqueMsg(itorque, pid, p, response, &rec, &ok);
                printf("%s\n", response.toString().c_str());
                break;
            }

        case VOCAB_INTERFACE_INTERACTION_MODE:
            {
                handleInteractionModeMsg(iInteract, p, response, &rec, &ok);
                printf("%s\n", response.toString().c_str());
                break;
            }

        case VOCAB_GET:
            switch(p.get(1).asVocab())
            {
                case VOCAB_POSITION_MOVE:
                {
                    if(!ipos)
                    {
                        printf ("unavailable interface\n");
                        break;
                    }
                    double ref;
                    int j = p.get(2).asInt();
                    bool ret = ipos->getTargetPosition(j, &ref);
                    printf("Ref joint %d is %.2f - [ret val is %s]\n", j, ref, ret?"true":"false");
                }
                break;

                case VOCAB_POSITION_MOVES:
                {
                    if(!ipos)
                    {
                        printf ("unavailable interface\n");
                        break;
                    }
                    bool ret = ipos->getTargetPositions(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_POSITION_MOVES).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")  - ret is [%s]\n", ret? "true":"false");
                }
                break;

                case VOCAB_POSITION_DIRECT:
                {
                    ok = iposDir->getRefPosition(p.get(3).asInt(), tmp);
                    response.addDouble(tmp[0]);
                    printf("Ref joint %d is %.2f - [ret val is %s]\n", p.get(3).asInt(), tmp[0], ok?"true":"false");
                }
                break;

                case VOCAB_POSITION_DIRECTS:
                {
                    ok = iposDir->getRefPositions(tmp);
                    Bottle& b = response.addList();
                    int i;
                    for (i = 0; i < jnts; i++)
                        b.addDouble(tmp[i]);
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (" - ret is [%s]\n", ok? "true":"false");
                }
                break;

                case VOCAB_VELOCITY_MOVE:
                {
                    ok = vel->getRefVelocity(p.get(2).asInt(), tmp);
                    response.addDouble(tmp[0]);
                    printf("Ref vel joint %d is %.2f - [ret val is %s]\n", p.get(3).asInt(), tmp[0], ok?"true":"false");
                }
                break;

                case VOCAB_VELOCITY_MOVES:
                {
                    ok = vel->getRefVelocities(tmp);
                    Bottle& b = response.addList();
                    int i;
                    for (i = 0; i < jnts; i++)
                        b.addDouble(tmp[i]);
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (" - ret is [%s]\n", ok? "true":"false");
                }
                break;

                case VOCAB_INFO_NAME:
                {
                   int j = p.get(2).asInt();
                   if (iInfo == 0) { printf("unavailable interface\n"); break; }
                   std::string tmp_str;
                   iInfo->getAxisName(j,tmp_str);
                   printf("%s: %d %s\n", Vocab::decode(VOCAB_INFO_NAME).c_str(), j, tmp_str.c_str());
                }
                break;

                case VOCAB_AXES: {
                    int nj = 0;
                    enc->getAxes(&nj);
                    printf ("%s: %d\n", Vocab::decode(VOCAB_AXES).c_str(), nj);
                }
                break;

                case VOCAB_MOTION_DONE:
                {
                    if (ipos==0) {yError ("unavailable interface iPos\n"); break;}
                    bool b=false;
                    int j = p.get(2).asInt();
                    ipos->checkMotionDone(j,&b);
                    if (b==true) printf("1");
                    else printf ("0");
                }
                break;

                case VOCAB_MOTION_DONES:
                {
                    if (ipos==0) {yError ("unavailable interface iPos\n"); break;}
                    bool b=false;
                    ipos->checkMotionDone(&b);
                    if (b==true) printf("1");
                    else printf ("0");
                }
                break;

                case VOCAB_MOTION_DONE_GROUP:
                {
                    if (ipos==0) {yError ("unavailable interface iPos\n"); break;}
                    bool b=false;
                    ipos->checkMotionDone(jnts, jtmp ,&b);
                    if (b==true) printf("1");
                    else printf ("0");
                }
                break;

                case VOCAB_ENCODERS: {
                    enc->getEncoders(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_ENCODERS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_MOTOR_ENCODERS: {
                    if (iMotEnc==0) {printf ("unavailable interface\n"); break;}
                    iMotEnc->getMotorEncoders(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_MOTOR_ENCODERS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_MOTOR_ENCODER_SPEEDS: {
                    if (iMotEnc==0) {printf ("unavailable interface\n"); break;}
                    iMotEnc->getMotorEncoderSpeeds(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_MOTOR_ENCODER_SPEEDS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_MOTOR_ENCODER_ACCELERATIONS: {
                    if (iMotEnc==0) {printf ("unavailable interface\n"); break;}
                    iMotEnc->getMotorEncoderAccelerations(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_MOTOR_ENCODER_ACCELERATIONS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_MOTOR_CPR:
                {
                    if (iMotEnc==0) {printf ("unavailable interface\n"); break;}
                    int j = p.get(2).asInt();
                    double v;
                    iMotEnc->getMotorEncoderCountsPerRevolution(j, &v);
                    printf("%s: ", Vocab::decode(VOCAB_MOTOR_CPR).c_str());
                    printf("%.2f ", v);
                    printf("\n");
                }
                break;

                case VOCAB_AMP_MAXCURRENT:
                {
                    if (amp==0) {printf ("unavailable interface\n"); break;}
                    int j = p.get(2).asInt();
                    double v;
                    amp->getMaxCurrent(j, &v);
                    printf("%s: ", Vocab::decode(VOCAB_AMP_MAXCURRENT).c_str());
                    printf("%.2f ", v);
                    printf("\n");
                }
                break;

                case VOCAB_MOTORS_NUMBER:
                {
                    if (imot==0) {printf ("unavailable interface\n"); break;}
                    int v;
                    imot->getNumberOfMotors(&v);
                    printf("%s: ", Vocab::decode(VOCAB_MOTORS_NUMBER).c_str());
                    printf("%d ", v);
                    printf("\n");
                }
                break;

                case VOCAB_MOTOR_ENCODER_NUMBER:
                {
                    if (iMotEnc==0) {printf ("unavailable interface\n"); break;}
                    int v;
                    iMotEnc->getNumberOfMotorEncoders(&v);
                    printf("%s: ", Vocab::decode(VOCAB_MOTOR_ENCODER_NUMBER).c_str());
                    printf("%d ", v);
                    printf("\n");
                }
                break;

                case VOCAB_PID: {
                    Pid pd;
                    int j = p.get(2).asInt();
                    pid->getPid(VOCAB_PIDTYPE_POSITION, j, &pd);
                    printf("%s: ", Vocab::decode(VOCAB_PID).c_str());
                    printf("kp %.2f ", pd.kp);
                    printf("kd %.2f ", pd.kd);
                    printf("ki %.2f ", pd.ki);
                    printf("maxi %.2f ", pd.max_int);
                    printf("maxo %.2f ", pd.max_output);
                    printf("off %.2f ", pd.offset);
                    printf("scale %.2f ", pd.scale);
                    printf("\n");
                }
                break;

               case VOCAB_PIDS: {
                    Pid *p = new Pid[jnts];
                    ok = pid->getPids(VOCAB_PIDTYPE_POSITION, p);
                    Bottle& b = response.addList();
                    int i;
                    for (i = 0; i < jnts; i++)
                        {
                          Bottle& c = b.addList();
                          c.addDouble(p[i].kp);
                          c.addDouble(p[i].kd);
                          c.addDouble(p[i].ki);
                          c.addDouble(p[i].max_int);
                          c.addDouble(p[i].max_output);
                          c.addDouble(p[i].offset);
                          c.addDouble(p[i].scale);
                        }
                    printf("%s\n", b.toString().c_str());
                    delete[] p;
                }
                break;

                case VOCAB_LIMITS: {
                    double min, max;
                    int j = p.get(2).asInt();
                    lim->getLimits(j, &min, &max);
                    printf("%s: ", Vocab::decode(VOCAB_LIMITS).c_str());
                    printf("limits: (%.2f %.2f)\n", min, max);
                }
                break;

                case VOCAB_VEL_LIMITS: {
                     double min, max;
                     int j = p.get(2).asInt();
                     lim->getVelLimits(j, &min, &max);
                     printf("%s: ", Vocab::decode(VOCAB_VEL_LIMITS).c_str());
                     printf("limits: (%.2f %.2f)\n", min, max);
                }
                break;

                case VOCAB_ERRS: {
                    pid->getPidErrors(VOCAB_PIDTYPE_POSITION, tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_ERRS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;
                
                case VOCAB_TEMPERATURES: {
                    if (imot==0) {printf ("unavailable interface\n"); break;}
                    imot->getTemperatures(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_TEMPERATURES).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_TEMPERATURE: {
                    if (imot==0) {printf ("unavailable interface\n"); break;}
                    int j = p.get(2).asInt();
                    double v;
                    imot->getTemperature(j, &v);
                    printf("%s: ", Vocab::decode(VOCAB_TEMPERATURE).c_str());
                    printf("%.2f ", v);
                    printf("\n");
                }
                break;

                case VOCAB_TEMPERATURE_LIMIT: {
                    if (imot==0) {printf ("unavailable interface\n"); break;}
                    int j = p.get(2).asInt();
                    double v;
                    imot->getTemperatureLimit(j, &v);
                    printf("%s: ", Vocab::decode(VOCAB_TEMPERATURE_LIMIT).c_str());
                    printf("%.2f ", v);
                    printf("\n");
                }
                break;

                case VOCAB_PWMCONTROL_PWM_OUTPUTS: {
                    if (ipwm==0) {printf ("unavailable interface\n"); break;}
                    ipwm->getDutyCycles(tmp);
                    printf("%s: (", Vocab::decode(VOCAB_PWMCONTROL_PWM_OUTPUTS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_PWMCONTROL_PWM_OUTPUT: 
                //case VOCAB_AMP_PWM:
                {
                    if (ipwm == 0) { printf("unavailable interface\n"); break; }
                    int j = p.get(2).asInt();
                    double v;
                    ipwm->getDutyCycle(j, &v);
                    printf("%s: ", Vocab::decode(VOCAB_PWMCONTROL_PWM_OUTPUT).c_str());
                    printf("%.2f ", v);
                    printf("\n");
                }
                break;

                case VOCAB_REFERENCE: {
                    double ref_pos;
                    int j = p.get(2).asInt();
                    pid->getPidReference(VOCAB_PIDTYPE_POSITION, j,&ref_pos);
                    printf ("%s: (", Vocab::decode(VOCAB_REFERENCE).c_str());
                    printf ("%.2f ", ref_pos);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REFERENCES: {
                    pid->getPidReferences(VOCAB_PIDTYPE_POSITION, tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_REFERENCES).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REF_SPEEDS: {
                    ipos->getRefSpeeds(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_REF_SPEEDS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REF_SPEED: {
                    double ref_speed;
                    int j = p.get(2).asInt();
                    ipos->getRefSpeed(j,&ref_speed);
                    printf ("%s: (", Vocab::decode(VOCAB_REF_SPEED).c_str());
                    printf ("%.2f ", ref_speed);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REF_ACCELERATION: {
                    double ref_acc;
                    int j = p.get(2).asInt();
                    ipos->getRefAcceleration(j,&ref_acc);
                    printf ("%s: (", Vocab::decode(VOCAB_REF_ACCELERATION).c_str());
                    printf ("%.2f ", ref_acc);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REF_ACCELERATIONS: {
                    ipos->getRefAccelerations(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_REF_ACCELERATIONS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");                    
                }
                break;

                case VOCAB_AMP_CURRENTS: {
                    amp->getCurrents(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_AMP_CURRENTS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_AMP_PWM_LIMIT:
                {
                    int j = p.get(2).asInt();
                    amp->getPWMLimit(j, tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_AMP_PWM_LIMIT).c_str());
                    printf ("%.2f )\n", tmp[0]);
                }
                break;

                case VOCAB_AMP_PEAK_CURRENT:
                {
                    int j = p.get(2).asInt();
                    amp->getPeakCurrent(j, tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_AMP_PEAK_CURRENT).c_str());
                    printf ("%.2f )\n", tmp[0]);
                }
                break;

                case VOCAB_AMP_NOMINAL_CURRENT:
                {
                    int j = p.get(2).asInt();
                    amp->getNominalCurrent(j, tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_AMP_NOMINAL_CURRENT).c_str());
                    printf ("%.2f )\n", tmp[0]);
                }
                break;

                case VOCAB_AMP_VOLTAGE_SUPPLY:
                {
                    int j = p.get(2).asInt();
                    amp->getPowerSupplyVoltage(j, tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_AMP_VOLTAGE_SUPPLY).c_str());
                    printf ("%.2f )\n", tmp[0]);
                }
                break;
            }
            break;

        case VOCAB_SET:
            switch(p.get(1).asVocab()) {
                case VOCAB_POSITION_MOVE: {
                    int j = p.get(2).asInt();
                    double ref = p.get(3).asDouble();
                    printf("%s: moving %d to %.2f\n", Vocab::decode(VOCAB_POSITION_MOVE).c_str(), j, ref);
                    ipos->positionMove(j, ref);
                }
                break;

                case VOCAB_VELOCITY_MOVE: {
                    int j = p.get(2).asInt();
                    double ref = p.get(3).asDouble();
                    printf("%s: accelerating %d to %.2f\n", Vocab::decode(VOCAB_VELOCITY_MOVE).c_str(), j, ref);
                    vel->velocityMove(j, ref);
                }
                break;

                case VOCAB_REF_SPEED: {
                    int j = p.get(2).asInt();
                    double ref = p.get(3).asDouble();
                    printf("%s: setting speed for %d to %.2f\n", Vocab::decode(VOCAB_REF_SPEED).c_str(), j, ref);
                    ipos->setRefSpeed(j, ref);
                }
                break;

                case VOCAB_REF_ACCELERATION: {
                    int j = p.get(2).asInt();
                    double ref = p.get(3).asDouble();
                    printf("%s: setting acceleration for %d to %.2f\n", Vocab::decode(VOCAB_REF_ACCELERATION).c_str(), j, ref);
                    ipos->setRefAcceleration(j, ref);
                }
                break;

                case VOCAB_POSITION_MOVES: {
                    Bottle *l = p.get(2).asList();
                    printf("received %s\n", l->toString().c_str());
                    for (i = 0; i < jnts; i++) {
                        printf("%d - ", i); fflush(stdout);
                        tmp[i] = l->get(i).asDouble();
                        printf("tmp[i] %f\n", tmp[i]); fflush(stdout);
                    }
                    printf("%s: moving all joints\n", Vocab::decode(VOCAB_POSITION_MOVES).c_str());
                    ipos->positionMove(tmp);
                }
                break;

                case VOCAB_VELOCITY_MOVES: {
                    Bottle *l = p.get(2).asList();
                    for (i = 0; i < jnts; i++) {
                        tmp[i] = l->get(i).asDouble();
                    }
                    printf("%s: moving all joints\n", Vocab::decode(VOCAB_VELOCITY_MOVES).c_str());
                    vel->velocityMove(tmp);
                }
                break;

                case VOCAB_REF_SPEEDS: {
                    Bottle *l = p.get(2).asList();
                    for (i = 0; i < jnts; i++) {
                        tmp[i] = l->get(i).asDouble();
                    }
                    printf("%s: setting speed for all joints\n", Vocab::decode(VOCAB_REF_SPEEDS).c_str());
                    ipos->setRefSpeeds(tmp);
                }
                break;

                case VOCAB_REF_ACCELERATIONS: {
                    Bottle *l = p.get(2).asList();
                    for (i = 0; i < jnts; i++) {
                        tmp[i] = l->get(i).asDouble();
                    }
                    printf("%s: setting acceleration for all joints\n", Vocab::decode(VOCAB_REF_ACCELERATIONS).c_str());
                    ipos->setRefAccelerations(tmp);
                }
                break;

                case VOCAB_STOP: {
                    int j = p.get(2).asInt();
                    printf("%s: stopping axis %d\n", Vocab::decode(VOCAB_STOP).c_str(), j);
                    ipos->stop(j);
                }
                break;

                case VOCAB_STOPS: {
                    printf("%s: stopping all axes\n", Vocab::decode(VOCAB_STOPS).c_str());
                    ipos->stop();
                }
                break;


                case VOCAB_POSITION_DIRECT: {
                    printf("%s: setting position direct reference\n", Vocab::decode(VOCAB_POSITION_DIRECT).c_str());
                    iposDir->setPosition(p.get(2).asInt(), p.get(3).asDouble());
                }
                break;

                case VOCAB_POSITION_DIRECTS: {
                    printf("%s: setting position direct references to %s\n", Vocab::decode(VOCAB_POSITION_DIRECTS).c_str(), p.toString().c_str() );
                    Bottle *l = p.get(2).asList();
                    for (i = 0; i < jnts; i++) {
                        tmp[i] = l->get(i).asDouble();
                    }
                    iposDir->setPositions(tmp);
                }
                break;

                case VOCAB_ENCODER: {
                    int j = p.get(2).asInt();
                    double ref = p.get(3).asDouble();
                    printf("%s: setting the encoder value for %d to %.2f\n", Vocab::decode(VOCAB_ENCODER).c_str(), j, ref);
                    enc->setEncoder(j, ref);                    
                }
                break; 

                case VOCAB_ENCODERS: {
                    Bottle *l = p.get(2).asList();
                    for (i = 0; i < jnts; i++) {
                        tmp[i] = l->get(i).asDouble();
                    }
                    printf("%s: setting the encoder value for all joints\n", Vocab::decode(VOCAB_ENCODERS).c_str());
                    enc->setEncoders(tmp);
                }
                break;

                case VOCAB_PID: {
                    Pid pd;
                    int j = p.get(2).asInt();
                    Bottle *l = p.get(3).asList();
                    if (l==0)
                        {
                            printf("Check you specify a 7 elements list, e.g. set pid 0 (2000 20 1 300 300 0 0)\n");
                        }
                    else
                        {
                            int elems=l->size();
                            if (elems>=3)
                                {
                                    pd.kp = l->get(0).asDouble();
                                    pd.kd = l->get(1).asDouble();
                                    pd.ki = l->get(2).asDouble();
                                    if (elems>=7)
                                        {
                                            pd.max_int = l->get(3).asDouble();
                                            pd.max_output = l->get(4).asDouble();
                                            pd.offset = l->get(5).asDouble();
                                            pd.scale = l->get(6).asDouble();
                                        }
                                    printf("%s: setting PID values for axis %d\n", Vocab::decode(VOCAB_PID).c_str(), j);
                                    pid->setPid(VOCAB_PIDTYPE_POSITION, j, pd);
                                }
                            else
                                {
                                    printf("Error, check you specify at least 7 elements, e.g. set pid 0 (2000 20 1 300 300 0 0)\n");
                                }
                        }
                }
                break;

                case VOCAB_TEMPERATURE_LIMIT: {
                    if (imot==0) {printf ("unavailable interface\n"); break;}
                    int j=p.get(2).asInt();
                    double v=p.get(3).asDouble();
                    imot->setTemperatureLimit(j,v);
                    printf("%s: setting temperature limit for axis %d to %f\n", Vocab::decode(VOCAB_TEMPERATURE_LIMIT).c_str(), j, v);            
                }
                break;

                case VOCAB_AMP_MAXCURRENT: {
                    int j=p.get(2).asInt();
                    double v=p.get(3).asDouble();
                    amp->setMaxCurrent(j,v);
                    printf("%s: setting max current for motor %d to %f\n", Vocab::decode(VOCAB_AMP_MAXCURRENT).c_str(), j, v);            
                }
                break;

                case VOCAB_DISABLE: {
                    int j = p.get(2).asInt();
                    printf("%s: disabling control for axis %d\n", Vocab::decode(VOCAB_DISABLE).c_str(), j);
                    pid->disablePid(VOCAB_PIDTYPE_POSITION,j);
                    amp->disableAmp(j);
                }
                break;

                case VOCAB_ENABLE: {
                    int j = p.get(2).asInt();
                    printf("%s: enabling control for axis %d\n", Vocab::decode(VOCAB_ENABLE).c_str(), j);
                    amp->enableAmp(j);
                    pid->enablePid(VOCAB_PIDTYPE_POSITION,j);
                }
                break;

                case VOCAB_LIMITS: {
                    int j = p.get(2).asInt();
                    printf("%s: setting limits for axis %d\n", Vocab::decode(VOCAB_LIMITS).c_str(), j);
                    Bottle *l = p.get(3).asList();
                    lim->setLimits(j, l->get(0).asDouble(), l->get(1).asDouble());
                }
                break;

                case VOCAB_AMP_PWM_LIMIT:
                {
                    int j = p.get(2).asInt();
                    double lim = p.get(3).asDouble();
                    bool ret = amp->setPWMLimit(j, lim);
                    printf("%s: setting PWM limits for axis %d to %0.2f - ret: %d\n", Vocab::decode(VOCAB_AMP_PWM_LIMIT).c_str(), j, lim, ret);
                }
                break;

                case VOCAB_AMP_PEAK_CURRENT:
                {
                    int j = p.get(2).asInt();
                    double lim = p.get(3).asDouble();
                    bool ret = amp->setPeakCurrent(j, lim);
                    printf("%s: setting peak current for axis %d to %0.2f - ret: %d\n", Vocab::decode(VOCAB_AMP_PEAK_CURRENT).c_str(), j, lim, ret);
                }
                break;

                case VOCAB_PWMCONTROL_REF_PWM: {
                    int j=p.get(2).asInt();
                    double v=p.get(3).asDouble();
                    ipwm->setRefDutyCycle(j,v);
                    printf("%s: setting pwm for axis %d to %f\n", Vocab::decode(VOCAB_PWMCONTROL_REF_PWM).c_str(), j, v);
                }
                break;
            }
            break;
        } /* switch get(0) */

    } /* while () */

ApplicationCleanQuit:
    dd.close();
    delete[] tmp;
    delete[] btmp;
    delete[] jtmp;

    Network::fini();
    return 0;
}

void handleTorqueMsg(ITorqueControl *torque, IPidControl *ipid, const yarp::os::Bottle& cmd,
                     yarp::os::Bottle& response, bool *rec, bool *ok) 
{
    fprintf(stderr, "Handling ITorque messages\n");

    if (!torque)
        {
            fprintf(stderr, "Error, I do not have a valid ITorque interface\n");
            *ok=false;
            return;
        }
    
    int controlledJoints;
    torque->getAxes(&controlledJoints);

    int code = cmd.get(1).asVocab();
    switch (code)
        {
        case VOCAB_SET:
            {
                *rec = true;
                
                switch(cmd.get(2).asVocab())
                    {
                    case VOCAB_REF: 
                        {
                            *ok = torque->setRefTorque(cmd.get(3).asInt(), cmd.get(4).asDouble());
                        }
                        break;

                    case VOCAB_REFS: 
                        {
                            Bottle& b = *(cmd.get(3).asList());
                            int i;
                            const int njs = b.size();
                            if (njs==controlledJoints)
                                {
                                    double *p = new double[njs];    // LATER: optimize to avoid allocation. 
                                    for (i = 0; i < njs; i++)
                                        p[i] = b.get(i).asDouble();
                                    *ok = torque->setRefTorques (p);
                                    delete[] p;
                                }
                        }
                        break;


                    case VOCAB_REFG:
                        {
                            int n_joint = cmd.get(3).asInt();

                            Bottle& jointList = *(cmd.get(4).asList());
                            Bottle& refList   = *(cmd.get(5).asList());
                            int i;
                            const int joint_size = jointList.size();
                            const int refs_size  = refList.size();

                            if( (joint_size != n_joint) || (refs_size != n_joint) )
                            {
                                yError() << "command malformed, size of joint list and reference list must match the number of joints (" << n_joint << ")";
                                *rec = false;
                                break;
                            }

                            if (n_joint > controlledJoints)
                            {
                                yError() << "command malformed, number of joints inserted is bigger than number of available joints (" << controlledJoints << ")";
                                *rec = false;
                                break;
                            }

                                int    *joints = new int[n_joint];       // LATER: optimize to avoid allocation.
                                double *refs   = new double[n_joint];    // LATER: optimize to avoid allocation.

                                for (i = 0; i < n_joint; i++)
                                {
                                    joints[i] = jointList.get(i).asInt();
                                    refs[i]   = refList.get(i).asDouble();
                                }
                                *ok = torque->setRefTorques(n_joint, joints, refs);
                                delete[] joints;
                                delete[] refs;
                        }
                        break;

                    case VOCAB_LIM: 
                        {
                            *ok = ipid->setPidErrorLimit (VOCAB_PIDTYPE_POSITION, cmd.get(3).asInt(), cmd.get(4).asDouble());
                        }
                        break;

                    case VOCAB_LIMS: 
                        {
                            Bottle& b = *(cmd.get(3).asList());
                            int i;
                            const int njs = b.size();
                            if (njs==controlledJoints)
                                {
                                    double *p = new double[njs];    // LATER: optimize to avoid allocation. 
                                    for (i = 0; i < njs; i++)
                                        p[i] = b.get(i).asDouble();
                                    *ok = ipid->setPidErrorLimits (VOCAB_PIDTYPE_POSITION, p);
                                    delete[] p;                
                                }        
                        }
                        break;

                    case VOCAB_PID: 
                        {
                            Pid p;
                            int j = cmd.get(3).asInt();
                            Bottle& b = *(cmd.get(4).asList());
                            p.kp = b.get(0).asDouble();
                            p.kd = b.get(1).asDouble();
                            p.ki = b.get(2).asDouble();
                            p.max_int = b.get(3).asDouble();
                            p.max_output = b.get(4).asDouble();
                            p.offset = b.get(5).asDouble();
                            p.scale = b.get(6).asDouble();
                            *ok = ipid->setPid(VOCAB_PIDTYPE_TORQUE,j, p);
                        }
                        break;

                    case VOCAB_PIDS: 
                        {
                            Bottle& b = *(cmd.get(3).asList());
                            int i;
                            const int njs = b.size();
                            if (njs==controlledJoints)
                                {
                                    Pid *p = new Pid[njs];
                                    for (i = 0; i < njs; i++)
                                        {
                                            Bottle& c = *(b.get(i).asList());
                                            p[i].kp = c.get(0).asDouble();
                                            p[i].kd = c.get(1).asDouble();
                                            p[i].ki = c.get(2).asDouble();
                                            p[i].max_int = c.get(3).asDouble();
                                            p[i].max_output = c.get(4).asDouble();
                                            p[i].offset = c.get(5).asDouble();
                                            p[i].scale = c.get(6).asDouble();
                                        }
                                    *ok = ipid->setPids(VOCAB_PIDTYPE_TORQUE,p);
                                    delete[] p;
                                }
                        }
                        break;

                    case VOCAB_RESET: 
                        {
                            *ok = ipid->resetPid (VOCAB_PIDTYPE_TORQUE,cmd.get(3).asInt());
                        }
                        break;

                    case VOCAB_DISABLE:
                        {
                            *ok = ipid->disablePid (VOCAB_PIDTYPE_TORQUE,cmd.get(3).asInt());
                        }
                        break;

                    case VOCAB_ENABLE: 
                        {
                            *ok = ipid->enablePid (VOCAB_PIDTYPE_TORQUE,cmd.get(3).asInt());
                        }
                        break;

                    }
            }
            break;

        case VOCAB_GET:
            {
                *rec = true;

                int tmp = 0;
                double dtmp = 0.0;
                response.addVocab(VOCAB_IS);
                response.add(cmd.get(1));

                switch(cmd.get(2).asVocab()) 
                    {
                    case VOCAB_AXES:
                        {
                            int tmp;
                            *ok = torque->getAxes(&tmp);
                            response.addInt(tmp);
                        }
                        break;

                    case VOCAB_TRQ:
                        {
                            *ok = torque->getTorque(cmd.get(3).asInt(), &dtmp);
                            response.addDouble(dtmp);
                        }
                        break;

                    case VOCAB_TRQS:
                        {
                            double *p = new double[controlledJoints];
                            *ok = torque->getTorques(p);
                            Bottle& b = response.addList();
                            int i;
                            for (i = 0; i < controlledJoints; i++)
                                b.addDouble(p[i]);
                            delete[] p;
                        }
                        break;

                    case VOCAB_ERR: 
                        {
                            *ok = ipid->getPidError(VOCAB_PIDTYPE_TORQUE,cmd.get(3).asInt(), &dtmp);
                            response.addDouble(dtmp);
                        }
                        break;

                    case VOCAB_ERRS: 
                        {
                            double *p = new double[controlledJoints];
                            *ok = ipid->getPidErrors(VOCAB_PIDTYPE_TORQUE,p);
                            Bottle& b = response.addList();
                            int i;
                            for (i = 0; i < controlledJoints; i++)
                                b.addDouble(p[i]);
                            delete[] p;
                        }
                        break;

                    case VOCAB_OUTPUT: 
                        {
                            *ok = ipid->getPidOutput(VOCAB_PIDTYPE_TORQUE,cmd.get(3).asInt(), &dtmp);
                            response.addDouble(dtmp);
                        }
                        break;

                    case VOCAB_OUTPUTS: 
                        {
                            double *p = new double[controlledJoints];
                            *ok = ipid->getPidOutputs(VOCAB_PIDTYPE_TORQUE,p);
                            Bottle& b = response.addList();
                            int i;
                            for (i = 0; i < controlledJoints; i++)
                                b.addDouble(p[i]);
                            delete[] p;
                        }
                        break;

                    case VOCAB_PID: 
                        {
                            Pid p;
                            *ok = ipid->getPid(VOCAB_PIDTYPE_TORQUE,cmd.get(3).asInt(), &p);
                            Bottle& b = response.addList();
                            b.addDouble(p.kp);
                            b.addDouble(p.kd);
                            b.addDouble(p.ki);
                            b.addDouble(p.max_int);
                            b.addDouble(p.max_output);
                            b.addDouble(p.offset);
                            b.addDouble(p.scale);
                        }
                        break;

                    case VOCAB_PIDS: 
                        {
                            Pid *p = new Pid[controlledJoints];
                            *ok = ipid->getPids(VOCAB_PIDTYPE_TORQUE,p);
                            Bottle& b = response.addList();
                            int i;
                            for (i = 0; i < controlledJoints; i++)
                                {
                                    Bottle& c = b.addList();
                                    c.addDouble(p[i].kp);
                                    c.addDouble(p[i].kd);
                                    c.addDouble(p[i].ki);
                                    c.addDouble(p[i].max_int);
                                    c.addDouble(p[i].max_output);
                                    c.addDouble(p[i].offset);
                                    c.addDouble(p[i].scale);
                                }
                            delete[] p;
                        }
                        break;

                    case VOCAB_REFERENCE: 
                        {
                            *ok = torque->getRefTorque(cmd.get(3).asInt(), &dtmp);
                            response.addDouble(dtmp);
                        }
                        break;

                    case VOCAB_REFERENCES:
                        {
                            double *p = new double[controlledJoints];
                            *ok = torque->getRefTorques(p);
                            Bottle& b = response.addList();
                            int i;
                            for (i = 0; i < controlledJoints; i++)
                                b.addDouble(p[i]);
                            delete[] p;
                        }
                        break;

                    case VOCAB_LIM:
                        {
                            *ok = ipid->getPidErrorLimit(VOCAB_PIDTYPE_TORQUE,cmd.get(3).asInt(), &dtmp);
                            response.addDouble(dtmp);
                        }
                        break;

                    case VOCAB_LIMS: 
                        {
                            double *p = new double[controlledJoints];
                            *ok = ipid->getPidErrorLimits(VOCAB_PIDTYPE_TORQUE,p);
                            Bottle& b = response.addList();
                            int i;
                            for (i = 0; i < controlledJoints; i++)
                                b.addDouble(p[i]);
                            delete[] p;
                        }
                        break;

                    }
            }
            break;
        }
    //rec --> true se il comando e' riconosciuto
    //ok --> contiene il return value della chiamata all'interfaccia
    // ...*ok=torque->setPid();
    //torque->
}

void handleImpedanceMsg(IImpedanceControl *iimp, const yarp::os::Bottle& cmd,
                     yarp::os::Bottle& response, bool *rec, bool *ok) 
{
    fprintf(stderr, "Handling IImpedance messages\n");

    if (!iimp)
        {
            fprintf(stderr, "Error, I do not have a valid IImpedance interface\n");
            *ok=false;
            return;
        }
    
    int controlledJoints;
    iimp->getAxes(&controlledJoints);

    int code = cmd.get(1).asVocab();
    switch (code)
        {
        case VOCAB_SET:
            {
                *rec = true;
                
                switch(cmd.get(2).asVocab())
                    {
                    case VOCAB_IMP_PARAM: 
                        {
                            *ok = iimp->setImpedance(cmd.get(3).asInt(), cmd.get(4).asDouble(),cmd.get(5).asDouble());
                        }
                        break;
                    case VOCAB_IMP_OFFSET: 
                        {
                            *ok = iimp->setImpedanceOffset (cmd.get(3).asInt(), cmd.get(4).asDouble());
                        }
                        break;
                    }
            }
            break;

        case VOCAB_GET:
            {
                *rec = true;

                int tmp = 0;
                double dtmp0 = 0.0;
                double dtmp1 = 0.0;
                double dtmp2 = 0.0;
                response.addVocab(VOCAB_IS);
                response.add(cmd.get(1));

                switch(cmd.get(2).asVocab()) 
                    {

                    case VOCAB_IMP_PARAM:
                        {
                            *ok = iimp->getImpedance(cmd.get(3).asInt(), &dtmp0, &dtmp1);
                            response.addDouble(dtmp0);
                            response.addDouble(dtmp1);
                        }
                        break;

                    case VOCAB_IMP_OFFSET: 
                        {
                            *ok = iimp->getImpedanceOffset(cmd.get(3).asInt(), &dtmp0);
                            response.addDouble(dtmp0);
                        }
                        break;
                    }
            }
            break;
        }
    //rec --> true se il comando e' riconosciuto
    //ok --> contiene il return value della chiamata all'interfaccia
    // ...*ok=torque->setPid();
    //torque->
}

void handleControlModeMsg_DEBUG(IControlMode *iMode, const yarp::os::Bottle& cmd,
                          yarp::os::Bottle& response, bool *rec, bool *ok)
{
    //THE PURPOSE OF THIS FUCTION IS BEING ABLE TO SET ALL POSSIBILE CONTROL MODES, ALSO THE ONES THAT CANNOT BE NORMALLY SET (e.g. HW_FAULT)
    //THIS IS USEFUL FOR DEBUG PURPOSES e.g. IN THE SIMULATOR
    fprintf(stderr, "Handling IControlMode message %s, DEBUG MODE\n", cmd.toString().c_str());
    if (!iMode)
        {
            fprintf(stderr, "Error I do not have a valid interface\n");
            *ok=false;
            return;
        }

    int code = cmd.get(1).asVocab();
    *ok=true;

    switch(code)
        {
        case VOCAB_SET:
            {
                int axis = cmd.get(3).asInt();
                yarp::os::Value mode_vocab=cmd.get(2);
                int mode = mode_vocab.asInt();
                printf ("setting mode: %s (%d)",mode_vocab.toString().c_str(), mode);
                *ok = iMode->setControlMode(axis, mode);
            }
        case VOCAB_GET:
            {
                if (cmd.get(2).asVocab()==VOCAB_CM_CONTROL_MODE)
                {
                    int p=-1;
                    int axis = cmd.get(3).asInt();
                    fprintf(stderr, "Calling getControlMode\n");
                    *ok = iMode->getControlMode(axis, &p);

                        response.addVocab(VOCAB_IS);
                        response.addInt(axis);
                        response.addVocab(VOCAB_CM_CONTROL_MODE);       
                        response.addVocab(p);
            
                        //fprintf(stderr, "Returning %d\n", p);
                        *rec=true;
                 }
            }
        break;
        default:
            {
                *rec=false;
            }
        break;
    }
}

void handleControlModeMsg(IControlMode *iMode, const yarp::os::Bottle& cmd,
                          yarp::os::Bottle& response, bool *rec, bool *ok)
{
    fprintf(stderr, "Handling IControlMode message %s\n", cmd.toString().c_str());
    if (!iMode)
        {
            fprintf(stderr, "Error I do not have a valid interface\n");
            *ok=false;
            return;
        }

    int code = cmd.get(1).asVocab();
    *ok=true;

    switch(code)
        {
        case VOCAB_SET:
            {
                int axis = cmd.get(3).asInt();
                int mode=cmd.get(2).asVocab();
                switch (mode)
                    {
                    case VOCAB_CM_POSITION:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_POSITION);
                        break;
                    case VOCAB_CM_POSITION_DIRECT:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_POSITION_DIRECT);
                        break;
                    case VOCAB_CM_VELOCITY:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_VELOCITY);
                        break;
                    case VOCAB_CM_MIXED:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_MIXED);
                        break;
                    case VOCAB_CM_TORQUE:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_TORQUE);
                        break;
                    case VOCAB_CM_PWM:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_PWM);
                        break;
                    case VOCAB_CM_CURRENT:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_CURRENT);
                        break;
                    case VOCAB_CM_IDLE:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_IDLE);
                        break;
                    case VOCAB_CM_FORCE_IDLE:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_FORCE_IDLE);
                        break;
                    case VOCAB_CM_IMPEDANCE_POS:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_IMPEDANCE_POS);
                        break;
                    case VOCAB_CM_IMPEDANCE_VEL:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_IMPEDANCE_VEL);
                        break;
                    default:
                        *ok = false;
                        break;
                    }
                *rec=true; //or false
            }
            break;
        case VOCAB_GET:
            {
                if (cmd.get(2).asVocab()==VOCAB_CM_CONTROL_MODE)
                    {
                        int p=-1;
                        int axis = cmd.get(3).asInt();
                        fprintf(stderr, "Calling getControlMode\n");
                        *ok = iMode->getControlMode(axis, &p);

                        response.addVocab(VOCAB_IS);
                        response.addInt(axis);
                        response.addVocab(VOCAB_CM_CONTROL_MODE);       
                        response.addVocab(p);
            
                        //fprintf(stderr, "Returning %d\n", p);
                        *rec=true;
                    }
            }
            break;
        default:
            {
                *rec=false;
            }
            break;
        }
}



void handleInteractionModeMsg(IInteractionMode *iInteract, const yarp::os::Bottle& cmd,
                          yarp::os::Bottle& response, bool *rec, bool *ok)
{
    fprintf(stderr, "Handling IInteractionMode message %s\n", cmd.toString().c_str());
    if (!iInteract)
    {
        fprintf(stderr, "Error I do not have a valid interface\n");
        *ok=false;
        return;
    }

    int code = cmd.get(1).asVocab();
    *ok=true;

    switch(code)
    {
        case VOCAB_SET:
        {
            int axis = cmd.get(3).asInt();
            yarp::dev::InteractionModeEnum mode = (yarp::dev::InteractionModeEnum) cmd.get(2).asVocab();
            *ok = iInteract->setInteractionMode(axis, mode);
            *rec=true; //or false
        }
        break;

        case VOCAB_GET:
        {
            int which = cmd.get(2).asVocab();
            switch(which)
            {
                case VOCAB_INTERACTION_MODE:
                {
                    int axis = cmd.get(3).asInt();
                    yarp::dev::InteractionModeEnum mode;
                    *ok = iInteract->getInteractionMode(axis, &mode);
                    // create response
                    if(*ok)
                    {
                        response.addVocab(VOCAB_IS);
                        response.addInt(axis);
                        response.addVocab(VOCAB_INTERACTION_MODE);
                        response.addVocab(mode);
                        *rec=true;
                    }
                    else
                    {
                        response.addVocab(VOCAB_FAILED);
                        *rec = false;
                    }
                }
                break;

                case VOCAB_INTERACTION_MODES:
                {
                    int axis = cmd.get(3).asInt();
                    yarp::dev::InteractionModeEnum *modes;
                    modes = new yarp::dev::InteractionModeEnum[jnts];

                    *ok = iInteract->getInteractionMode(axis, modes);
                    // create response
                    if(*ok)
                    {
                        response.addVocab(VOCAB_IS);
                        response.addVocab(VOCAB_INTERACTION_MODES);
                        for(int i=0; i<jnts; i++)
                            response.addVocab(modes[i]);
                        *rec=true;
                    }
                    else
                    {
                        response.addVocab(VOCAB_FAILED);
                        *rec = false;
                    }
                }
                break;

                default:
                {
                    fprintf(stderr, "get command not understood");
                    *rec=false;
                    break;
                }
                break;
            }
        }
        break;
        default:
        {
            fprintf(stderr, "type of command not understood");
            *rec=false;
        }
        break;
    }
}
