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

#include <string>

using namespace yarp::dev;
using namespace yarp::os;

#define VOCAB_HELP VOCAB4('h','e','l','p')
#define VOCAB_QUIT VOCAB4('q','u','i','t')

void handleTorqueMsg(ITorqueControl *itq, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);
void handleImpedanceMsg(IImpedanceControl *iimp, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);
void handleControlModeMsg(IControlMode2 *icm, const yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool *rec, bool *ok);
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
        return 0;
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

    IPositionControl *pos;
    IPositionDirect  *posDir;
    IVelocityControl *vel;
    IEncoders *enc;
    IPidControl *pid;
    IAmplifierControl *amp;
    IControlLimits *lim;
//    IControlMode *icm;
    IControlMode2 *iMode2;
    ITorqueControl *itorque;
    IOpenLoopControl *iopenloop;
	IImpedanceControl *iimp;
    IInteractionMode *iInteract;

    bool ok;
    ok = dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);
//    ok &= dd.view(icm);
    ok &= dd.view(itorque);
    ok &= dd.view(iopenloop);
	ok &= dd.view(iimp);
    ok &= dd.view(posDir);
    ok &= dd.view(iMode2);
    ok &= dd.view(iInteract);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 1;
    }

    pos->getAxes(&jnts);
    printf("Working with %d axes\n", jnts);
    double *tmp = new double[jnts];

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

            printf("IOpenLoop:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_IOPENLOOP).c_str());
            printf("	[set] [%s] <int> <float>\n",
                    Vocab::decode(VOCAB_OUTPUT).c_str());
            printf("	[get] [%s] <int>\n",
                    Vocab::decode(VOCAB_OUTPUT).c_str());
            printf("	[get] [%s]\n\n",
                    Vocab::decode(VOCAB_OUTPUTS).c_str());

            printf("IControlMode:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_ICONTROLMODE).c_str());
            printf("	[set] [%s]|[%s]|[%s]|[%s]|[%s]|[%s]|[%s]|[%s][%s]|[%s]\n",
                    Vocab::decode(VOCAB_CM_POSITION).c_str(),
                    Vocab::decode(VOCAB_CM_POSITION_DIRECT).c_str(),
                    Vocab::decode(VOCAB_CM_VELOCITY).c_str(),
                    Vocab::decode(VOCAB_CM_MIXED).c_str(),
                    Vocab::decode(VOCAB_CM_TORQUE).c_str(),
                    Vocab::decode(VOCAB_CM_OPENLOOP).c_str(),
                    Vocab::decode(VOCAB_CM_IDLE).c_str(),
                    Vocab::decode(VOCAB_CM_FORCE_IDLE).c_str(),
                    Vocab::decode(VOCAB_CM_IMPEDANCE_POS).c_str(),
                    Vocab::decode(VOCAB_CM_IMPEDANCE_VEL).c_str());
            
            printf("	[get] [%s] <int>\n\n",
                Vocab::decode(VOCAB_CM_CONTROL_MODE).c_str());

            printf("ITorqueControl:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_TORQUE).c_str());
            printf("	[get] [%s] <int> to read the measured torque for a single axis\n",                  Vocab::decode(VOCAB_TRQ).c_str());
            printf("	[get] [%s]  to read the measured torque for all axes\n",                      Vocab::decode(VOCAB_TRQS).c_str());
            printf("	[set] [%s] <int> <float> to set the reference torque for a single axis\n",          Vocab::decode(VOCAB_REF).c_str());
            printf("	[set] [%s] <float list> to set the reference torque for all axes\n",        Vocab::decode(VOCAB_REFS).c_str());
            printf("	[get] [%s] <int> to read the reference torque for a single axis\n",                  Vocab::decode(VOCAB_REF).c_str());
            printf("	[get] [%s] to read the reference torque for all axes\n\n",                      Vocab::decode(VOCAB_REFS).c_str());

			printf("IImpedanceControl:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_IMPEDANCE).c_str());
            printf("	[set] [%s] <int> <float> <float> \n", 
                Vocab::decode(VOCAB_IMP_PARAM).c_str());
            printf("	[set] [%s] <int> <float>\n\n", 
                Vocab::decode(VOCAB_IMP_OFFSET).c_str());

            printf("	[get] [%s] <int>\n", 
                Vocab::decode(VOCAB_IMP_PARAM).c_str());
            printf("	[get] [%s] <int>\n\n", 
                Vocab::decode(VOCAB_IMP_OFFSET).c_str());

            printf("IInteractionMode:\ntype [%s] and one of the following:\n", Vocab::decode(VOCAB_INTERFACE_INTERACTION_MODE).c_str());
            printf("	[set] [%s]|[%s] <int>\n",
                    Vocab::decode(VOCAB_IM_STIFF).c_str(),
                    Vocab::decode(VOCAB_IM_COMPLIANT).c_str());

            printf("	[get] [%s] <int>\n",
                Vocab::decode(VOCAB_INTERACTION_MODE).c_str());
            printf("	[get] [%s] \n\n",
                Vocab::decode(VOCAB_INTERACTION_MODES).c_str());

			printf("Standard Interfaces:\n");
            printf("type [get] and one of the following:\n");
            printf("	[%s] to read the number of controlled axes\n", Vocab::decode(VOCAB_AXES).c_str());
            printf("	[%s] to read the encoder value for all axes\n", Vocab::decode(VOCAB_ENCODERS).c_str());
            printf("	[%s] to read the PID values for all axes\n", Vocab::decode(VOCAB_PIDS).c_str());
            printf("	[%s] <int> to read the PID values for a single axis\n", Vocab::decode(VOCAB_PID).c_str());
            printf("	[%s] <int> to read the limit values for a single axis\n", Vocab::decode(VOCAB_LIMITS).c_str());
            printf("	[%s] to read the PID error for all axes\n", Vocab::decode(VOCAB_ERRS).c_str());
            printf("	[%s] to read the PID output for all axes\n", Vocab::decode(VOCAB_OUTPUTS).c_str());
            printf("	[%s] to read the reference position for all axes\n", Vocab::decode(VOCAB_REFERENCES).c_str());
			printf("	[%s] <int> to read the reference position for a single axis\n", Vocab::decode(VOCAB_REFERENCE).c_str());
            printf("	[%s] to read the reference speed for all axes\n", Vocab::decode(VOCAB_REF_SPEEDS).c_str());
			printf("	[%s] <int> to read the reference speed for a single axis\n", Vocab::decode(VOCAB_REF_SPEED).c_str());
            printf("	[%s] to read the reference acceleration for all axes\n", Vocab::decode(VOCAB_REF_ACCELERATIONS).c_str());
			printf("	[%s] <int> to read the reference acceleration for a single axis\n", Vocab::decode(VOCAB_REF_ACCELERATION).c_str());
            printf("	[%s] to read the current consumption for all axes\n", Vocab::decode(VOCAB_AMP_CURRENTS).c_str());

            printf("\n");

            printf("type [set] and one of the following:\n");
            printf("	[%s] <int> <double> to move a single axis\n", Vocab::decode(VOCAB_POSITION_MOVE).c_str());
            printf("	[%s] <int> <double> to accelerate a single axis to a given speed\n", Vocab::decode(VOCAB_VELOCITY_MOVE).c_str());            
            printf("	[%s] <int> <double> to set the reference speed for a single axis\n", Vocab::decode(VOCAB_REF_SPEED).c_str());
            printf("	[%s] <int> <double> to set the reference acceleration for a single axis\n", Vocab::decode(VOCAB_REF_ACCELERATION).c_str());
            printf("	[%s] <list> to move multiple axes\n", Vocab::decode(VOCAB_POSITION_MOVES).c_str());
            printf("	[%s] <list> to accelerate multiple axes to a given speed\n", Vocab::decode(VOCAB_VELOCITY_MOVES).c_str());
            printf("	[%s] <list> to set the reference speed for all axes\n", Vocab::decode(VOCAB_REF_SPEEDS).c_str());
            printf("	[%s] <list> to set the reference acceleration for all axes\n", Vocab::decode(VOCAB_REF_ACCELERATIONS).c_str());          
            printf("	[%s] <int> to stop a single axis\n", Vocab::decode(VOCAB_STOP).c_str());
            printf("	[%s] <int> to stop all axes\n", Vocab::decode(VOCAB_STOPS).c_str());
            printf("	[%s] <int> <list> to set the PID values for a single axis\n", Vocab::decode(VOCAB_PID).c_str());
            printf("	[%s] <int> <list> to set the limits for a single axis\n", Vocab::decode(VOCAB_LIMITS).c_str());
            printf("	[%s] <int> to disable the PID control for a single axis\n", Vocab::decode(VOCAB_DISABLE).c_str());
            printf("	[%s] <int> to enable the PID control for a single axis\n", Vocab::decode(VOCAB_ENABLE).c_str());
            printf("	[%s] <int> <double> to set the encoder value for a single axis\n", Vocab::decode(VOCAB_ENCODER).c_str());
            printf("	[%s] <list> to set the encoder value for all axes\n", Vocab::decode(VOCAB_ENCODERS).c_str());
			printf("\n");
			printf("NOTES: - A list is a sequence of numbers in parenthesis, e.g. (10 2 1 10)\n");
			printf("       - Pids are expressed as a list of 7 numbers, type get pid <int> to see an example\n");
            printf("\n");
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

        case VOCAB_IMPEDANCE:
            {
                handleImpedanceMsg(iimp, p, response, &rec, &ok);
                printf("%s\n", response.toString().c_str());
                break;
            }

		case VOCAB_TORQUE:
			{
				handleTorqueMsg(itorque, p, response, &rec, &ok);
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
            switch(p.get(1).asVocab()) {
                case VOCAB_AXES: {
                    int nj = 0;
                    enc->getAxes(&nj);
                    printf ("%s: %d\n", Vocab::decode(VOCAB_AXES).c_str(), nj);
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

                case VOCAB_PID: {
                    Pid pd;
                    int j = p.get(2).asInt();
                    pid->getPid(j, &pd);
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
                    ok = pid->getPids(p);
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

                case VOCAB_ERRS: {
					pid->getErrors(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_ERRS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_OUTPUTS: {
                    iopenloop->getOutputs(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_OUTPUTS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");
                }
                break;

                case VOCAB_OUTPUT: {
                    int j = p.get(2).asInt();
                    double v;
                    iopenloop->getOutput(j, &v);
                    printf("%s: ", Vocab::decode(VOCAB_OUTPUT).c_str());
                    printf("%.2f ", v);
                    printf("\n");
                }
                break;

				case VOCAB_REFERENCE: {
					double ref_pos;
					int j = p.get(2).asInt();
                    pid->getReference(j,&ref_pos);
                    printf ("%s: (", Vocab::decode(VOCAB_REFERENCE).c_str());
                    printf ("%.2f ", ref_pos);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REFERENCES: {
                    pid->getReferences(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_REFERENCES).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REF_SPEEDS: {
                    pos->getRefSpeeds(tmp);
                    printf ("%s: (", Vocab::decode(VOCAB_REF_SPEEDS).c_str());
                    for(i = 0; i < jnts; i++)
                        printf ("%.2f ", tmp[i]);
                    printf (")\n");                    
                }
                break;

				case VOCAB_REF_SPEED: {
					double ref_speed;
					int j = p.get(2).asInt();
                    pos->getRefSpeed(j,&ref_speed);
                    printf ("%s: (", Vocab::decode(VOCAB_REF_SPEED).c_str());
                    printf ("%.2f ", ref_speed);
                    printf (")\n");                    
                }
                break;

				case VOCAB_REF_ACCELERATION: {
					double ref_acc;
					int j = p.get(2).asInt();
                    pos->getRefAcceleration(j,&ref_acc);
                    printf ("%s: (", Vocab::decode(VOCAB_REF_ACCELERATION).c_str());
                    printf ("%.2f ", ref_acc);
                    printf (")\n");                    
                }
                break;

                case VOCAB_REF_ACCELERATIONS: {
                    pos->getRefAccelerations(tmp);
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
            }
            break;

        case VOCAB_SET:
            switch(p.get(1).asVocab()) {
                case VOCAB_POSITION_MOVE: {
                    int j = p.get(2).asInt();
                    double ref = p.get(3).asDouble();
                    printf("%s: moving %d to %.2f\n", Vocab::decode(VOCAB_POSITION_MOVE).c_str(), j, ref);
                    pos->positionMove(j, ref);
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
                    pos->setRefSpeed(j, ref);
                }
                break;

                case VOCAB_REF_ACCELERATION: {
                    int j = p.get(2).asInt();
                    double ref = p.get(3).asDouble();
                    printf("%s: setting acceleration for %d to %.2f\n", Vocab::decode(VOCAB_REF_ACCELERATION).c_str(), j, ref);
                    pos->setRefAcceleration(j, ref);
                }
                break;

                case VOCAB_POSITION_MOVES: {
                    Bottle *l = p.get(2).asList();
                    for (i = 0; i < jnts; i++) {
                        tmp[i] = l->get(i).asDouble();
                    }
                    printf("%s: moving all joints\n", Vocab::decode(VOCAB_POSITION_MOVES).c_str());
                    pos->positionMove(tmp);
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
                    pos->setRefSpeeds(tmp);
                }
                break;

                case VOCAB_REF_ACCELERATIONS: {
                    Bottle *l = p.get(2).asList();
                    for (i = 0; i < jnts; i++) {
                        tmp[i] = l->get(i).asDouble();
                    }
                    printf("%s: setting acceleration for all joints\n", Vocab::decode(VOCAB_REF_ACCELERATIONS).c_str());
                    pos->setRefAccelerations(tmp);
                }
                break;

                case VOCAB_STOP: {
                    int j = p.get(2).asInt();
                    printf("%s: stopping axis %d\n", Vocab::decode(VOCAB_STOP).c_str(), j);
                    pos->stop(j);
                }
                break;

                case VOCAB_STOPS: {
                    printf("%s: stopping all axes\n", Vocab::decode(VOCAB_STOPS).c_str());
                    pos->stop();
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
                                    pid->setPid(j, pd);
                                }
                            else
                                {
                                    printf("Error, check you specify at least 7 elements, e.g. set pid 0 (2000 20 1 300 300 0 0)\n");
                                }
                        }
                }
                break;

                case VOCAB_DISABLE: {
                    int j = p.get(2).asInt();
                    printf("%s: disabling control for axis %d\n", Vocab::decode(VOCAB_DISABLE).c_str(), j);
                    pid->disablePid(j);
                    amp->disableAmp(j);
                }
                break;

                case VOCAB_ENABLE: {
                    int j = p.get(2).asInt();
                    printf("%s: enabling control for axis %d\n", Vocab::decode(VOCAB_ENABLE).c_str(), j);
                    amp->enableAmp(j);
                    pid->enablePid(j);
                }
                break;

                case VOCAB_LIMITS: {
                    int j = p.get(2).asInt();
                    printf("%s: setting limits for axis %d\n", Vocab::decode(VOCAB_LIMITS).c_str(), j);
                    Bottle *l = p.get(3).asList();
                    lim->setLimits(j, l->get(0).asDouble(), l->get(1).asDouble());
                }
                break;

                case VOCAB_OUTPUT: {
                    int j=p.get(2).asInt();
                    double v=p.get(3).asDouble();
                    iopenloop->setRefOutput(j,v);
                    printf("%s: setting output for axis %d to %f\n", Vocab::decode(VOCAB_OUTPUT).c_str(), j, v);            
                }
                break;
            }
            break;
        } /* switch get(0) */

    } /* while () */

ApplicationCleanQuit:
    dd.close();
    delete[] tmp;

    Network::fini();
    return 0;
}

void handleTorqueMsg(ITorqueControl *torque, const yarp::os::Bottle& cmd,
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

					case VOCAB_LIM: 
                        {
                            *ok = torque->setTorqueErrorLimit (cmd.get(3).asInt(), cmd.get(4).asDouble());
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
                                    *ok = torque->setTorqueErrorLimits (p);
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
                            *ok = torque->setTorquePid(j, p);
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
                                    *ok = torque->setTorquePids(p);
                                    delete[] p;
                                }
                        }
                        break;

					case VOCAB_RESET: 
						{
							*ok = torque->resetTorquePid (cmd.get(3).asInt());
						}
                        break;

					case VOCAB_DISABLE:
						{
							*ok = torque->disableTorquePid (cmd.get(3).asInt());              
						}
                        break;

					case VOCAB_ENABLE: 
						{
							*ok = torque->enableTorquePid (cmd.get(3).asInt());                   
						}
                        break;

					case VOCAB_TORQUE_MODE: 
                        {
                            *ok = torque->setTorqueMode();
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
							*ok = torque->getTorqueError(cmd.get(3).asInt(), &dtmp);
							response.addDouble(dtmp);
						}
						break;

					case VOCAB_ERRS: 
						{
							double *p = new double[controlledJoints];
							*ok = torque->getTorqueErrors(p);
							Bottle& b = response.addList();
							int i;
							for (i = 0; i < controlledJoints; i++)
								b.addDouble(p[i]);
							delete[] p;
						}
						break;

					case VOCAB_OUTPUT: 
						{
							*ok = torque->getTorquePidOutput(cmd.get(3).asInt(), &dtmp);
							response.addDouble(dtmp);
						}
						break;

					case VOCAB_OUTPUTS: 
						{
							double *p = new double[controlledJoints];
							*ok = torque->getTorquePidOutputs(p);
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
							*ok = torque->getTorquePid(cmd.get(3).asInt(), &p);
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
							*ok = torque->getTorquePids(p);
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
							*ok = torque->getTorqueErrorLimit(cmd.get(3).asInt(), &dtmp);
							response.addDouble(dtmp);
						}
						break;

					case VOCAB_LIMS: 
						{
							double *p = new double[controlledJoints];
							*ok = torque->getTorqueErrorLimits(p);
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


void handleControlModeMsg(IControlMode2 *iMode, const yarp::os::Bottle& cmd,
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
                        *ok = iMode->setPositionMode(axis);
						break;
                    case VOCAB_CM_POSITION_DIRECT:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_POSITION_DIRECT);
						break;
                    case VOCAB_CM_VELOCITY:
                        *ok = iMode->setVelocityMode(axis);
						break;
                    case VOCAB_CM_MIXED:
                        *ok = iMode->setControlMode(axis, VOCAB_CM_MIXED);
                        break;
                    case VOCAB_CM_TORQUE:
                        *ok = iMode->setTorqueMode(axis);
                        break;
                    case VOCAB_CM_OPENLOOP:
                        *ok = iMode->setOpenLoopMode(axis);
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
