// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_module
 *
 * \defgroup icub_simple_headcontrol simple_headcontrol
 *
 * A basic controller for the robot head.
 * Call with no arguments for usage information.
 *
 * \author Giorgio Metta
 *
 */

#include <yarp/os/PortablePair.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Terminator.h>
#include <yarp/os/Os.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <yarp/sig/Vector.h>

#include <string>
#include <string.h> //memcpy and the like

#include "pidfilter.h"

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

#define VOCAB_HELP VOCAB4('h','e','l','p')
#define VOCAB_QUIT VOCAB4('q','u','i','t')

//
int main(int argc, char *argv[]) 
{
    Network::init();
	Time::turboBoost();

    Terminee terminee("/simpleHeadControl/quit");
    if (!terminee.isOk()) {
        printf("Failed to create proper quit socket\n");
        return 1;
    }

	const char *conf = yarp::os::getenv("ICUB_ROOT");
    
	if (conf == NULL) {
		printf("This application requires the ICUB_ROOT environment variable\n");
		printf("defined to point at the configuration files under $ICUB_ROOT/conf\n");
		return 0;
	}

    // just list the devices if no argument given
    if (argc <= 2) {
        printf("You can call %s like this:\n", argv[0]);
        printf("   %s --robot ROBOTNAME --OPTION VALUE ...\n", argv[0]);
        printf("For example:\n");
        printf("   %s --robot icub\n", argv[0]);
        return 0;
    }

    // get command line options
    Property options;
    options.fromCommand(argc, argv);
    if (!options.check("robot")) {
        printf("Missing the --robot option\n");
        return 0;
    }

	if (!options.check("device")) {
		printf("Adding device remote_controlboard\n");
		options.put("device", "remote_controlboard");
	}

    Value& robotname = options.find("robot");

	std::string s("/");
	s += robotname.asString().c_str();
	s += "/head/control";
	options.put("local", s.c_str());

	s.clear();
	s += "/";
	s += robotname.asString().c_str();
	s += "/head";
	options.put("remote", s.c_str());

	std::string filename;
	filename += conf;
	filename += "/conf/";
	filename += robotname.asString().c_str();
	filename += "_simpleheadcontrol.ini";

	Property p;
	printf("Head controller working with config file %s\n", filename.c_str());
	p.fromConfigFile(filename.c_str());
    
    fprintf(stderr, "%s", p.toString().c_str());

    bool verbose = options.check("verbose") || p.findGroup("GENERAL").find("Verbose").asInt() != 0;

    // receive the reference value for the PID controller.
    BufferedPort<yarp::sig::Vector> targetLeft;  
    BufferedPort<yarp::sig::Vector> targetRight;

	std::string name;
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/right/target";
    targetRight.open(name.c_str());
	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/left/target";
    targetLeft.open(name.c_str());

	Port inertial;
	PortReaderBuffer<yarp::sig::Vector> inertial_buffer;
	inertial_buffer.attach(inertial);

	name.clear();
	name += "/";
	name += robotname.asString().c_str();
	name += "/inertial_data";
    inertial.open(name.c_str());

    // create a device to connect to the remote robot server.
    PolyDriver dd(options);
    if (!dd.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 1;
    }

    IPositionControl *pos;
    IVelocityControl *vel;
    IEncoders *enc;
    IPidControl *pid;
    IAmplifierControl *amp;
    IControlLimits *lim;

    bool ok;
    ok = dd.view(pos);
    ok &= dd.view(vel);
    ok &= dd.view(enc);
    ok &= dd.view(pid);
    ok &= dd.view(amp);
    ok &= dd.view(lim);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 1;
    }

	std::string name_src("/");
	name_src += robotname.asString().c_str();
	name_src += "/inertial";

	Network::connect(name_src.c_str(), name.c_str(), "udp");

    int jnts = 0;
    pos->getAxes(&jnts);
    printf("Working with %d axes\n", jnts);

    double *tmp = new double[jnts];
	double *prev_control = new double[jnts];
	double *q = new double[jnts];
	memset(q, 0, sizeof(double)*jnts);
	memset(tmp, 0, sizeof(double)*jnts);
	memset(prev_control, 0, sizeof(double)*jnts);

    printf("Device active...\n");
    printf("Setting up default configuration for the robot head\n");

    // setting up the controller parameters for velocity control.
    int i;
    for (i = 0; i < jnts; i++) {
        tmp[i] = 100.0;
    }
    pos->setRefAccelerations(tmp);

	for (i = 0; i < jnts; i++) {
		amp->enableAmp(i);
		pid->enablePid(i);
	}

    bool quit = false;
    double before, now;
    int period = 20;

	Bottle& tmp_b = p.findGroup("PIDS").findGroup("Pan");
	if (verbose)
		printf("%s\n", tmp_b.toString().c_str());
    
    // Beware, strange things might happen to those who use the PidFilter class.
    // Why should you use a PidFilter class when all you need is as simple as a double?
    // PidFilter pan(tmp_b.get(1).asDouble(), tmp_b.get(2).asDouble(), tmp_b.get(3).asDouble(), tmp_b.get(4).asDouble());
    double Kpan=tmp_b.get(1).asDouble();
    	
	tmp_b = p.findGroup("PIDS").findGroup("Tilt");
	if (verbose)
		printf("%s\n", tmp_b.toString().c_str());

    // Beware, strange things might happen to those who use the PidFilter class.
    // Why should you use a PidFilter class when all you need is as simple as a double?
	// PidFilter tilt(tmp_b.get(1).asDouble(), tmp_b.get(2).asDouble(), tmp_b.get(3).asDouble(), tmp_b.get(4).asDouble());
    double Ktilt=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("Vergence");
	if (verbose)
		printf("%s\n", tmp_b.toString().c_str());
    double Kvergence=tmp_b.get(1).asDouble();
	
	tmp_b = p.findGroup("PIDS").findGroup("NeckPan");
	if (verbose)
		printf("%s\n", tmp_b.toString().c_str());
	//PidFilter neckpan(tmp_b.get(1).asDouble(), tmp_b.get(2).asDouble(), tmp_b.get(3).asDouble(), tmp_b.get(4).asDouble());
    double Kn_pan=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("NeckTilt");
	if (verbose)
		printf("%s\n", tmp_b.toString().c_str());
    //	PidFilter necktilt(tmp_b.get(1).asDouble(), tmp_b.get(2).asDouble(), tmp_b.get(3).asDouble(), tmp_b.get(4).asDouble());
    double Kn_tilt=tmp_b.get(1).asDouble();

	tmp_b = p.findGroup("PIDS").findGroup("VOR");
	if (verbose)
		printf("%s\n", tmp_b.toString().c_str());
    double vorx = tmp_b.get(1).asDouble();
	double vory = tmp_b.get(2).asDouble();
	double vorz = tmp_b.get(3).asDouble();

    Vector *vr=0;
    Vector *vl=0;
    Vector *v=0; //backwad cmp

    while (!terminee.mustQuit()) 
        {
            before = Time::now();
            vr = targetRight.read(0);
            vl = targetLeft.read(0);

            if (vr != 0) 
                {
                    v=vr;
                    Vector *gyro = 0;
                    if (inertial_buffer.check()) 
                        gyro = inertial_buffer.read(0);
				
                    if (verbose && gyro != 0 && gyro->size() != 0) 
                        {
                            int k;
                            for (k = 6; k < 9; k++) {
                                printf("%.2f ", (*gyro)[k]);
                            }
                            printf("\n");
                        }


                    // compute PID for tracking.
                    enc->getEncoders(q);

                    //vvv std
                    memset(tmp, 0, sizeof(double)*jnts);
                    //                	(*v)[1] = (*v)[1] / (*v)[5] - 0.5;
                    //                	(*v)[2] = (*v)[2] / (*v)[6] - 0.5;
                    //                (*v)[1]=-0.25*(*v)[1];
                    //                (*v)[2]=0;//-0.25*(*v)[2];
                    
                    //tmp[5] = pan.pid((*v)[1]);
                    //tmp[3] = tilt.pid((*v)[2]);

                    // vvv ref frame
                    // tmp[5]=Kpan*(*v)[1];
                    // tmp[3]=Ktilt*(*v)[2];
                    
                    //// +1/-1 standard
                    tmp[4]=Kpan*(*vr)[0];
                    tmp[3]=Ktilt*(*vr)[1];

                    if (vl!=0)
                        {
                            double d=(*vr)[0]-(*vl)[0];
                            tmp[5]=Kvergence*d;
                            // fprintf(stderr, "%.3lf %.3lf %.3lf\n", Kvergence, d, tmp[4]);
                        }
                    else
                        tmp[5]=0;

                    if (gyro != 0 && gyro->size() != 0)	
                        {
                            tmp[1] = vorx*(*gyro)[0]; //roll
                            tmp[3] -= ((*gyro)[6] * 180.0 / 3.141528 * vory); //tilt
                            tmp[4] -= ((*gyro)[7] * 180.0 / 3.141528 * vorz); //pan
                        }
                    else
                        tmp[1]=0.0; //roll
                    
                    tmp[2] = Kn_pan*(q[4]);
                    tmp[0] = Kn_tilt*(q[3]);
                    // tmp[1] = 0.0;

                    bool ok = vel->velocityMove(tmp);
                    if (!ok)
                        printf("Something wrong with the velocityMove command\n");
                    
                    memcpy(prev_control, tmp, sizeof(double)*jnts);
                }
            else {
                // slow down when signal is not missing.
                bool ok = vel->velocityMove(prev_control);
                if (!ok)
                    printf("Something wrong with the velocityMove command\n");
                
                int i;
                for (i = 0; i < jnts; i++)
                    prev_control[i] /= 2;			
            }

            now = Time::now();
            if ((now-before)*1000 < period) {
                const double k = double(period)/1000.0-(now-before);
                Time::delay(k);
            }
            else {
                printf("Can't comply with the %d ms period (%.2f)\n", period, (now-before)*1000);
            }
        }

    for (i = 0; i < jnts; i++) {
        tmp[i] = 0.0;
    }
    vel->velocityMove(tmp);
    Time::delay(0.5);

    dd.close();
    delete[] tmp;
    delete[] prev_control;
    
    Network::fini();
    return 0;
}
