#include <ace/OS.h>
//#include <ace/Log_Msg.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <RFNet.h>
#include <symbolicModel.h>
#include <math.h>
#include <iostream>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

const double	P_GAIN=0.008;
const double	TMP_GAIN=10;
const int		NC_RATE=10;
const double	IN_POSITION_THRESHOLD = 0.1;
const double	NEAR_POSITION_THRESHOLD = 0.5;
const double	REL_WEIGHT = 0.9;					//WARNING: has to be set between 0 and 1!!
const double    P_CONV = 0.0383;
const double    R_CONV = 0.0317;


 
class NeuralNeckControl: public RateThread
{
private:
    PolyDriver *ddInertia;
    PolyDriver *ddHead;
    
    IGenericSensor *isensor;
    IVelocityControl *ivel;
    IPidControl *ipid;
    IEncoders *iencs;
    IAmplifierControl *iamps;
	IControlCalibration *icalib;
	IPositionControl *ipos;

	RFNet net;
    
public:
    NeuralNeckControl(int r, PolyDriver *inertia, PolyDriver *head, const char* filename): RateThread(r),
        ddInertia(0),
        ddHead(0),
        inertiaValue(3),
        encoders(0),
        nAxes(0)
    {
        pGain=P_GAIN;
		roll_conv=R_CONV;
		pitch_conv=P_CONV;

        if (inertia!=0)
            ddInertia=inertia;
        if (head!=0)
            ddHead=head;

        vCmds=new double [4];

		net.LoadNet(filename);

		netIn.Resize(5);
		netOut.Resize(3);

    }
    
    ~NeuralNeckControl()
    {
        delete [] vCmds;
    }
    
    bool start()
    {
        bool ok;
        if (ddInertia==0)
            return false;
        if (ddHead==0)
            return false;

        ok=ddInertia->view(isensor);
        if (!ok)
        {
            ACE_OS::printf("NeuralNeckControl::Error getting IGenericSensor interface, returning false\n");
            return false;
        }

        // calibration
        isensor->calibrate(0,0);
 
        ok=ddHead->view(ivel);
        ok=ok && ddHead->view(iencs);
        ok=ok && ddHead->view(iamps);
        ok=ok && ddHead->view(ipid);
		ok=ok && ddHead->view(icalib);
		ok=ok && ddHead->view(ipos);
        
		if (!ok)
        {
            ACE_OS::printf("NeuralNeckControl::Error getting head interfaces, returning false\n");
            return false;
        }

        iencs->getAxes(&nAxes);
        encoders=new double [nAxes];
		return RateThread::start();
    }

    void setRef(double r, double p)
    {
        roll_d=r;
        pitch_d=p;
    }

	void enableControl()
	{
		ipid->enablePid(4);
		ipid->enablePid(5);
        ipid->enablePid(6);
        ipid->enablePid(7);

		iamps->enableAmp(4);
        iamps->enableAmp(5);
        iamps->enableAmp(6);
        iamps->enableAmp(7);
	}

	void disableControl()
	{

		ipid->disablePid(4);
        ipid->disablePid(5);
        ipid->disablePid(6);
        ipid->disablePid(7);

		iamps->disableAmp(4);
        iamps->disableAmp(5);
        iamps->disableAmp(6);
        iamps->disableAmp(7);
	}

    bool threadInit()
    {
        ivel->setRefAcceleration(5, 1000);
        ivel->setRefAcceleration(6, 1000);
        ivel->setRefAcceleration(7, 1000);

		enableControl();

		ACE_OS::printf("NeckCollector::Starting calibration sequence\n");
		
		yawCalibrate();
		rollPitchCalibrate();
	
        return true;
    }

	void yawCalibrate()
	{
		double pos;
		bool done;

		ACE_OS::printf("Calibrating yaw\n");

		icalib->calibrate(4, -200);   // giving a constant voltage (torque) to motor 4 (head yaw)

		while(!icalib->done(4))
			ACE_OS::printf(".");

		ACE_OS::printf("\nMoving to the center:");

		iencs->getEncoder(4, &pos);
		ipos->setRefSpeed(4,20);
		ipos->positionMove(4, 90);

		done = false;
		while(!done)
		{
			if (ipos->checkMotionDone(4, &done))
				ACE_OS::printf(".");
			else
				ACE_OS::printf("CheckMotionDone returned false\n");
		}
		
		ACE_OS::printf("\nYaw calibration terminated\n");

		ipid->disablePid(4);
		iamps->disableAmp(4);
 
	}



	void rollPitchCalibrate()
	{
		ACE_OS::printf("Calibrating roll and pitch\n");

		bool inPosition = false;
		int k;

		setRef(0.0, 0.0);

		ACE_OS::printf("Trying to reach zero configuration: ");
		while(!inPosition)
		{
			ACE_OS::printf(".");

			isensor->read(inertiaValue);

			roll=inertiaValue[0];
			pitch=inertiaValue[1];

	        vCmds[0]=-600*(roll-roll_d);                     //here a model-based Jacobian is used
		    vCmds[1]=500*(pitch-pitch_d)+400*(roll-roll_d);
			vCmds[2]=-500*(pitch-pitch_d)+400*(roll-roll_d);
 
	        vCmds[0]*=pGain;
		    vCmds[1]*=pGain;
			vCmds[2]*=pGain;

			if (fabs(roll - roll_d) < IN_POSITION_THRESHOLD)   //a little position error is allowed
			{
				if (fabs(pitch - pitch_d) < IN_POSITION_THRESHOLD)
				{
					ACE_OS::printf("\nZero Configuration Reached\n");

					inPosition = true;

			        vCmds[0]=0;
				    vCmds[1]=0;
					vCmds[2]=0;
				}
			}


			ivel->velocityMove(5, vCmds[0]);
			ivel->velocityMove(6, vCmds[1]);
			ivel->velocityMove(7, vCmds[2]);
		}

		double pos;
		bool done;

		ACE_OS::printf("Pulling the cables:");
		icalib->calibrate(5, 400);
		icalib->calibrate(6, 400);
		icalib->calibrate(7, 400);

		for (k = 5; k <8; k++)
		{
			while(!icalib->done(k))
				ACE_OS::printf(".");
		}

		ACE_OS::printf("\nReleasing cable:");
		for (k = 5; k <8; k++)
		{
			iencs->getEncoder(k, &pos);
			ipos->setRefSpeed(k,5);
			ipos->positionMove(k, pos-5);
		}

		for (k = 5; k <8; k++)
		{
			done = false;
			while(!done)
			{
				if (ipos->checkMotionDone(k, &done))
					ACE_OS::printf(".");
				else
					ACE_OS::printf("CheckMotionDone returned false\n");
			}
		}
		
		ACE_OS::printf("\n");

	    disableControl();

		iencs->resetEncoder(5);
		iencs->resetEncoder(6);
		iencs->resetEncoder(7);

		enableControl();
	}


    void run()
    {
		double t1=Time::now();
        static int count=1;
		double d1;				//length[cm] of the cable attached to joint 7
		double d2;				//length[cm] of the cable attached to joint 6
		double d3;				//length[cm] of the cable attached to joint 5
        double yaw;

        isensor->read(inertiaValue);

        iencs->getEncoders(encoders);

        roll=inertiaValue[0];
        pitch=inertiaValue[1];
		yaw = encoders[4];

		//accounts for the fact that the base of the neck
		//is not affected by the yaw (pan of the head)
		//while the sensor is affected.
		double pitch_hat;
		double roll_hat;
		double pitch_d_hat;
		double roll_d_hat;

		computeModifiedPitchRoll(yaw-90, roll, pitch, roll_hat, pitch_hat);
		computeModifiedPitchRoll(yaw-90, roll_d, pitch_d, roll_d_hat, pitch_d_hat);
		computeTendonsLength(d1, d2, d3, roll_hat, pitch_hat);

		netIn[0]=encoders[5];
		netIn[1]=encoders[6];
		netIn[2]=encoders[7];

		netIn[3]=roll_conv*(roll_d_hat-roll_hat);
		netIn[4]=pitch_conv*(pitch_d_hat-pitch_hat);

		net.Simulate(netIn, 0.001, netOut);

		vCmds[0]=netOut[0];
		vCmds[1]=netOut[1];
		vCmds[2]=netOut[2];

        vCmds[0]*=TMP_GAIN;
        vCmds[1]*=TMP_GAIN;
        vCmds[2]*=TMP_GAIN;

        ivel->velocityMove(5, vCmds[0]);
        ivel->velocityMove(6, vCmds[1]);
        ivel->velocityMove(7, vCmds[2]);

        double t2=Time::now();
        
        count++;
        static double dT=0;
        dT+=t2-t1;
        
        if (count%100==0)
        {
            fprintf(stderr, "%.2lf %.2lf %.2lf ", (encoders[5]-d3), (encoders[6]-d2), (encoders[7]-d1));
            //fprintf(stderr, "%.2lf %.2lf %.2lf ", d3, d2, d1);
            fprintf(stderr, "Inertial: %.2lf %.2lf ", roll, pitch);
			//fprintf(stderr, "Inertial: %.2lf %.2lf", roll, pitch);
            fprintf(stderr, "dT=%.3lf ", (dT)/100);
			fprintf(stderr, "NetIn= %.2lf %.2lf ", netIn[3], netIn[4]);
			fprintf(stderr, "NetOut= %.2lf %.2lf %.2lf \n", netOut[0], netOut[1], netOut[2] );
            dT=0;
        }
    }

    void threadRelease()
    {
        ivel->velocityMove(5, 0);
        ivel->velocityMove(6, 0);
        ivel->velocityMove(7, 0);

		disableControl();

		delete [] encoders;
        encoders=0;
    }
    
    double roll;
    double pitch;
    double roll_d;
    double pitch_d;
    Vector inertiaValue;
    double *encoders;
    double *vCmds;
    double pGain;
    int nAxes;
	YVector netOut;
	YVector netIn;
	double roll_conv;
	double pitch_conv;
};

int main(int argc, const char **argv) //argv[1] is the name of the file containing the Net to be used for control
{
	Network::init();

	if (argc==1) 
	{
		ACE_OS::printf("A file name must be provided as argument... The file contains informations about the Net you want to use for neck control\n");
		Network::fini();
        return 0;
	}

    ////////////////////////
    Property inertiaOptions;
    inertiaOptions.fromConfig("[GENERAL]\nComPort 0\n");
    inertiaOptions.put("device", "jamesinertiacube");
    PolyDriver inertia_dd(inertiaOptions);

    if (!inertia_dd.isValid())
    {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }
    
    ////////////////////////////////
    Property headOptions;
	headOptions.fromConfigFile("Y:/iCub-james/conf/james_head.ini");
	headOptions.put("device", "controlboard");
	headOptions.put("subdevice", "jamesesd");
   
    // create a device
    PolyDriver head_dd(headOptions);
    if (!head_dd.isValid()) {
        ACE_OS::printf("Device not available.  Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
        return 0;
    }

    ///////////////////////////////////////////////////
	ACE_OS::printf("\nCreating NeuralNeckControl...\n");
	///////////////////////////////////////////////////
	
	NeuralNeckControl *neckControl=new NeuralNeckControl(NC_RATE, &inertia_dd, &head_dd, argv[1]);
    
	ACE_OS::printf("\nNeuralNeckControl ready...\nStarting main thread...\n");

    if (!neckControl->start())
    {
        ACE_OS::printf("Error starting neck control, returning\n");
        Network::fini();
        return 0;
    }

    ACE_OS::printf("NeuralNeckControl thread running\n");
    ACE_OS::printf("Type q + [Enter] to quit the application\n");
	ACE_OS::printf("Type m + [Enter] to change reference values of roll and pitch\n");
    char c = 'a';
    double pitch;
    double roll;
    while (c != 'q')
	{
		std::cin>>c;
        if (c == 'q')
            break;
        else if (c=='m')
        {
            ACE_OS::printf("Type roll value:\n");
			std::cin>>roll;
			ACE_OS::printf("Type pitch value:\n");
            std::cin>>pitch;
                
			if (fabs(roll)<40 && fabs(pitch)<40)
			{
				ACE_OS::printf("Saving new values for desired roll and pitch\n");
				neckControl->setRef(roll, pitch);
				ACE_OS::printf("New roll: %lf pitch:%lf\n", neckControl->roll_d, neckControl->pitch_d);
			}
        }
	}

    neckControl->stop();

    delete neckControl;

    Network::fini();
    return 0;
}
