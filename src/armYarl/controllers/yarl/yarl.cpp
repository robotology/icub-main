#include <math.h>
#include <string>

#include <kdl/kinfam/serialchain.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <ace/Sched_Params.h>

#include <yarp/os/ConnectionReader.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <yarp/String.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace KDL;

const int TRIAL_MAX = 20;
const int LEARN_PARAM_NUMOF = 4;
const double LEARN_RATE = 1.0;

const double PROX_MAX = 0.02; // If closer than this, the target is reached
const int CORR_NO_MAX = 4; // The maximum number of corrective pulses

const int INIT_PW_MSEC = 3000; // Pulse widt for the initial pulse
const int CORR_PW_MSEC = 1000; // Pulse with for the correction pulses

// Target coordinates
const int TARGS_NUMOF = 3;
const double TARG_COORDS[TARGS_NUMOF][3] = {{  0.01, 0.66, -0.24 },
                                            { -0.15, 0.45, -0.14 },
                                            { -0.22, 0.93, -0.07}};

const int YARP_JOINTS_NUMOF = 7;
const int JOINTS_NUMOF = 4;
// Joint positions for the starting point
const double INIT_JOINT_POS[JOINTS_NUMOF] = {0.0, 0.0, 0.0, 0.0};

// Conversion factors from cartesian displacement to motor speed
// Based on the scientific estimate of 'seems to work'
const double DISP_CONV = 120.0;

bool webotstime = false;
int trial_max = TRIAL_MAX;
int targcount = 0;

BufferedPort<Bottle> timeport;

PolyDriver* ddlarm;
IAmplifierControl* larmamp; // For enabling the motors
IEncoders* larmenc;
IPositionControl* larmpos;
IVelocityControl* larmvel;

SerialChain* iw;

void init_yarp();
void init_kinematics(void);
void set_arm_pos(IPositionControl* p, double* pos=(double*)INIT_JOINT_POS);
void calc_pulse(double *pulse,double *handpos, bool expl=true);
void exec_pulse(IVelocityControl* v,double *pulse, int pw=CORR_PW_MSEC);
void my_sleep(int msecs=1000);
void calc_handpos(IEncoders* e,double* hp);
void calc_disp(double* handpos, double* disp);
bool targ_reached(double *handpos);

int main(int argc,char **argv) 
{
    double* disp;
    double handpos[3];
    double displacement[4];
    double initpulse[TARGS_NUMOF][LEARN_PARAM_NUMOF+1]; // Speeds to be learnt
    double corrpulse[LEARN_PARAM_NUMOF]; // Motor speeds corrective pulse
	double accorrpulse[LEARN_PARAM_NUMOF];

    for(int i=0;i<argc;i++){
        if(!strcmp(argv[i],"-wt")){
            webotstime = true;
            ACE_OS::printf("Using webots time.\n");
        }
        if(!strcmp(argv[i],"-trials") && argc>i+1){
            trial_max=atoi(argv[i+1]);
        }
    }
    if(!webotstime)
        ACE_OS::printf("Using real time.\n");
    ACE_OS::printf("Running %d trials.\n",trial_max);
    double* disps[trial_max];

    // Initialise
    Network::init();
    init_yarp();
    init_kinematics();
    ACE_OS::printf("Initialisation done.\n");

    // Learn to reach each of the targets
    for(targcount=0;targcount<TARGS_NUMOF;targcount++){

        ACE_OS::printf("\nLearning target %d.\n",targcount);

        // Set up first init pulse as corretion
	    // Move hand to start position
        set_arm_pos(larmpos);
        ACE_OS::printf("Hand in start position.\n");
        // Calculate hand position
        calc_handpos(larmenc,handpos);

        // Calculate first init pulse
        calc_pulse(initpulse[targcount],handpos,false); // No explore init pulse
        initpulse[targcount][2] = -20.0;
        initpulse[targcount][3] = 10.0;
        ACE_OS::printf("Set first init pulse:");
        for(int i=0;i<JOINTS_NUMOF;i++)
            ACE_OS::printf(" %f",initpulse[targcount][i]);
        ACE_OS::printf("\n");

        int trialcount = 0;
        int corrcount;
        // Reapeat reaching in order to learn
        do{
    
            // Execute init pulse
            ACE_OS::printf("\nInitial pulse trial %d \n",trialcount+1);
            exec_pulse(larmvel,initpulse[targcount],INIT_PW_MSEC);

            // Calculate hand position
            calc_handpos(larmenc,handpos);
            calc_disp(handpos,displacement);
            ACE_OS::printf("Displacement");
            for(int i=0;i<4;i++)
                printf(" %f",displacement[i]);
            ACE_OS::printf("\n");
            bool targreach = targ_reached(handpos);
            if(targreach)
                ACE_OS::printf("Target reached!\n");
            else
                ACE_OS::printf("Did not reach target...\n");

            for(int i=0;i<LEARN_PARAM_NUMOF;i++)
                accorrpulse[i] = 0.0;
            corrcount = 0;
	        // do until hand is at target and the trial has failed
	        while(!targreach && corrcount < CORR_NO_MAX){

                ACE_OS::printf("\nCorrection pulse %d:\n",corrcount);
		        // Calculate pulse
                calc_pulse(corrpulse,handpos);

		        // Accumulate corrections
                for(int i=0;i<LEARN_PARAM_NUMOF;i++)
		            accorrpulse[i] += corrpulse[i];
	
		        // Execute pulse
                my_sleep();
                exec_pulse(larmvel,corrpulse);

                // Calculate hand position
                calc_disp(handpos,displacement);
                ACE_OS::printf("Displacement");
                for(int i=0;i<4;i++)
                    printf(" %f",displacement[i]);
                ACE_OS::printf("\n");
                calc_handpos(larmenc,handpos);
                targreach = targ_reached(handpos);
                if(targreach)
                    ACE_OS::printf("Target reached!\n");
                else
                    ACE_OS::printf("Did not reach target...\n");

		        corrcount++;
	        }

            double* disp = new double[5];
            calc_disp(handpos,disp);
            disp[4] = corrcount;
            disps[trialcount] = disp;
            ACE_OS::printf("\nHistorical displacements target %d:\n",targcount);
            for(int i=0;i<trialcount+1;i++){
                ACE_OS::printf("%d: ",i);
                for(int j=0;j<5;j++)
                    ACE_OS::printf("%f ",disps[i][j]);
                ACE_OS::printf("\n");
            }
            ACE_OS::printf("\n");
            //double learnrate = disp[3]/LEARN_DISP;
            double learnrate = LEARN_RATE;

            // Update pulse speeds
            double corrtime,corrinitratio;
            double avgcorrsp[JOINTS_NUMOF],newsp[JOINTS_NUMOF];
            double diffsp[JOINTS_NUMOF];
            if(corrcount > 0){
                ACE_OS::printf("Old speeds ");
                for(int i=0;i<JOINTS_NUMOF;i++)
                    ACE_OS::printf(" %f",initpulse[targcount][i]);
                ACE_OS::printf("\n");
                ACE_OS::printf("Accumulated corrections ");
                for(int i=0;i<JOINTS_NUMOF;i++)
                    ACE_OS::printf(" %f",accorrpulse[i]);
                ACE_OS::printf("\n");
                for(int i=0;i<JOINTS_NUMOF;i++){
                    avgcorrsp[i] = accorrpulse[i]/(float)corrcount; 
                    corrtime = (double)corrcount*(double)CORR_PW_MSEC;
                    corrinitratio = corrtime/(double)INIT_PW_MSEC;
                    newsp[i] = initpulse[targcount][i] + avgcorrsp[i]*corrinitratio;
                    diffsp[i] = newsp[i] - initpulse[targcount][i];
                    initpulse[targcount][i] += learnrate*diffsp[i];
                }
                ACE_OS::printf("Avg correction speeds ");
                for(int i=0;i<JOINTS_NUMOF;i++)
                    ACE_OS::printf(" %f",avgcorrsp[i]);
                ACE_OS::printf("\n");
                ACE_OS::printf("New speeds ");
                for(int i=0;i<JOINTS_NUMOF;i++)
                    ACE_OS::printf(" %f",newsp[i]);
                ACE_OS::printf("\n");
                ACE_OS::printf("Speed diffs ");
                for(int i=0;i<JOINTS_NUMOF;i++)
                    ACE_OS::printf(" %f",diffsp[i]);
                ACE_OS::printf("\n");
                ACE_OS::printf("Updated pulse ");
                for(int i=0;i<JOINTS_NUMOF;i++)
                    ACE_OS::printf(" %f",initpulse[targcount][i]);
                ACE_OS::printf("\n");
            }else{
                ACE_OS::printf("\n");
                ACE_OS::printf("Unadjusted pulse speeds:");
                for(int i=0;i<JOINTS_NUMOF;i++)
                    ACE_OS::printf(" %f",initpulse[targcount][i]);
                ACE_OS::printf("\n");
            }

	        // Move hand to start position
            set_arm_pos(larmpos);

            trialcount++;

        }while(corrcount > 0 && trialcount < trial_max); // End of trials

        if(corrcount == 0){
            ACE_OS::printf("\nLearned target %d!\n",targcount); 
            initpulse[targcount][4] = 1.0;
        }else{
            ACE_OS::printf("\nFailed to learn target  %d!\n",targcount); 
            initpulse[targcount][4] = 0.0;
        }
        ACE_OS::printf("\n");
        ACE_OS::printf("Final init pulse speeds target %d:",targcount);
        for(int i=0;i<JOINTS_NUMOF;i++)
            ACE_OS::printf(" %f",initpulse[targcount][i]);
        ACE_OS::printf("\n");

        ACE_OS::printf("\n");
        ACE_OS::printf("Learnt targets:\n");
        ACE_OS::printf("%d:");
        for(int i=0;i<targcount+1;i++){
            for(int j=0;j<JOINTS_NUMOF+1;j++)
                ACE_OS::printf(" %f",initpulse[i][j]);
            ACE_OS::printf("\n");
        }

    } // End of targets

    // Show off
    for(targcount=0;targcount<TARGS_NUMOF;targcount++){
	        
        // Move hand to start position
        my_sleep();
        set_arm_pos(larmpos);

        my_sleep();
        // Execute init pulse
        ACE_OS::printf("\nShowing off target %d \n",targcount);
        exec_pulse(larmvel,initpulse[targcount],INIT_PW_MSEC);

        ACE_OS::printf("Thank you very much!\n");
    }
    
    my_sleep();
    set_arm_pos(larmpos);

    return 1;
};

void init_kinematics(void){
    // The kinematic chain for the webots icub model
    iw = new SerialChain("icub_webots",JOINTS_NUMOF,0);
    double s0x = -0.0912;
    double s0y = 0.655;
    double s0z = 0.01;
    Frame f0(Rotation::Identity(),Vector(s0x,s0y,s0z));
    iw->addJoint(new JointRotX(f0));
    double s1x = -0.01;
    double s1y = 0.0;
    double s1z = 0.0;
    Frame f1(Rotation::Identity(),Vector(s1x,s1y,s1z));
    iw->addJoint(new JointRotZ(f1));
    double s2x = -0.005;
    double s2y = 0.01;
    double s2z = 0.0;
    Frame f2(Rotation::Identity(),Vector(s2x,s2y,s2z));
    iw->addJoint(new JointRotY(f2));
    double s3x = 0.0;
    double s3y = -0.14;
    double s3z = 0.0;
    Frame f3(Rotation::Identity(),Vector(s3x,s3y,s3z));
    iw->addJoint(new JointRotX(f3));
    double s4x = 0.0;
    double s4y = -0.19;
    double s4z = 0.0;
    Frame f4(Rotation::Identity(),Vector(s4x,s4y,s4z));
    iw->setLastJointToEE(f4);
};

void init_yarp(){

    // Make yarp connections for webots time
    if(webotstime){
        bool portok = timeport.open("/yarl/webots/time/in"); 
        if(!portok){
            ACE_OS::printf("Failed to open port in.. \n");
            Network::fini();
        }
        char* outport = "/yarl/webots/time/out";
        char* inport = "/yarl/webots/time/in";
        bool netok = Network::connect(outport,inport);
        if(!netok){
            ACE_OS::printf("Failed to connect ports.. \n");
            Network::fini();
        }
    }

    // Make yarp connections for arm
    Property options;
    options.put("robot","icub");
    options.put("device","remote_controlboard");
    options.put("local","/yarl/left_arm");
    options.put("remote","/yarl/webots/left_arm");
    // The arm interfaces
    ddlarm = new PolyDriver(options);
    if(!ddlarm->isValid())
    {
        ACE_OS::printf("Device not available. Here are the known devices:\n");
        ACE_OS::printf("%s", Drivers::factory().toString().c_str());
        Network::fini();
    }
    ddlarm->view(larmpos);
    if(larmpos==0){
        ACE_OS::printf("Error getting IPositionControl interface.\n");
        Network::fini();
    }
    ddlarm->view(larmenc);
    if(larmenc==0){
        ACE_OS::printf("Error getting IEncoders interface.\n");
        Network::fini();
    }
    ddlarm->view(larmvel);
    if(larmvel==0){
        ACE_OS::printf("Error getting IVelocityControl interface.\n");
        Network::fini();
    }
    ddlarm->view(larmamp);
    if(larmamp==0){
        ACE_OS::printf("Error getting IAmplifier interface.\n");
        Network::fini();
    }
    // Enable the motors
    for(int i=0;i<JOINTS_NUMOF;i++){
        larmamp->enableAmp(i);
    }
};

void set_arm_pos(IPositionControl* p, double* pos){
    double yarp_args[YARP_JOINTS_NUMOF];
    for(int i=0;i<YARP_JOINTS_NUMOF;i++)
        yarp_args[i] = 0.0;
    for(int i=0;i<LEARN_PARAM_NUMOF;i++)
        yarp_args[i] = pos[i];
    p->setPositionMode();
    p->positionMove((const double*)yarp_args);
    bool done = false;
    while(!done)
        p->checkMotionDone(&done);
};

void calc_handpos(IEncoders* e, double* handpos){
    // Get encoder values
    double encoders[YARP_JOINTS_NUMOF];
    e->getEncoders(encoders);
    //ACE_OS::printf("Got initial encoder values");
    //for(int i=0;i<JOINTS_NUMOF;i++)
    //    ACE_OS::printf(" %f",encoders[i]);
    //ACE_OS::printf("\n");
    // Calculate starting coordinates
    JointVector iw_q(iw->nrOfJoints());
    for(int i=0;i<iw->nrOfJoints();i++)
        iw_q[i] = encoders[i]*deg2rad;
    // The forward kinematics for the webots iCub model
	Jnt2CartPos* jnt2cartpos = iw->createJnt2CartPos();
	assert(jnt2cartpos!=0);
    int res = jnt2cartpos->evaluate(iw_q);
    assert(res==0);
    Frame frame_base_ee;
    jnt2cartpos->getFrame(frame_base_ee);
    for(int i=0;i<3;i++)
        handpos[i] = frame_base_ee.p[i];
    //ACE_OS::printf("End effector frame origin:");
    //for(int i=0;i<3;i++)
    //    ACE_OS::printf(" %f",frame_base_ee.p[i]);
    //ACE_OS::printf("\n");
    delete(jnt2cartpos);
};

void calc_pulse(double *pulse,double *handpos, bool expl){
	// Calculate Displacement
    double disp[3];
    for(int i=0;i<3;i++)
	    disp[i] = (TARG_COORDS[targcount][i]-handpos[i]);
	// Calcultate pulse speeds
    // *** S0:
    // A positive change in the S0 joint always contributes a positive change
    // along the y axis.
    // If the joint position is higher than 90, a positive change produces a
    // positive change along the z axis.
    // Otherwise a positive change produces a negative change.
	pulse[0] = disp[1]*DISP_CONV*((double)rand()/(double)INT_MAX);
    // *** S1:
    // A positive change in the S1 joint always produces a positive change 
    // along the x axis.
    // If the joint position is higher than 0, a positive change produces a
    // negative change along the y axis.
    // Otherwise a positive change produces a negative change along the y axis.
    // If the S0 joint position is above 90 then a positive change pruduces a
    // negative change along the y axis.
    // Otherwise a positive change produces a positive change along the y axis.
	pulse[1] = disp[0]*DISP_CONV*((double)rand()/(double)INT_MAX);
    // *** S2:
    // A positive change in the S2 joint always produces a positive change
    // along the y axis.
    // The contribution along the x and z axis depends on the joint
    // configuration of the other joints.
	pulse[2] = disp[1]*DISP_CONV*((double)rand()/(double)INT_MAX);
    // *** S3:
    // A positive change in the S3 joint always produces a positive change
    // along the z axis.
    // If the S1 joint position is above -90 then a positive change produces a
    // positive change along the y axis.
    // Otherwise a positive change produces a negative change along the y axis.
    // If the S1 joint position is above 0 then a positive change produces a
    // positive change along the x axis
    // Otherwise a positive change produces a negative change along the x axis.
	pulse[3] = disp[2]*DISP_CONV*((double)rand()/(double)INT_MAX);
};

void exec_pulse(IVelocityControl* v, double* pulse, int pw){
    v->setVelocityMode();
    ACE_OS::printf("Executing pulse:");
    for(int i=0;i<LEARN_PARAM_NUMOF;i++)
        ACE_OS::printf(" %f",pulse[i]);
    ACE_OS::printf("\n");
	v->velocityMove((const double*)pulse);
    my_sleep(pw);
	// Stop hand
	v->stop();
};

bool targ_reached(double* handpos){
    bool tr = true;
    for(int i=0;i<3;i++){
        if(fabs(TARG_COORDS[targcount][i]-handpos[i])>PROX_MAX)
            tr = false;
    }
    return tr;
};

void calc_disp(double* handpos, double* disp){
    disp[3] = 0.0;
    for(int i=0;i<3;i++){
	    disp[i] = (TARG_COORDS[targcount][i]-handpos[i]);
        disp[3] += fabs(disp[i]);
    }
};

void my_sleep(int msecs){
    double starttime, time, passtime;
    int slsecs = msecs/1000;
    int slmsecs = msecs%1000;
    if(webotstime){
        // Wait for Webots time to pass
	    Bottle* wbt = timeport.read();
        starttime = wbt->get(0).asDouble();
        //ACE_Time_Value t;
        //t.set(0.25);
        do{
	        //ACE_OS::sleep(t);
	        wbt = timeport.read();
            time = wbt->get(0).asDouble();
	        passtime = time-starttime;
            //ACE_OS::printf("%f wbt secs passed.\n",passtime);
	    }while(passtime<(double)msecs/1000.0);
        //ACE_OS::printf("%f wbt secs passed.\n",passtime);
    }else{
        ACE_OS::sleep(ACE_Time_Value(slsecs,slmsecs));
        //ACE_OS::printf("%d real time secs passed.\n",msecs);
    }
};
