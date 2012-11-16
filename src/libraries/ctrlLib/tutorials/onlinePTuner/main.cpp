/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

//
// A tutorial on how to use ctrlLib library for online tuning of low-level P controllers.
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <stdio.h>
#include <math.h>
#include <string>
#include <sstream>
#include <algorithm>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <iCub/ctrl/tuning.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/*******************************************************/
int main(int argc, char *argv[])
{
    // usual stuff...
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP is not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.configure("ICUB_ROOT",argc,argv);

    string name=rf.check("name",Value("tuner")).asString().c_str();
    string robot=rf.check("robot",Value("icub")).asString().c_str();
    string part=rf.check("part",Value("right_arm")).asString().c_str();
    int joint=rf.check("joint",Value(11)).asInt();
    double encoder=rf.check("encoder",Value(2.43)).asDouble();

    Property pOptions;
    pOptions.put("device","remote_controlboard");
    pOptions.put("remote",("/"+robot+"/"+part).c_str());
    pOptions.put("local",("/"+name+"/"+part).c_str());
    PolyDriver driver(pOptions);
    if (!driver.isValid())
    {
        printf("Part \"%s\" is not ready!\n",("/"+robot+"/"+part).c_str());
        return -1;
    }

    // ##### Preamble
    // The objective is to tune online a P controller that will make 
    // the joint plant behave as the well known second order dynamics:
    //
    // w_n^2 / (s^2 + 2*zeta*w_n*s + w_n^2).
    //
    // The plant is to be identified using an Extended-Kalman-Filter (EKF)
    // under the assumption that the adopted model obeys to the following
    // tansfer function which gives out the position when fed with voltage:
    //
    // K / (s*(1 + tau*s))
    //
    // You might have realized that we disregard the electrical dynamics in
    // favour of the slower mechanical one represented by the time constant tau.
    //
    // ##### Additional notes on the control design
    // The low-level layer provides the user with a PID controller. However, the
    // I part is not specifically required for such control task since by virtue
    // of the internal model principle the steady-state error can be already 
    // compensated thanks to the presence of the integrator in the plant. On the
    // other hand, this principle holds only for linear systems and indeed,
    // nonlinear effects - such as the stiction - make the final behaviour deviate
    // from this baseline, eventually requiring the integral part. Nonetheless,
    // if stiction is compensated separately, I is again not strictly needed.
    // 
    // Finally, a few words about the D part. In order to include D within the design,
    // the derivative part must be implemented according to the standard which states
    // that D = Kd*s / (1 + tau_d*s).
    //
    // In the following a short example will guide you through the steps required
    // to identify the plant, validate the retrieved model and then design the
    // P controller that meets the user specifications. Stiction identification is
    // carried out as well.

    // First thing to do is to prepare configuration options
    // which are to be given in the form:
    //
    // [general]
    // joint ...
    // port  ...
    // [plant_estimation]
    // Ts    ...
    // ...
    // [stiction_estimation]
    // ...
    Property pGeneral;
    pGeneral.put("joint",joint);
    // the "port" option allows opening up a yarp port which
    // will stream out useful information while identifying
    // and validating the system
    pGeneral.put("port",("/"+name+"/info:o").c_str());
    string sGeneral="(general ";
    sGeneral+=pGeneral.toString().c_str();
    sGeneral+=')';

    Bottle bGeneral,bPlantEstimation,bStictionEstimation;
    bGeneral.fromString(sGeneral.c_str());
    // here follow the parameters for the EKF along with the
    // initial values for tau and K
    bPlantEstimation.fromString("(plant_estimation (Ts 0.01) (Q 1.0) (R 1.0) (P0 100000.0) (tau 1.0) (K 1.0) (max_pwm 800.0))");
    bStictionEstimation.fromString("(stiction_estimation (Ts 0.01) (T 2.0) (vel_thres 5.0) (e_thres 1.0) (gamma (10.0 10.0)) (stiction (0.0 0.0)))");

    // compose the overall configuration
    Bottle bConf=bGeneral;
    bConf.append(bPlantEstimation);
    bConf.append(bStictionEstimation);
    pOptions.fromString(bConf.toString().c_str());

    OnlineCompensatorDesign designer;
    if (!designer.configure(driver,pOptions))
    {
        printf("Configuration failed!\n");
        return -1;
    }

    // let's start the plant estimation by
    // setting up an experiment that will
    // last 20 seconds
    //
    // the experiment foresees a direct control
    // in voltage (pwm)
    Property pPlantEstimation;
    pPlantEstimation.put("max_time",20.0);
    // the switch_timeout option enforces a timeout
    // in the voltage switching logic to produce
    // rising and falling transitions.
    pPlantEstimation.put("switch_timeout",2.0);
    designer.startPlantEstimation(pPlantEstimation);

    printf("Estimation experiment will last %g seconds...\n",
           pPlantEstimation.find("max_time").asDouble());

    double t0=Time::now();
    while (!designer.isDone())
    {
        printf("elapsed %d [s]\n",(int)(Time::now()-t0));
        Time::delay(1.0);
    }

    // retrieve the identified values (averaged over time)
    Property pResults;
    designer.getResults(pResults);
    double tau=pResults.find("tau_mean").asDouble();
    double K=pResults.find("K_mean").asDouble();

    printf("plant = K/s * 1/(1+s*tau)\n");
    printf("Estimated parameters...\n");
    printf("tau = %g\n",tau);
    printf("K   = %g\n",K);

    // put the model to test for 10 seconds by
    // simulating the plant with a Kalman filter
    Property pPlantValidation;
    pPlantValidation.put("max_time",10.0);
    pPlantValidation.put("switch_timeout",2.0);
    pPlantValidation.put("tau",tau);
    pPlantValidation.put("K",K);
    // the "measure_update_ticks" option tells that
    // the measurement update will occur 100 times slower
    // with respect to the sample time. This way we give
    // the model enough time to evolve to test its properties
    // before comparing its response with the real output
    // to counteract drift phenomena.
    pPlantValidation.put("measure_update_ticks",100);
    designer.startPlantValidation(pPlantValidation);

    printf("Validation experiment will last %g seconds...\n",
           pPlantValidation.find("max_time").asDouble());

    t0=Time::now();
    while (!designer.isDone())
    {
        printf("elapsed %d [s]\n",(int)(Time::now()-t0));
        Time::delay(1.0);
    }

    // The design part...
    printf("Tuning P controller...\n");
    Property pControllerRequirements,pController;
    pControllerRequirements.put("tau",tau);
    pControllerRequirements.put("K",K);
    // we can specify either the natural frequency (in Hz) or
    // the damping ratio - since we have just one degree
    // of freedom controller (P). Therefore it is a common
    // rule to specify the crossover frequency (in Hz) which
    // amounts to the frequency where the open loop response
    // given by Kp * plant has a unity-gain; f_c determines
    // the bandwidth of the closed loop response.
    // Requirements: track min-jerk profiles whose trajectory
    // time is 2.0 seconds.
    // From spectral analysis we know that the most of the
    // frequency content of a signal composed of the given
    // min-jerk pulses lies whithin the range [0,0.6] Hz.
    // A crossover frequency of 0.75 is therefore suitable.
    pControllerRequirements.put("f_c",0.75);
    pControllerRequirements.put("type","P");
    designer.tuneController(pControllerRequirements,pController);
    printf("tuning results: %s\n",pController.toString().c_str());
    double Kp=pController.find("Kp").asDouble();
    printf("found Kp = %g\n",Kp);
    int scale=4;
    double Kp_fw=Kp*encoder*(1<<scale);
    printf("Kp (firmware) = %g; shift factor = %d\n",Kp_fw,scale);

    // let's identify the stictions values as well
    Property pStictionEstimation;
    pStictionEstimation.put("max_time",60.0);
    // the joint will be controlled under the action
    // of a high-level PID controller
    pStictionEstimation.put("Kp",Kp);
    pStictionEstimation.put("Ki",0.0);
    pStictionEstimation.put("Kd",0.0);
    designer.startStictionEstimation(pStictionEstimation);

    printf("Stiction estimation experiment will last no more than %g seconds...\n",
           pStictionEstimation.find("max_time").asDouble());

    t0=Time::now();
    while (!designer.isDone())
    {
        printf("elapsed %d [s]\n",(int)(Time::now()-t0));
        Time::delay(1.0);
    }

    // retrieve the stiction values
    designer.getResults(pResults);
    Vector stiction(2);
    stiction[0]=pResults.find("stiction").asList()->get(0).asDouble();
    stiction[1]=pResults.find("stiction").asList()->get(1).asDouble();
    printf("Stiction values: up = %g; down = %g\n",stiction[0],stiction[1]);

    // now that we know P and stiction, let's try out our controller
    // against the current version
    Property pControllerValidation;
    pControllerValidation.put("max_time",60.0);
    pControllerValidation.put("Kp",Kp_fw);
    pControllerValidation.put("scale",scale);
    ostringstream str;
    str<<"( ";
    str<<stiction[0];
    str<<" ";
    str<<stiction[1];
    str<<" )";
    Value val; val.fromString(str.str().c_str());
    pControllerValidation.put("stiction",val);
    // we let yarp apply the stiction values upon transitions;
    // by default the firmware takes care of it.
    pControllerValidation.put("stiction_compensation","middleware");
    // let's go for the classical "min-jerk" reference input with a
    // period of 2 seconds.
    // we have also the "square" waveform at our disposal, just in
    // case we aim at measuring traditional controller step response.
    pControllerValidation.put("ref_type","min-jerk");
    pControllerValidation.put("ref_period",2.0);
    // for the "min-jerk" reference type it turns to be useful to
    // have a "sustain" time where the reference is kept to the
    // final set-point before switching to the next value.
    pControllerValidation.put("ref_sustain_time",1.0);
    // in this experiment both the current controller and our controller 
    // will act, one after other, each for 4 cycles of 1 rising and 1 falling
    // transition in a row.
    pControllerValidation.put("cycles_to_switch",4);
    designer.startControllerValidation(pControllerValidation);

    printf("Controller validation will last %g seconds...\n",
           pControllerValidation.find("max_time").asDouble());

    t0=Time::now();
    while (!designer.isDone())
    {
        printf("elapsed %d [s]\n",(int)(Time::now()-t0));
        Time::delay(1.0);
    }

    return 0;
}


