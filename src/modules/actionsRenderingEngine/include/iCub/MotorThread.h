/* 
* Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
* Author: Carlo Ciliberto, Vadim Tikhanoff
* email:   carlo.ciliberto@iit.it vadim.tikhanoff@iit.it
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


#ifndef __MOTOR_THREAD__
#define __MOTOR_THREAD__

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/SVD.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/action/actionPrimitives.h>


#include <vector>

#include <iCub/utils.h>

#define HEAD_MODE_IDLE              0
#define HEAD_MODE_GO_HOME           1
#define HEAD_MODE_TRACK_HAND        2
#define HEAD_MODE_TRACK_TEMP        3
#define HEAD_MODE_TRACK_FIX         4
#define HEAD_MODE_LOOK              5

#define ARM_MODE_IDLE               0
#define ARM_MODE_LEARN_ACTION       1
#define ARM_MODE_LEARN_KINOFF       2
#define ARM_MODE_FINE_REACHING      3

#define GRASP_STATE_IDLE            0
#define GRASP_STATE_ABOVE           1
#define GRASP_STATE_SIDE            2

#define ARM_HOMING_PERIOD           1.5     //[s]


#define S2C_HOMOGRAPHY              VOCAB4('h','o','m','o')
#define S2C_DISPARITY               VOCAB4('d','i','s','p')
#define S2C_NETWORK                 VOCAB4('n','e','t','w')


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;
using namespace iCub::perception;


struct Dragger
{
    ICartesianControl           *ctrl;

    double                      extForceThresh;
    double                      t0;
    double                      samplingRate;
    Vector                      x0;
    int                         arm;
    Bottle                      actions;
    string                      actionName;
    double                      min;
    double                      max;

    bool                        using_impedance;

    iCub::ctrl::Integrator      *I;
    double                      damping;
    double                      inertia;

    Dragger()
    {
        I=NULL;
    }

    ~Dragger()
    {
        if(I!=NULL)
            delete I;
    }

    void init(Bottle &initializer, double thread_rate);
};



class MotorThread: public RateThread
{
private:
    ResourceFinder                      &rf;

    StereoTarget                        &stereo_target;

    ObjectPropertiesCollectorPort       &opcPort;

    PolyDriver                          *drv_head;
    PolyDriver                          *drv_torso;
    PolyDriver                          *drv_ctrl_gaze;
    PolyDriver                          *drv_arm[2];
    PolyDriver                          *drv_car_arm[2];

    IEncoders                           *enc_arm[2];
    IEncoders                           *enc_head;
    IEncoders                           *enc_torso;
    IGazeControl                        *ctrl_gaze;
        
    IPositionControl                    *pos_torso;
    IVelocityControl2                   *vel_torso;
    IControlMode2                       *ctrl_mode_torso;
    IInteractionMode                    *int_mode_torso;
    IImpedanceControl                   *ctrl_impedance_torso;

    IControlMode2                       *ctrl_mode_arm[2];
    IPositionControl                    *pos_arm[2];
    IInteractionMode                    *int_mode_arm[2];
    IImpedanceControl                   *ctrl_impedance_arm[2];

    int                                 initial_gaze_context;
    int                                 default_gaze_context;

    bool                                gazeUnderControl;
    bool                                status_impedance_on;

    bool                                stereo_track;
    int                                 dominant_eye;

    ff2LayNN_tansig_purelin             net;

    ActionPrimitivesLayer2              *action[2];

    bool                                homeFixCartType;
    Vector                              homeFix;
    Vector                              reachAboveDisp;
    Vector                              graspAboveRelief;
    Vector                              pushAboveRelief;
    double                              targetInRangeThresh;
    double                              extForceThresh[2];
    double                              default_exec_time;
    double                              reachingTimeout;

    //tool
    Vector                              takeToolPos[2];
    Vector                              takeToolOrient[2];

    Vector                              homePos[2];
    Vector                              homeOrient[2];
    Vector                              reachSideDisp[2];
    Vector                              reachAboveOrient[2];
    Vector                              reachAboveCata[2];
    Vector                              reachSideOrient[2];
    Vector                              deployPos[2];
    Vector                              deployOrient[2];
    Vector                              drawNearPos[2];
    Vector                              drawNearOrient[2];
    Vector                              shiftPos[2];
    Vector                              expectPos[2];
    Vector                              expectOrient[2];

    //tactile perception
    iCub::perception::Model             *graspModel[2];
    string                              graspPath[2];



    //stereo 2 cartesian mode
    int                                 modeS2C;
    bool                                neuralNetworkAvailable;
    RpcClient                           disparityPort;

    vector<Vector>                      pos_torsoes;
    vector<Vector>                      handPoses;
    vector<Vector>                      headPoses;

    Bottle                              bKinOffsets;
    string                              kinematics_path;
    Vector                              defaultKinematicOffset[2];
    Vector                              currentKinematicOffset[2];
    Vector                              table;
    double                              table_height;
    double                              table_height_tolerance;

    int                                 armInUse;
    int                                 head_mode;
    int                                 arm_mode;

    int                                 grasp_state;

    bool                                waveing;
    bool                                avoid_table;
    bool                                closed;
    bool                                interrupted;

    RpcClient                           wbdPort;

    //drag stuff
    Port                                wrenchPort[2];

    string                              actions_path;
    Dragger                             dragger;

    Vector                              gaze_fix_point;
    double                              avoid_table_height[2];

    double                              random_pos_y;


    bool loadExplorationPoses(const string &file_name);
    int checkArm(int arm);
    int checkArm(int arm, Vector &xd, const bool applyOffset=true);
    bool checkOptions(Bottle &options, const string &parameter);
    Vector eye2root(const Vector &out,bool forehead);
    bool stereoToCartesianHomography(const Vector &stereo, Vector &xd);
    bool stereoToCartesianDisparity(const Vector &stereo, Vector &xd);
    bool stereoToCartesianNetwork(const Vector &stereo, Vector &xd);
    Vector randomDeployOffset();
    bool getGeneralOptions(Bottle &b);
    bool loadKinematicOffsets(const string &_kinematics_path);
    bool saveKinematicOffsets();
    bool getArmOptions(Bottle &b, const int &arm);
    void goHomeHelper(ActionPrimitives *action, const Vector &xin, const Vector &oin);
    void close();
    
    bool avoidTable(bool avoid);

public:
    MotorThread(ResourceFinder &_rf, Initializer *initializer)
        :RateThread(20),rf(_rf),stereo_target(initializer->stereo_target),opcPort(initializer->port_opc)
    {
        ctrl_gaze=NULL;
        ctrl_mode_torso=NULL;
        int_mode_torso=NULL;
        drv_head=drv_torso=drv_ctrl_gaze=NULL;
        drv_arm[LEFT]=drv_arm[RIGHT]=NULL;
        drv_car_arm[LEFT]=drv_car_arm[RIGHT]=NULL;
        ctrl_mode_arm[LEFT]=ctrl_mode_arm[RIGHT]=NULL;
        int_mode_arm[LEFT]=int_mode_arm[RIGHT]=NULL;
        action[LEFT]=action[RIGHT]=NULL;
        closed=false;
    }

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();

    bool wbdRecalibration()
    {
        if (wbdPort.getOutputCount()>0)
        {
            Bottle cmd,reply;
            cmd.addInt(0);
            wbdPort.write(cmd,reply);
            return true;
        }
        else
            return false;
    }

    void trackTemplate(Bottle &options)
    {
        if(!gazeUnderControl)
        {
            ctrl_gaze->restoreContext(default_gaze_context);

            if(checkOptions(options,"no_sacc"))
                ctrl_gaze->setSaccadesStatus(false);

            head_mode=HEAD_MODE_TRACK_TEMP;

            gazeUnderControl=true;
        }
    }


    void lookAtHand(Bottle &options)
    {
        if(checkOptions(options,"no_sacc"))
            ctrl_gaze->setSaccadesStatus(false);
        head_mode=HEAD_MODE_TRACK_HAND;
    }

    void keepFixation(Bottle &options)
    {
        gazeUnderControl=true;
        ctrl_gaze->setTrackingMode(true);
        if(checkOptions(options,"no_sacc"))
            ctrl_gaze->setSaccadesStatus(false);
        head_mode=HEAD_MODE_TRACK_FIX;
    }

    void setGazeIdle()
    {
        head_mode=HEAD_MODE_IDLE;
        ctrl_gaze->stopControl();
        ctrl_gaze->restoreContext(initial_gaze_context);
        gazeUnderControl=false;
    }

    bool setArmInUse(int arm)
    {
        if(action[arm]==NULL)
            return false;

        armInUse=arm;
        return true;
    }

    bool setWaveing(bool _waveing)
    {
        waveing=_waveing;
        
        if(waveing)
        {
            if(action[RIGHT]!=NULL)
                action[RIGHT]->enableArmWaving(homePos[RIGHT]);

            if(action[LEFT]!=NULL)
                action[LEFT]->enableArmWaving(homePos[LEFT]);
        }
        else
        {
            if(action[RIGHT]!=NULL)
                action[RIGHT]->disableArmWaving();
            if(action[LEFT]!=NULL)
                action[LEFT]->disableArmWaving();
        }
        return true;
    }

    void update();
    void interrupt();
    void reinstate();

    //get the table height
    bool getTableHeight(double *_table_height)
    {
        *_table_height=table_height-table_height_tolerance;
        return true;
    }

    //transforms stereo coordinates to cartesian space
    bool stereoToCartesian(const Vector &stereo, Vector &xd);
    bool targetToCartesian(Bottle *target, Vector &xd);

    // basic commands
    bool preTakeHand(Bottle &options);
    bool goUp(Bottle &options, const double h);
    bool goHome(Bottle &options);
    bool reach(Bottle &options);
    bool powerGrasp(Bottle &options);
    bool push(Bottle &options);
    bool point(Bottle &options);
    bool look(Bottle &options);
    bool grasp(Bottle &options);
    bool grasp_tool(Bottle &options);
    bool release(Bottle &options);
    bool deploy(Bottle &options);
    bool drawNear(Bottle &options);
    bool shiftAndGrasp(Bottle &options);
    bool expect(Bottle &options);
    bool give(Bottle &options);

    bool clearIt(Bottle &options);

    // explore
    bool exploreTorso(Bottle &options);
    bool exploreHand(Bottle &options);

    //tool functions
    bool takeTool(Bottle &options);


    bool getHandImagePosition(Bottle &hand_image_pos);
    bool isHolding(Bottle &options);
    bool calibTable(Bottle &options);
    bool calibFingers(Bottle &options);

    bool startLearningModeAction(Bottle &options);
    bool suspendLearningModeAction(Bottle &options);
    bool imitateAction(Bottle &options);

    bool startLearningModeKinOffset(Bottle &options);
    bool suspendLearningModeKinOffset(Bottle &options);

    bool changeElbowHeight(const int arm, const double height, const double weight);

    bool setImpedance(bool turn_on);
    bool setTorque(bool turn_on, int arm=ARM_IN_USE);

    int setStereoToCartesianMode(const int &mode, Bottle &reply);
    int setStereoToCartesianMode(const int &mode);

    void setGraspState(bool side)
    {
        if(side)
            grasp_state=GRASP_STATE_SIDE;
        else
            grasp_state=GRASP_STATE_ABOVE;
    }

    void getStatus(Bottle &status);
};


#endif


