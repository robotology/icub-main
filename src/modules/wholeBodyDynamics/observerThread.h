/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo, Matteo Fumagalli
 * email:  marco.randazzo@iit.it
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

#ifndef OBSERVER_THREAD
#define OBSERVER_THREAD

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>
#include <iCub/skinDynLib/skinContactList.h>

#include <iostream>
#include <iomanip>
#include <string.h>
#include <list>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

#define MAX_JN 12
#define MAX_FILTER_ORDER 6
#define SKIN_EVENTS_TIMEOUT 0.2     // max time (in sec) a contact is kept without reading anything from the skin events port

enum thread_status_enum {STATUS_OK=0, STATUS_DISCONNECTED}; 
enum calib_enum {CALIB_ALL=0, CALIB_ARMS, CALIB_LEGS, CALIB_FEET};

// struct version
// {
//     int head_version;
//     int legs_version;
// };

// filter
double lpf_ord1_3hz(double input, int j);

class iCubStatus
{
    public:
    double timestamp;
    Vector all_q_up, all_dq_up, all_d2q_up;
    Vector all_q_low, all_dq_low, all_d2q_low;

    Vector ft_arm_left;
    Vector ft_arm_right;
    Vector ft_leg_left;
    Vector ft_leg_right;
    Vector ft_foot_left;
    Vector ft_foot_right;

    Vector inertial_w0,inertial_dw0,inertial_d2p0;

    bool iCub_not_moving;

    iCubStatus ()
    {
        timestamp=0;
        ft_arm_left=0;
        ft_arm_right=0;
        ft_leg_left=0;
        ft_leg_right=0;
        ft_foot_left=0;
        ft_foot_right=0;
        inertial_w0.resize(3);     inertial_w0.zero();
        inertial_dw0.resize(3);    inertial_dw0.zero();
        inertial_d2p0.resize(3);   inertial_d2p0.zero();
        all_q_up.resize(3+7+7);    all_q_up.zero();
        all_dq_up.resize(3+7+7);   all_dq_up.zero();
        all_d2q_up.resize(3+7+7);  all_d2q_up.zero();
        all_q_low.resize(3+6+6);   all_q_low.zero();
        all_dq_low.resize(3+6+6);  all_dq_low.zero();
        all_d2q_low.resize(3+6+6); all_d2q_low.zero();
        ft_arm_left.resize(6);     ft_arm_left.zero();
        ft_arm_right.resize(6);    ft_arm_right.zero();
        ft_leg_left.resize(6);     ft_leg_left.zero();
        ft_leg_right.resize(6);    ft_leg_right.zero();
        ft_foot_left.resize(6);    ft_foot_left.zero();
        ft_foot_right.resize(6);   ft_foot_right.zero();
    }

    Vector get_q_head()    {return all_q_up.subVector(0,2);}
    Vector get_dq_head()   {return all_dq_up.subVector(0,2);}
    Vector get_d2q_head()  {return all_d2q_up.subVector(0,2);}
    Vector get_q_larm()    {return all_q_up.subVector(3,9);}
    Vector get_dq_larm()   {return all_dq_up.subVector(3,9);}
    Vector get_d2q_larm()  {return all_d2q_up.subVector(3,9);}
    Vector get_q_rarm()    {return all_q_up.subVector(10,16);}
    Vector get_dq_rarm()   {return all_dq_up.subVector(10,16);}
    Vector get_d2q_rarm()  {return all_d2q_up.subVector(10,16);}
    Vector get_q_torso()   {return all_q_low.subVector(0,2);}
    Vector get_dq_torso()  {return all_dq_low.subVector(0,2);}
    Vector get_d2q_torso() {return all_d2q_low.subVector(0,2);}
    Vector get_q_lleg()    {return all_q_low.subVector(3,8);}
    Vector get_dq_lleg()   {return all_dq_low.subVector(3,8);}
    Vector get_d2q_lleg()  {return all_d2q_low.subVector(3,8);}
    Vector get_q_rleg()    {return all_q_low.subVector(9,14);}
    Vector get_dq_rleg()   {return all_dq_low.subVector(9,14);}
    Vector get_d2q_rleg()  {return all_d2q_low.subVector(9,14);}

    bool checkIcubNotMoving();
    void dump (FILE* f);

};

// class inverseDynamics: class for reading from Vrow and providing FT on an output port
class inverseDynamics: public RateThread
{
public:
    bool       com_enabled;
    bool       com_vel_enabled;
    bool       dummy_ft;
    bool       w0_dw0_enabled;
    bool       dumpvel_enabled;
    bool       auto_drift_comp;
    bool       default_ee_cont;
    bool       add_legs_once;

private:
    string      robot_name;
    string      local_name;
    bool        autoconnect;
    version_tag icub_type;

    PolyDriver *ddAL;
    PolyDriver *ddAR;
    PolyDriver *ddH;
    IEncoders  *iencs_arm_left;
    IEncoders  *iencs_arm_right;
    IEncoders  *iencs_head;
    IInteractionMode *iint_arm_left;
    IInteractionMode *iint_arm_right;
    IInteractionMode *iint_head;
    IControlMode2 *icmd_arm_left;
    IControlMode2 *icmd_arm_right;
    IControlMode2 *icmd_head;

    PolyDriver *ddLL;
    PolyDriver *ddLR;
    PolyDriver *ddT;
    IEncoders  *iencs_leg_left;
    IEncoders  *iencs_leg_right;
    IEncoders  *iencs_torso;
    IInteractionMode *iint_leg_left;
    IInteractionMode *iint_leg_right;
    IInteractionMode *iint_torso;
    IControlMode2 *icmd_leg_left;
    IControlMode2 *icmd_leg_right;
    IControlMode2 *icmd_torso;

    string part;

    const  long double zero_sens_tolerance;
    double skinContactsTimestamp;
    
    //input ports
    BufferedPort<Vector> *port_ft_arm_left;
    BufferedPort<Vector> *port_ft_arm_right;
    BufferedPort<Vector> *port_ft_leg_left;
    BufferedPort<Vector> *port_ft_leg_right;
	BufferedPort<Vector> *port_ft_foot_left;
    BufferedPort<Vector> *port_ft_foot_right;
    BufferedPort<Vector> *port_inertial_thread;
    BufferedPort<iCub::skinDynLib::skinContactList> *port_skin_contacts;

    //output ports
    BufferedPort<Bottle> *port_RATorques;
    BufferedPort<Bottle> *port_RLTorques;
    BufferedPort<Bottle> *port_RWTorques;
    BufferedPort<Bottle> *port_LATorques;
    BufferedPort<Bottle> *port_LLTorques;
    BufferedPort<Bottle> *port_LWTorques;
    BufferedPort<Bottle> *port_TOTorques;
    BufferedPort<Bottle> *port_HDTorques;
    BufferedPort<Vector> *port_external_wrench_RA;
    BufferedPort<Vector> *port_external_wrench_LA;
    BufferedPort<Vector> *port_external_wrench_RL;
    BufferedPort<Vector> *port_external_wrench_LL;
    BufferedPort<Vector> *port_external_wrench_RF;
    BufferedPort<Vector> *port_external_wrench_LF;
    BufferedPort<Vector> *port_external_cartesian_wrench_RA;
    BufferedPort<Vector> *port_external_cartesian_wrench_LA;
    BufferedPort<Vector> *port_external_cartesian_wrench_RL;
    BufferedPort<Vector> *port_external_cartesian_wrench_LL;
    BufferedPort<Vector> *port_external_cartesian_wrench_RF;
    BufferedPort<Vector> *port_external_cartesian_wrench_LF;
    BufferedPort<Vector> *port_sensor_wrench_RL;
    BufferedPort<Vector> *port_sensor_wrench_LL;
    BufferedPort<Vector> *port_model_wrench_RL;
    BufferedPort<Vector> *port_model_wrench_LL;
    BufferedPort<Vector> *port_external_wrench_TO;
    BufferedPort<Vector> *port_com_all;
    BufferedPort<Vector> *port_com_all_foot;
    BufferedPort<Vector> *port_com_lb;
    BufferedPort<Vector> *port_com_ub;
    BufferedPort<Vector> *port_com_la;
    BufferedPort<Vector> *port_com_ra;
    BufferedPort<Vector> *port_com_ll;
    BufferedPort<Vector> *port_com_rl;
    BufferedPort<Vector> *port_com_hd;
    BufferedPort<Vector> *port_com_to;
    BufferedPort<Vector> *port_monitor;
    BufferedPort<iCub::skinDynLib::skinContactList> *port_contacts;
    BufferedPort<Vector> *port_dumpvel;
    BufferedPort<Vector> *port_COM_vel;
    BufferedPort<Matrix> *port_COM_Jacobian;
    BufferedPort<Vector> *port_all_velocities;
    BufferedPort<Vector> *port_all_positions;
    BufferedPort<Matrix> *port_root_position_mat;
    BufferedPort<Vector> *port_root_position_vec;

    // ports outputing the external dynamics seen at the F/T sensor
    BufferedPort<Vector> *port_external_ft_arm_left;
    BufferedPort<Vector> *port_external_ft_arm_right;
    BufferedPort<Vector> *port_external_ft_leg_left;
    BufferedPort<Vector> *port_external_ft_leg_right;
    yarp::os::Stamp timestamp;

    bool first;
    thread_status_enum thread_status;

    AWLinEstimator  *InertialEst;
    AWLinEstimator  *linEstUp;
    AWQuadEstimator *quadEstUp;
    AWLinEstimator  *linEstLow;
    AWQuadEstimator *quadEstLow;

    int ctrlJnt;
    int allJnt;
    iCubWholeBody *icub;
    iCubWholeBody *icub_sens;
    iCubStatus    current_status;

    size_t status_queue_size;
    list<iCubStatus> previous_status;
    list<iCubStatus> not_moving_status;

    Vector encoders_arm_left;
    Vector encoders_arm_right;
    Vector encoders_head;

    Vector encoders_leg_left;
    Vector encoders_leg_right;
    Vector encoders_torso;

    Vector Fend,Muend;
    Vector F_LArm, F_RArm;
    Vector F_iDyn_LArm, F_iDyn_RArm, Offset_LArm, Offset_RArm;
    Vector F_ext_left_arm, F_ext_right_arm, F_ext_torso;
    Vector F_ext_cartesian_left_arm, F_ext_cartesian_right_arm;
    Vector F_ext_left_leg, F_ext_right_leg;
    Vector F_ext_left_foot, F_ext_right_foot;
    Vector F_sns_left_leg, F_sns_right_leg;
    Vector F_mdl_left_leg, F_mdl_right_leg;
    Vector F_ext_cartesian_left_leg, F_ext_cartesian_right_leg;
    Vector F_ext_cartesian_left_foot, F_ext_cartesian_right_foot;
    Vector F_LLeg, F_RLeg; 
    Vector F_LFoot, F_RFoot; 
    Vector F_iDyn_LLeg, F_iDyn_RLeg, Offset_LLeg, Offset_RLeg;
    Vector F_iDyn_LFoot, F_iDyn_RFoot, Offset_LFoot, Offset_RFoot;
    Matrix F_sens_up, F_sens_low, F_ext_up, F_ext_low;
    Vector F_ext_sens_right_arm, F_ext_sens_left_arm;       // external wrench seen at the F/T sensors
    Vector F_ext_sens_right_leg, F_ext_sens_left_leg;       // external wrench seen at the F/T sensors

    iCub::skinDynLib::skinContactList skinContacts;
    iCub::skinDynLib::dynContactList dynContacts;


    // icub model
    int comp;
    Matrix FM_sens_up,FM_sens_low;

    //COM Jacobian Matrix
    Matrix com_jac;

    Vector evalVelUp(const Vector &x);
    Vector evalVelLow(const Vector &x);
    Vector eval_domega(const Vector &x);
    Vector evalAccUp(const Vector &x);
    Vector evalAccLow(const Vector &x);

    void init_upper();
    void init_lower();
    void setUpperMeasure(bool _init=false);
    void setLowerMeasure(bool _init=false);

    void addSkinContacts();

public:
    inverseDynamics(int _rate, PolyDriver *_ddAL, PolyDriver *_ddAR, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddLR, PolyDriver *_ddT, string _robot_name, string _local_name, version_tag icub_type, bool _autoconnect=false );
    bool threadInit();
    void setStiffMode();
    inline thread_status_enum getThreadStatus() 
    {
        return thread_status;
    }

    void run();
    void threadRelease();
    void closePort(Contactable *_port);
    void writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port);
    template <class T> void broadcastData(T& _values, BufferedPort<T> *_port);
    void calibrateOffset(calib_enum calib_code=CALIB_ALL);
    bool readAndUpdate(bool waitMeasure=false, bool _init=false);
    bool getLowerEncodersSpeedAndAcceleration();
    bool getUpperEncodersSpeedAndAcceleration();
    void setZeroJntAngVelAcc();
    void sendMonitorData();
    void sendVelAccData();

};

#endif


