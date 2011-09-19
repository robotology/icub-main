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

// filter
double lpf_ord1_3hz(double input, int j);

// class inverseDynamics: class for reading from Vrow and providing FT on an output port
class inverseDynamics: public RateThread
{
public:
	bool       com_enabled;
	bool       com_vel_enabled;
	bool       dummy_ft;
	bool       w0_dw0_enabled;

private:
	string     robot_name;
	string     local_name;
	bool       autoconnect;

    PolyDriver *ddAL;
    PolyDriver *ddAR;
    PolyDriver *ddH;
    IEncoders  *iencs_arm_left;
    IEncoders  *iencs_arm_right;
    IEncoders  *iencs_head;
	
    PolyDriver *ddLL;
    PolyDriver *ddLR;
    PolyDriver *ddT;
    IEncoders  *iencs_leg_left;
    IEncoders  *iencs_leg_right;
    IEncoders  *iencs_torso;

    string part;

    Vector *ft_arm_left;
    Vector *ft_arm_right;
    Vector *inertial;
    double skinEventsLeftTimestamp;
    double skinEventsRightTimestamp;
    
	//input ports
	BufferedPort<Vector> *port_ft_arm_left;
    BufferedPort<Vector> *port_ft_arm_right;
	BufferedPort<Vector> *port_ft_leg_left;
    BufferedPort<Vector> *port_ft_leg_right;
	BufferedPort<Vector> *port_inertial_thread;
    BufferedPort<iCub::skinDynLib::skinContactList> *port_skin_events_left;
    BufferedPort<iCub::skinDynLib::skinContactList> *port_skin_events_right;

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
	BufferedPort<Vector> *port_cartesian_external_wrench_RA;
	BufferedPort<Vector> *port_cartesian_external_wrench_LA;
	BufferedPort<Vector> *port_external_wrench_TO;
	BufferedPort<Vector> *port_com_all;
	BufferedPort<Vector> *port_com_la;
	BufferedPort<Vector> *port_com_ra;
	BufferedPort<Vector> *port_com_ll;
	BufferedPort<Vector> *port_com_rl;
	BufferedPort<Vector> *port_com_hd;
	BufferedPort<Vector> *port_com_to;
    BufferedPort<Vector> *port_monitor;
    BufferedPort<iCub::skinDynLib::dynContactList> *port_dyn_contacts;

    Vector *ft_leg_left;
    Vector *ft_leg_right;

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

    Vector encoders_arm_left;
    Vector encoders_arm_right;
    Vector encoders_head;

    Vector encoders_leg_left;
    Vector encoders_leg_right;
    Vector encoders_torso;

	Vector q_head, dq_head, d2q_head;
	Vector q_larm, dq_larm, d2q_larm;
	Vector q_rarm, dq_rarm, d2q_rarm;
	Vector all_q_up, all_dq_up, all_d2q_up;

	Vector q_torso, dq_torso, d2q_torso;
	Vector q_lleg, dq_lleg, d2q_lleg;
	Vector q_rleg, dq_rleg, d2q_rleg;
	Vector all_q_low, all_dq_low, all_d2q_low;

    Vector w0,dw0,d2p0,Fend,Muend;
    Vector F_LArm, F_RArm, F_iDyn_LArm, F_iDyn_RArm, Offset_LArm, Offset_RArm;
	Vector F_ext_left_arm, F_ext_right_arm, F_ext_torso;
	Vector F_ext_cartesian_left_arm, F_ext_cartesian_right_arm;
    Vector F_LLeg, F_RLeg, F_iDyn_LLeg, F_iDyn_RLeg, Offset_LLeg, Offset_RLeg;
	Matrix F_sens_up, F_sens_low, F_ext_up, F_ext_low;
	Vector inertial_measurements;

    // icub model
	int comp;
    Matrix FM_sens_up,FM_sens_low;

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
    inverseDynamics(int _rate, PolyDriver *_ddAL, PolyDriver *_ddAR, PolyDriver *_ddH, PolyDriver *_ddLL, PolyDriver *_ddLR, PolyDriver *_ddT, string _robot_name, string _local_name, string icub_type, bool _autoconnect=false );
    bool threadInit();
	inline thread_status_enum getThreadStatus() 
	{
		return thread_status;
	}

    void run();
    void threadRelease();
	void closePort(Contactable *_port);
	void writeTorque(Vector _values, int _address, BufferedPort<Bottle> *_port);
	void calibrateOffset(const unsigned int Ntrials);
	bool readAndUpdate(bool waitMeasure=false, bool _init=false);
	bool getLowerEncodersSpeedAndAcceleration();
	bool getUpperEncodersSpeedAndAcceleration();
	void setZeroJntAngVelAcc();
	void sendMonitorData();

};

#endif
