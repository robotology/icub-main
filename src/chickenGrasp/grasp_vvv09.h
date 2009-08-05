// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* iCub grasp library VVV09, Copyright (C) 2009 RobotCub Consortium
 * authors: Kail Frank, Theo Jacobs, Julian Schill, Yan Wu
 * http://www.robotcub.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  US
*/

#include <iostream>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

enum arm_side 
    {
	eLEFT_HAND, 
	eRIGHT_HAND
	};

class grasp_vvv09
{
private:
	Network yarp;
	Property params;
	Property options;
	Vector PrePose[2];
	double Prepose_param;
	double Endpose_param;
	PolyDriver robotDevice;
	IPositionControl *pos;
	IEncoders *encs;
	IAmplifierControl *amp;
	IVelocityControl *vels;
	IPidControl *pid;
	Vector positions;
	Vector amplifier;
	Vector velocities;
	Vector pid_val;
	Vector current_thresholds;
	Vector pid_thresholds;
	Vector refAccelerations_velMode;
	Vector fistPose;
	Vector pinchPose;
	bool* currentThresholdExceeded;
	bool* pidThresholdExceeded;
	bool* collisionDetected;
	int* currentcounter;
	int* pidcounter;
	int nj;
	int startingJoint;
	double position_threshold;
	bool* position_reached;
	double max_jointSpeed;
	Vector velocityCorrectionFactor;
	Vector targetPosition;
public:

 	grasp_vvv09(){};
	~grasp_vvv09(){robotDevice.close();};

	/**
	 * initialize parameters needed for grasping
	 * @param robot_name name of the robot you want to use, e.g. 'icub'
	 * @param prepose interpolation ratio for pregrasp position: 0 (pinch) .. 1 (fist)
	 * @param endpose interpolation ratio for grasp movement: 0 (pinch) .. 1 (fist)
	 * @param side left hand (0) or right hand (1)
	 * @return true if nothing unexpected occurs
	 */
	bool Init( string robot_name, double prepose, double endpose, int side);

	/**
	 * initialize all parameters in case the program is called from the command line. Parameters
	 * are the robot name, the interpolation ratio for pregrasp and grasp position and the side.
	 * @return true if nothing unexpected occurs
	 */
	bool Init(int arc, char *arv[]);

	/**
	 * set the current thresholds for collision detection. Value has to be reached several times in a row
	 * @param currentThresholds one threshold for each joint of arm and hand (0..16). Arm joints are ignored later.
	 * @return true if nothing unexpected occurs
	 */
	bool setCurrentThreshold(Vector currentThresholds);

	/**
	 * set the thresholds for the pid controller error for collision detection. Value has to be reached several times in a row
     * @param currentThresholds one threshold for each joint of arm and hand (0..16). Arm joints are ignored later.
	 * @return true if nothing unexpected occurs
	 */
	bool setPidThreshold(Vector PidThresholds);

	/**
	 * move fingers to pregrasp position. Interpolation between the 'pinch' and 'fist' pregrasp positions is realized via
	 * the init parameter 'prepose'
	 * @return true if nothing unexpected occurs
	 */
	bool move_to_pregrasp_pos();

	/**
	 * execute grasping. function 'doGrasp(bool* excludedJoints)' is called with standard values (joint 7 excluded)
	 */
	bool doGrasp();

	/**
     * execute grasping. While the fingers are moving, the current and the pid controller errors are checked in order to detect
     * a contact between the fingers and the object, that is to be grasped. When contact is detected, the fingers are stopped.
     * The function exits when all finger joints are either in contact or have reached their target position.
	 * @param excludedJoints array fingers that are not to be checked because the detection doesn't work properly
     * @return true, when there seems to be an object between the fingers, otherwise false
     */
	bool doGrasp(bool* excludedJoints);

	/**
	 * send a position command to the hand. Function blocks until the position has been reached.
	 * Sets only joint 7..15 (hand)
	 * @param POS one float value for each joint (0..15).
	 * @return true if nothing unexpected occurs 
	 */
	bool position_move(Vector POS);

	/**
	 * send a velocity command to the hand. Sets only joint 7..15 (hand)
	 * @param velocity one float value for each joint (0..15).
     * @return true if nothing unexpected occurs
	 */
	bool velocity_move(Vector velocity);

	/**
	 * get the number of joints operated by this controller
	 * @return number of joints
	 */
	int get_joints();

	/**
	 * get index of the first joint of the hand
	 * @return index of the joint
	 */
	int get_startingJoint();

	/**
	 * read the latest position data from the encoder and store them in a local variable
	 * @return the encoder values 
	 */
	Vector get_encs();

	/**
     * read the latest current values from the encoder and store them in a local variable
     * @return the current values 
     */
	Vector get_amps();

	/**
     * read the latest velocity values from the encoder and store them in a local variable
     * @return the current values 
     */
	Vector get_velocity();

	/**
     * read the latest pid controller error values from the controller board and store them in a local variable
     * @return the current values
     */
	Vector get_pid();

	/**
	 * get information which joints are considered to be in contact. The information is generated by the function
	 * 'checkCollisions()'
	 * @return joints in contact (arm joints are ignored and thus false)
	 */
	bool *get_collisionDetected();

	/**
	 * get the pregrasp joint positions for both grasping styles 'pinch' and 'fist'
	 * @return 16x2 vector
	 */
	Vector *get_PrePose();

	/**
	 * get the interpolation parameter for the grasping position
	 * @return parameter beteween 0 (pinch) and 1 (fist)
	 */
	double get_endpose_param();

	/**
     * get the interpolation parameter for the pregrasp position
     * @return parameter beteween 0 (pinch) and 1 (fist)
     */
	double get_prepose_param();

	/**
	 * check current threshold for each joint. If the threshold is exceeded several times the joint is considered to
	 * be in contact. If the threshold is undercut several times the joint is considered to have lost contact. The
	 * result is stored in 'currentThresholdExceeded'.
	 * @return 0 if nothing unexpected occurs
	 */
    int checkCurrents();

	/**
     * check pid error threshold for each joint. If the threshold is exceeded several times the joint is considered to
     * be in contact. If the threshold is undercut several times the joint is considered to have lost contact. The
     * result is stored in 'pidThresholdExceeded'.
     * @return 0 if nothing unexpected occurs
     */
    int checkPidErrors();

	/**
     * check pid error as well as current threshold for each joint. If the threshold is exceeded several times the joint is considered to
     * be in contact. If the threshold is undercut several times the joint is considered to have lost contact. The
     * result is stored in 'collisionDetected'.
     * @return 0 if nothing unexpected occurs
     */
    int checkCollisions();

	/**
	 * compare the present encoder value to the target position for each joint. If the target position is either reached or exceeded the
	 * variable 'position_reached' is set true for this joint.
	 * @return 0 if nothing unexpected occurs
	 */
	int check_encoders(Vector target_positions);

	/**
	 * try to check if an object is in the gripper by the end of the movement. A successful grasp requires that all joints have stopped
	 * moving AND that all thumb joints are in contact AND that at least one of the other joints is in contact.
	 * @return true if grasping is assumed to be successful, else false
	 */
	bool contactAtEndOfGripping(bool* excludedJoints);

	/**
	 * auxiliary function to add two vectors
	 * @param A first vector
	 * @param B second vector
	 * @return the result of (A+B)
	 */
	Vector vectorAdd(Vector A, Vector B);

	/**
	 * auxiliary function to subtract two vectors
	 * @param A first vector
	 * @param B second vector
	 * @return the result of (A-B)
	 */
	Vector vectorSubtr(Vector A, Vector B);
	
	bool setPreposeParam(double pprepose);
	bool setEndposeParam(double pendpose);

	
};
