// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2008 Micha Hersch, EPFL
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   micha.hersch@robotcub.org
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
#ifndef __REACHING_H__
#define __REACHING_H__


using namespace std;
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <assert.h>



//#define OBSTACLE_AVOIDANCE


#define NEW_REACHING_STEP
//#define SILENT


#define ADAPT_VITE
#ifndef M_DEBUG
#define M_DEBUG
#endif


#define DYN_WEIGHTS


//#define PRINT_WEIGHTS
#define PRINT_TARGET

//#define BODY_SCHEMA

//#define MANY_JUMPS
//#define LEARNING
#define MAX_IT 500

//#define TRACE_COSTS
//#define REST_POS
//#define ADAPTIVE_REST_POS

/* #ifdef REST_POS */
/* #define MAX_IT 200 */
/* #endif */





/* #ifdef OBSTACLE_AVOIDANCE */
/* #define WITH_ENVIRONMENT */
/* #endif */


/* #ifdef WITH_ENVIRONMENT//OBSTACLE_AVOIDANCE */
/* class Reaching; */
/* class ReachingRight; */
/* class ReachingLeft; */
/* #include "Environment.h" */
/* #endif */



#include "BodySchema.h"



/**
 * This class implements the multi-referential VITE reaching algorithm for an arbitary number
 * of degrees of freedom. For being able to use it, you need to specify a BodySchema with
 * which to perform the reaching. The body schema contains all the geometrical information
 * about the limb you want to reach with
 * 
 */

class Reaching
{

 public:


  BodySchema *body;


  cart_vec_t shoulder_abs_pos;    // shoulder position

  float incr;                     // configManifold coarseness

  cart_vec_t target;              // target  relative to shoulder position
  float tol;                      // zero distance to target 
    float zero_vel_angle;            // zero angular velocity

    int pure_joint_ctl;
  cart_vec_t weight_cart;         // inverse of cost function weights for angular values
  joint_vec_t weight_angle;        // inverse of cost function weights for cartesian values

#ifdef DYN_WEIGHTS
  cart_vec_t base_weight_cart;         // base weights for angular values
  joint_vec_t base_weight_angle;        // base weights for cartesian values
#endif

#ifdef OBSTACLE_AVOIDANCE

  float obstacle_rad;
  float nu;
  joint_vec_t des_a_angle_obs; //acceleration du to obstacle potential
#endif


 protected:



  // vite variables
  float alpha;
  float beta;

  /**
   * angular velocity vector
   */
  joint_vec_t v_angle;   


  /**
   * cartesian velocity vector
   */
  cart_vec_t v_cart;    // 


  /**
   * desired angle position
   */
  joint_vec_t des_angle; // 


  /**
   * desired cartesian position
   */
  cart_vec_t des_cart;  // 


  /**
   * actual angle position
   */
  joint_vec_t pos_angle; // 


  /**
   *  actual cartesian position 
   */
  cart_vec_t pos_cart;  //


  /**
   * target angle position
   */
  joint_vec_t tar_angle; //


  /**
   * desired angular velocity vector
   */
  joint_vec_t des_v_angle; 


  /**
   * desired cartesian velocity vector
   */
  cart_vec_t des_v_cart;    
 

  
  //input-output stuff
 
  /**
   * for streaming  into the output file
   */
 ofstream *out_str; //for streaming out
  

 
 public:

    /**
     * constructor
     */
    Reaching();
   
    /**
     * destructor
     */
    virtual ~Reaching();

    /**
     * sets the body schema
     */
    void  SetBodySchema(BodySchema *b);
   
    /**
     * retrieves the body schema
     * \return the BodySchema attached to this reaching
     */
    BodySchema* GetBodySchema();


    /**
     * sets the target angle
     * \param angles desired target angle
     */
    void SetTargetAngle(joint_vec_t& angles);

    /**
     * retrieves the target
     * \param pos where to put the target
     */
    void GetTarget(cart_vec_t& pos){pos=target;};

    /**
     * retrieves the target angle
     * \param joint where to put the target angle
     */
    void GetTargetAngle(joint_vec_t& joint){joint=tar_angle;}

    /**
     * sets a new target and computes the corresponding joint angle target
     * \param new_target target in local referential
     * \param strict if 0, pointing if target out if range, else no pointing
     * \return 1 if target is reachable
     * \return 0 otherwise
     */
    int SetLocalTarget(cart_vec_t& new_target,int strict=0);


    /**
     * checks whether the target has been reached
     * \return 1 if the target has been reached (position and velocities are within some
     * boundaries given by tol and zero_vel_angle), 0 otherwise
     */
  int TargetReached(){return (target-pos_cart).Norm2() < 2*tol*tol && v_angle.Norm2()<zero_vel_angle*zero_vel_angle;}
  
  /**
   * Performs a reaching step, using the Grossberg's VITE algorithm
   * on the cartesian and joint angle space. Unification is enforced
   * by constraining the coherence of joint angles and cartesian positions
   * (by use of lagrange multipliers)
   * 
   */
  void ReachingStep(float dt=1);

  /**
   * performs the vite dynamical system in cartesian space
   * \param target the target
   * \param pos the position at time t
   * \param speed the speed at time t
   * \param new_pos the position at time t+1 (output of the method)
   * \param new_speed the speed at time t+1 (output of the method
   */
  void ViteCart(cart_vec_t& target,cart_vec_t& pos,cart_vec_t& speed,
       cart_vec_t& new_pos, cart_vec_t& new_speed);
  
    /**
     * performs the vite dynamical system in joint space
     * \param target the target
     * \param pos the position at time t
     * \param speed the speed at time t
     * \param new_pos the position at time t+1 (output of the method)
     * \param new_speed the speed at time t+1 (output of the method
     */  
    void ViteAngle(joint_vec_t& target,joint_vec_t& pos,joint_vec_t& speed,
       joint_vec_t& new_pos, joint_vec_t& new_speed);

    /**
     * projects solves the constrained optimization problem, i.e., projects
     * the desired positions on the manifold of consistent arm configuration and
     * end-effector positions
     * \param v3 desired end-effector position
     * \param v4 desired joint angle position
     * \param out where to put the resulting joint angle position
     */
  void ProjectVector(cart_vec_t& v3,joint_vec_t& v4, joint_vec_t& out);


    /**
     * retrieves the joint angle and cartesian position
     * \param cart where to put the cartesian position
     * \param joint where to put the joint angle position
     */
  void GetPosition(cart_vec_t& cart,joint_vec_t& joint){cart=pos_cart;joint=pos_angle;}
    
    /**
     * retrieves the joint angle position
     * \param joint where to put the joint angle position
     */
    void GetAnglePosition(joint_vec_t& joint){joint = pos_angle;}
 
    /**
     * sets the position the the one of the body schema
     */
    void MatchToBodySchema();

    /**
     * sets a random target
     * \param strict should be one if pointing is not allowed, 0 otherwise
     */
    void RandomTarget(int strict=1){cart_vec_t p;RandomTarget(p,strict);}
    

    /**
     * sets a random target
     * \param strict should be one if pointing is not allowed, 0 otherwise
     * \param tar the resulting target
     */
void RandomTarget(cart_vec_t& tar,int strict=1);
  

  /**
   * Updates the target in joint angle coordinates and in cartesian coordinates (if an
   * argument is given) as a function of actual postion <pos_angle>. 
   * @param new_target the new target (if changed), assumed to be very closed from the previous
   * one.
   * @return the discrepancy (distance) between joint angle and cartesian targets, a negative
   * number if the new target is not close enough from the old one.
   */
  float UpdateLocalTarget(cart_vec_t *new_target = NULL);


    /**
     * Stops reaching
     */
    void SetStill();
    

    /**
     * Use a pure joint angle controller, no cartesian space controller
     */
    void PureJointControl(){pure_joint_ctl=1;};

    /**
     * Use a hybrid joint angle and cartesian space controller
     */
    void HybridControl(){pure_joint_ctl=0;};

    /**
     * Prints the position, and the target to cout
     */
    void Print(){cout<<pos_cart<<" : "<<target<<" -  "<<pos_angle<<" : "<<tar_angle<<endl;}

 protected:

  void UpdateWeights();


#ifdef NOT_YET




  /**
   * Computes all the joint angle positions that
   * correpond to a target location. If the target
   * is too far, computes the positions that point
   * to the target. A sampling is performed, whose
   * coarsness in defined by <incr>
   * @param target cartesian target relative to shoulder position
   * (same orientation as SetTarget)
   * @return 1 if the target is reachable
   * @return 0 if the target is not reachable
  */
  virtual int ComputeConfigManifold(cart_vec_t& target);
  
  /**
   * Inverse kinematics method based on florent's
   * method. If the target is too far, the method will return
   * a position that points to the target (will reach a point on
   * the shoulder-target axis
   * @param target : target (in cartesian coordinates, 
   * eric's referential, origin at shoulder)
   * @param alpha : angle with the vertical (parameter of the solution curve)
   * @param out: where the solution is put
   *
   * @return 1 if the target is reachable or pointable while keeping 
   * in the joint angle bounds
   * @return 0. Otherwise.
   *
   * Note: this method is not the cleanest, could be implemented using
   * quaternions (cf Hestenes)
   */  
int InvKinematics (cart_vec_t& target, float alpha, float tol, pArmConfig_t out);


/**
 * Converts joint angles from Icub degree frame of ref to Robota rad frame of
 * ref.
 * @param angle the array of 4 angles
 */
 void Icub2Rob(joint_vec_t& angles);


/**
 * Converts joint angles from Robota rad frame of ref to Icub degree frame of
 * ref.
 * @param angle the array of 4 angles
 */
 void Rob2Icub(joint_vec_t& angles);


  /**
   * Converts joint angles from Hoap deg referential to 
   * Robota rad referential
   * @param conf the structure in which the angles are replaced
   */
  void Hoap2Rob(pArmConfig_t conf);


  /**
   * Converts joint angles from Hoap deg referential to 
   * Robota rad referential
   * @param conf the array in which the angles are replaced
   */
  void Hoap2Rob(joint_vec_t& conf);

  /**
   * Converts joint angles from  Robota rad to Hoap deg referential 
   * @param conf the structure in which the angles are replaced
   */
  void Rob2Hoap(pArmConfig_t conf);

  /**
   * Converts joint angles from  Robota rad to Hoap deg referential 
   * @param conf the structure in which the angles are replaced
   */
  void Rob2Hoap(joint_vec_t& conf);

  /**
   * Performs the forward kinematic function
   * @param config joint angle configuration in Robota rad referential
   * @param out output in Eric's referential relative to shoulder
   */
  virtual void Angle2Cart(pArmConfig_t config, cart_vec_t& out);
  

  /**
   * Performs the forward kinematic function
   * Input in Robota rad referential
   * Output in Eric's referential relative to shoulder
   */
  virtual void Angle2Cart(joint_vec_t& config, cart_vec_t& out);

  /**
   * Performs the forward kinematic function
   * Input in Robota rad referential
   * Output in global Eric's referential
   */
  void Angle2CartAbs(joint_vec_t& config, cart_vec_t& out);

  void ElbowPosition(joint_vec_t& config, cart_vec_t& out);

  /**
   * @brief  sets the arm length - and changes the weights accordingly
   * @param desired upper arm length
   * @param desired forearm length
   */
  void SetArmLength(float l1,float l2);
  void SetWeights(cart_vec_t& w_cart, joint_vec_t& w_angle);
  void SetIncrement(float new_increment);

  void DecrementAngleWeights(float factor=2.0);
  void IncrementAngleWeights(float factor=2.0);
  void IncrementCartWeights(float factor=2.0);// beware, not to set to 0.

  /**
   * sets a new target and computes the corresponding joint angle target
   * @param new_target target in global referential
   * @return 1 if target is reachable
   * @return -1 if target is pointable
   * @return 0 otherwise
   */
  virtual int SetTarget(cart_vec_t& new_target,int strict=0);


  int SetArmConfigurationTarget(joint_vec_t& new_target);

  int UpdateTarget();

 
  /**
   * Finds the nearest joint angle position to <pos1> out of
   *  the set of positions that are in the configManifold
   * Closness is measured using the euclidean distance if UPPER_ARM_PRIORITY
   * is not defined.  If it is only the two first angles are considered
   * See AngleDistance for more information.
   * Note: One could have a distance measure which takes into account
   * the physical world (energy, work, ...)
   * This is a brute force method (trying all possibilities). One could
   * think of something smarter if one has an inverse kinematics function
   * @param pos1 the distance to which the candidates are evaluated
   * @param out where the result is put 
   * @return the squared euclidean distance to target in joint angle space
   */
  float FindNearestPosition(pArmConfig_t pos1,pArmConfig_t out);

  /**
   * computes the squared euclidean distance between two
   * joint angle configurations
   * @param pos1 first joint angle configuration
   * @param pos2 second joint angle configuration
   * @return the squared euclidiean distance between the two configurations
   * (if ifdef UPPER_ARM_PRIORITY only she et saa are considered)
   */ 
  float AngleDistance(pArmConfig_t pos1,pArmConfig_t pos2);
  /**
   * computes the squared euclidean distance between two
   * joint angle configurations
   * @param pos1 first joint angle configuration
   * @param pos2 second joint angle configuration
   * @return the squared euclidiean distance between the two configurations
   * (if ifdef UPPER_ARM_PRIORITY only she et saa are considered)
   */ 
  float AngleDistance(joint_vec_t& pos1,joint_vec_t& pos2);

  /**
   * computes the jacobian of the kinematic function
   * valid for eric's frame of ref and robota rad frame of ref.
   * @param angle the angle position at which to compute
   * the Jacobian matrix
   * @param jac a 4x4 matrix filled up by the method
   * the last row contains zeros and 1 on the diagonal.
  */
  virtual void Jacobian(joint_vec_t& angle, CMatrix4_t jac);

/**
 * method IntermediateJacobian
 *
 * computes the Jacobian for a point on the arm.
 * valid for eric's referential and robota rad ref. 
 * args:
 * --- 
 * @param link 1 for the upper arm, 2 for the foremarm
 * @param len length of that link (from joint to the point considered)
 * @param v the angle position at which to compute
 * the Jacobian matrix
 * @param jac  a 4x4 matrix filled up by the method
 * the last row contains zeros and 1 on the diagonal.
 *     
 */
  void IntermediateJacobian(int link, float len,joint_vec_t& v, CMatrix4_t jac);
  int ArmConfig2CVector(pArmConfig_t arm,joint_vec_t& out);
  int CVector2ArmConfig(joint_vec_t& v,pArmConfig_t out);
#ifdef TRACE_COSTS
  float GetCosts(joint_vec_t& des_angle,cart_vec_t& des_cart);
  float ReachingStep();
#else
/**
 * method ReachingStep
 *
 * Performs a reaching step, using the Grossberg's VITE algorithm
 * on the cartesian and joint angle space. Unification is enforced
 * by constraining the coherence of joint angles and cartesian positions
 * (by use of lagrange multipliers)
*/
 
#endif
#ifndef REACH_MASTER
#ifndef LIGHT_VERSION  
  void RetrieveJointAngles(pHoap2_JointAngles angles);
  int SetActualHoapPosition(pHoap2_JointAngles angles);
#endif  
#endif
  
  

 /**
  * @param angles the new position in rad Robota frame of reference 
  * @return 1 if the new position is within the workspace boundaries
  * @return 0 otherwise (in that case nothing happens)
  */
  int SetActualRobPosition(joint_vec_t& angles);


  /**
   * @param position the new position in rad Robota frame of reference 
   * @param speed the new joint angle speed 
   * @return 1 if the new position is within the workspace boundaries
   * and 0 otherwise (in that case nothing happens)
   */
  int SetActualRobPosAndSpeed(joint_vec_t& position, joint_vec_t& speed);

  /**
   * @param acc the new acceleration in rad Robota frame of reference 
   * @return 1 if the new position is within the workspace boundaries
   * @return 0 otherwise (in that case the position is put on the boundaries)
   */
  int SetRobAcceleration(joint_vec_t& angle_acc);
  
  /**
   * sets the shoulder position with respoect to an 
   * absolute referential (the same one as SetTarget).
   * this referential is oriented like Eric's (x:left,
   * y: up, z:front)
   * @param position the position of the shoulder according to an external
   * frame of reference
   */
  void SetShoulderPosition(cart_vec_t& position);
  
  /**
   * tests if a configuration is within the joint angle boundaries
   * @param angle the confiration to be tested 
   * @returns  1 if <angle> is within workspace boundaries
   * @return 0 otherwise.
   */
  int AcceptablePosition(joint_vec_t& angle);

  /**
   * writes the arm configuration and target location in
   * a file. Format: "x y z: th1 th2 th3 th4" (x,y,z = target location)
   * @param fname the filename where to write the arm configuration
   * and target location (in shoulder-centered, erics referential)
   * @return: 1 if sucessful writing, 0 otherwise
   */
  int SaveConfig(string *fname);
  int InitOutputStream(string& filename, ofstream **ostr);
  
/**
   *@brief performs trajectories specified in a file and writes the result
   * into another file
   * @param infname name of file specifying the initial arm configuration and
   * the target location (one trajectory per line)
   * @param outfname file basename where to write the resulting trajectories
   * @param max_traj max number of trajectories to execute (unbounded if 
   * negative)
   */

  int PerformTrajectories(string *infname, string *outfname, int max_traj=-1);

/**
 * @param infname the file name specifying the initial position, the
 *  target location, and the external perturbation. The file has the following
 * format:
 * target(3 numbers) : initial joint angle configuration (4 numbers)
 * time (1 number): joint angle acceleration (4 numbers)
 * @param outfname the file name where to write the resulting trajectory
 * @return 1 if the target is reached, 0 if the target is not reached, a
 * negative number in case of error
 */
  int PerformPerturbedTrajectory(string *infname, string *outfname);
  int RecordTraj(joint_vec_t& initpos, cart_vec_t& target,string& fname);
  int RecordTraj_ACT(joint_vec_t& initpos, joint_vec_t& target,string& fname);

  /**
   * records a trajectory and write the output into a file
   * @param initpos the inital joint angle configuration
   * @param out a stream to write the output
   * @return 0 if the target was reached
   * @return 1 if the target was not reached
   * @return -1 if the initial position is not valid 
   */
  int StartRecordTraj(joint_vec_t& initpos,ofstream& out); 

  /**
   * performs a reaching trajectory
   * @param initpos the inital joint angle configuration
   * @return 0 if the target was reached
   * @return 1 if the target was not reached
   * @return -1 if the initial position is not valid 
   */
  int StartRecordTraj(joint_vec_t& initpos); // no trajectory output
 
  /**
   * performs a reaching trajectory from its actual position
   * @return 0 if the target was reached
   * @return 1 if the target was not reached
   */
  int Reach();

  /**
   * Indicates whether a target is reached.
   * The target is considered to be reached if within a distance of <tol>
   * from end-effector. This method doesn't consider speed and could be
   * changed accordingly
   * @return 1 if target is reached
   * @return 0 otherwise
   */
  int TargetReached();
  int HasStopped();

  void RandomAnglePos(joint_vec_t& angle);
  void RandomCartPos(cart_vec_t& pos);
  void RandomCartAbsPos(cart_vec_t& pos);

  // joint_vec_t& GetAngle();

  /**
   * gives the current angular position in robota rad frame of ref.
   * @param out the vector in which to put the current angular position
   */
  void GetAngle(joint_vec_t& out) const ;
 

  /**
 * gets the target in a global frame of ref
 *@param tar where to put the resut
 */
  void GetTarget(cart_vec_t& tar) const;

 /**
 * gets the target in a local frame of ref
 *@param tar where to put the resut
 */
  void GetLocalTarget(cart_vec_t& tar) const {v_copy(target,tar);}

  void GetWeights(cart_vec_t& w_cart, joint_vec_t& w_angle) const;
 
/**
   * gives the angular target position in robota rad frame of ref.
   * @param out the vector in which to put the current angular position
   */
  virtual void GetTargetAngle(joint_vec_t& out) const ;
  /**
   * transforms a position from a local shoulder
   * centered frame of ref into a global . For now, it's only a translation,
   * no rotation
   * (would be needed when using the torso)
   * @param cart_l location in local frame of ref.
   * @param cart output, i.e.,location in global frame.of ref
   */

  virtual void Local2AbsRef(const cart_vec_t& cart_l, cart_vec_t& cart) const;
  
  /**
   * transforms a position from a global frame of ref, into a shoulder
   * centered frame of ref. For now, it's only a translation, no rotation
   * (would be needed when using the torso)
   * @param cart location in global frame of ref.
   * @param output, i.e.,location in local frame.of ref
   */

  virtual void Abs2LocalRef(const cart_vec_t& cart, cart_vec_t& out) const;

  /**
   * gives the current cartesian position in a global ref.
   * @param out the vector in which to put the current cartesian position
   * of end-effector
   */
  void GetAbsHandPosition(cart_vec_t& out) const;

  /**
   * gives the current cartesian position in a local ref.
   * @param out the vector in which to put the current cartesian position
   * of end-effector
   */
  void GetLocalHandPosition(cart_vec_t& out) const;


  /**
   * indicates how far from a spurious attractor the arm is
   * @return a number indicating how far we are from a
   * spurious attractor
   * 
   */
  float SpuriousAttractorIndex();


  virtual int GetVisualMode();

#ifdef LEARNING
  //put the function necessary for learning here
  float LearningLocal();
  float LearningGlobal();

#endif

#ifdef WITH_ENVIRONMENT
  int SetEnvironment(Environment *env);
#endif

#ifdef OBSTACLE_AVOIDANCE
  int LinkDistanceToObject(int link, EnvironmentObject *obj, float *dist, 
			   float *point, cart_vec_t& contact_vector);
  int FindSegmentsNearestPoints(cart_vec_t& p1,cart_vec_t& vec1,cart_vec_t& p2,cart_vec_t& vec2, float *nearest_point1, float *nearest_point2, float *dist);
#endif

#ifdef DYN_WEIGHTS_OLD
  void ResetWeights();
#endif

#ifdef REST_POS
  void SetRestPosWeight(joint_vec_t& w);
#endif

#ifdef DYN_WEIGHTS
#ifdef PARAM_COS
  void SetWeightFunction(float param);
#endif
#endif

 protected:

  void UpdateWeights();
  void SetAnglesInRange(joint_vec_t& angle);
};
// external operators
/**
 * out streaming operator
 */ 
ostream& operator<<(ostream& out, const Reaching& reach);
/**
 * in streaming operator
 */ 
istream& operator>>(istream& in, Reaching& reach);


ostream& operator<<(ostream& out, const ArmConfigList_t manifold);

typedef Reaching *pReaching;


class ReachingRight : public Reaching {
	public :
	ReachingRight();
	~ReachingRight();
#ifndef LIGHT_VERSION  
  	void RetrieveJointAngles(pHoap2_JointAngles angles);
  	int SetActualHoapPosition(pHoap2_JointAngles angles);
#endif
};


/**
 * @brief same as Reaching, but for the left hand
 */
class ReachingLeft : public Reaching {
 protected:
  void Real2VirtualAngles(joint_vec_t& angles)const;
  void Virtual2RealAngles(joint_vec_t& angles)const;
  public :
    ReachingLeft();
  ~ReachingLeft();
#ifndef LIGHT_VERSION  
  void RetrieveJointAngles(pHoap2_JointAngles angles);
  int SetActualHoapPosition(pHoap2_JointAngles angles);
#endif
  //int SetTarget(cart_vec_t& new_target);
  virtual void Local2AbsRef(const cart_vec_t& cart_l, cart_vec_t& cart) const;
  virtual void Abs2LocalRef(const cart_vec_t& cart, cart_vec_t& out) const;
  virtual void GetTargetAngle(joint_vec_t& angle) const;
  //  virtual void Angle2Cart(joint_vec_t& angle, cart_vec_t& pos);


#endif //NOT_YET
};




#endif

