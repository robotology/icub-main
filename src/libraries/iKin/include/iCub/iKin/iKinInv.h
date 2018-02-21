/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

/**
 * \defgroup iKinInv iKinInv
 *  
 * @ingroup iKin 
 *
 * Classes for inverse kinematics of serial-links chains and 
 * iCub limbs 
 *
 * Date: first release 16/06/2008
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKININV_H__
#define __IKININV_H__

#include <yarp/os/Property.h>
#include <iCub/ctrl/minJerkCtrl.h>
#include <iCub/ctrl/pids.h>
#include <iCub/iKin/iKinFwd.h>

#define IKINCTRL_STATE_RUNNING      0
#define IKINCTRL_STATE_INTARGET     1
#define IKINCTRL_STATE_DEADLOCK     2

#define IKINCTRL_POSE_FULL          0
#define IKINCTRL_POSE_XYZ           1
#define IKINCTRL_POSE_ANG           2

#define IKINCTRL_STEEP_JT           0
#define IKINCTRL_STEEP_PINV         1

#define IKINCTRL_RET_TOLX           0
#define IKINCTRL_RET_TOLSIZE        1
#define IKINCTRL_RET_TOLQ           2
#define IKINCTRL_RET_MAXITER        3
#define IKINCTRL_RET_EXHALT         4
                                    
#define IKINCTRL_DISABLED           -1


namespace iCub
{

namespace iKin
{

/**
* \ingroup iKinInv
*
* Abstract class for inverting chain's kinematics.
*/
class iKinCtrl
{
private:
    // Default constructor: not implemented.
    iKinCtrl();
    // Copy constructor: not implemented.
    iKinCtrl(const iKinCtrl&);
    // Assignment operator: not implemented.
    iKinCtrl &operator=(const iKinCtrl&);

protected:
    iKinChain &chain;
    unsigned int ctrlPose;

    yarp::sig::Vector x_set;
    yarp::sig::Vector x;
    yarp::sig::Vector e;
    yarp::sig::Vector q;
    yarp::sig::Matrix J;
    yarp::sig::Matrix Jt;
    yarp::sig::Matrix pinvJ;
    yarp::sig::Vector grad;

    yarp::sig::Vector q_old;

    double inTargetTol;
    double watchDogTol;

    unsigned int dim;
    unsigned int iter;

    int  state;

    bool watchDogOn;
    int  watchDogCnt;
    int  watchDogMaxIter;

    /**
    * Computes the error according to the current controller
    * settings (complete pose/translational/rotational part). 
    * Note that x must be previously set.
    * @return the error.
    */
    virtual yarp::sig::Vector calc_e();

    /**
    * Updates the control state.
    */
    virtual void update_state();

    /**
    * Handles the watchDog.
    */
    virtual void watchDog();

    /**
    * Checks each joint velocity and sets it to zero if it steers 
    * the joint out of range. 
    * @param _qdot is the joint velocities vector to be checked. 
    * @param _Ts is the joint velocities sample time.
    * @return the new velocity. 
    */
    virtual yarp::sig::Vector checkVelocity(const yarp::sig::Vector &_qdot, double _Ts);

    /**
    * Method called whenever in target. 
    * Shall be implemented. 
    */
    virtual void inTargetFcn() = 0;

    /**
    * Method called whenever the watchDog is triggered. Put here the
    * code to recover from deadLock. Shall be implemented. 
    */
    virtual void deadLockRecoveryFcn() = 0;

    /**
    * Dumps warning or status messages.
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @note Angles are dumperd as degrees. 
    * Shall be implemented. 
    */
    virtual void printIter(const unsigned int verbose=0) = 0;

    /**
    * Method to be called within the printIter routine inherited by 
    * children in order to handle the highest word of verbose 
    * integer. 
    */
    unsigned int printHandling(const unsigned int verbose=0);

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled.
    */
    iKinCtrl(iKinChain &c, unsigned int _ctrlPose);

    /**
    * Enables/Disables joint angles constraints.
    * @param _constrained if true then constraints are applied.
    */
    virtual void setChainConstraints(bool _constrained) { chain.setAllConstraints(_constrained); }

    /**
    * Executes one iteration of the control algorithm 
    * @param xd is the End-Effector target Pose to be tracked.
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @return current estimation of joints configuration.
    * Shall be implemented. 
    */
    virtual yarp::sig::Vector iterate(yarp::sig::Vector &xd, const unsigned int verbose=0) = 0;

    /**
    * Iterates the control algorithm trying to converge on the 
    * target. 
    * @param xd is the End-Effector target Pose to be tracked. 
    * @param tol_size exits if test_convergence(tol_size) is true 
    *                 (tol_size<0 disables this check, default).
    * @param max_iter exits if iter>=max_iter (max_iter<0 disables
    *                 this check, default).
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @param exit_code stores the exit code (NULL by default). Test 
    *                  for one of this:
    *                 IKINCTRL_RET_TOLX
    *                 IKINCTRL_RET_TOLSIZE
    *                 IKINCTRL_RET_TOLQ
    *                 IKINCTRL_RET_MAXITER
    *                 IKINCTRL_RET_EXHALT
    * @param exhalt checks for an external request to exit (NULL by
    *               default).
    * @see setInTargetTol
    * @see getInTargetTol
    */
    virtual yarp::sig::Vector solve(yarp::sig::Vector &xd, const double tol_size=IKINCTRL_DISABLED,
                                    const int max_iter=IKINCTRL_DISABLED, const unsigned int verbose=0,
                                    int *exit_code=NULL, bool *exhalt=NULL);

    /**
    * Tests convergence by comparing the size of the algorithm 
    * internal structure (may be the gradient norm or the simplex
    * size or whatever) to a certain tolerance. 
    * @param tol_size is tolerance to compare to.
    * Shall be implemented.
    */
    virtual bool test_convergence(const double tol_size) = 0;

    /**
    * Reinitializes the algorithm's internal state and resets the 
    * starting point. 
    * @param q0 is the new starting point. 
    */
    virtual void restart(const yarp::sig::Vector &q0);

    /**
    * Returns the algorithm's name.
    * @return algorithm name as string.
    * Shall be implemented. 
    */
    virtual std::string getAlgoName() = 0;

    /**
    * Switch on/off the watchDog mechanism to trigger deadLocks. 
    * A deadLock is triggered whenerver norm(q(k)-q(k-1))<tol_q for 
    * a specified number of iterations. 
    * @param sw control the watchDog activation. 
    */
    void switchWatchDog(bool sw) { watchDogOn=sw; }

    /**
    * Sets tolerance for in-target check (5e-3 by default). 
    * @param tol_x is the tolerance
    */
    virtual void setInTargetTol(double tol_x) { inTargetTol=tol_x; }

    /**
    * Returns tolerance for in-target check. 
    * @return tolerance
    */
    virtual double getInTargetTol() const { return inTargetTol; }

    /**
    * Sets tolerance for watchDog check (1e-4 by default). 
    * @param tol_q is the tolerance
    */
    virtual void setWatchDogTol(double tol_q) { watchDogTol=tol_q; }

    /**
    * Returns tolerance for watchDog check. 
    * @return tolerance
    */
    virtual double getWatchDogTol() const { return watchDogTol; }

    /**
    * Sets maximum number of iterations to trigger the watchDog (200
    * by default). 
    * @param maxIter is the iterations limit.
    */
    virtual void setWatchDogMaxIter(int maxIter) { watchDogMaxIter=maxIter; }

    /**
    * Returns maximum number of iterations to trigger the watchDog
    * @return iterations limit.
    */
    virtual int getWatchDogMaxIter() const { return watchDogMaxIter; }

    /**
    * Checks if the End-Effector is in target. 
    * @return true if in target.
    */
    virtual bool isInTarget() { return dist()<inTargetTol; }
                                                                          
    /**
    * Returns the algorithm's state.
    * @return algorithm's state:
    * IKINCTRL_STATE_RUNNING 
    * IKINCTRL_STATE_INTARGET 
    * IKINCTRL_STATE_DEADLOCK 
    */
    int get_state() const { return state; }

    /**
    * Sets the state of Pose control settings.
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled.
    */
    void set_ctrlPose(unsigned int _ctrlPose);

    /**
    * Returns the state of Pose control settings.
    * @return Pose control settings.
    */
    unsigned int get_ctrlPose() const { return ctrlPose; }

    /**
    * Returns the number of Chain DOF
    * @return number of Chain DOF.
    */
    unsigned int get_dim() const { return dim; }

    /**
    * Returns the number of performed iterations.
    * @return number of performed iterations.
    */
    unsigned int get_iter() const { return iter; }

    /**
    * Returns the actual cartesian position of the End-Effector.
    * @return actual cartesian position of the End-Effector.
    */
    virtual yarp::sig::Vector get_x() const { return x; }

    /**
    * Returns the actual cartesian position error.
    * @return actual cartesian position error.
    */
    virtual yarp::sig::Vector get_e() const { return e; }

    /**
    * Sets the joint angles values.
    * @param q0 is the joint angles vector. 
    */
    virtual void set_q(const yarp::sig::Vector &q0);

    /**
    * Returns the actual joint angles values.
    * @return actual joint angles values.
    */
    virtual yarp::sig::Vector get_q() const { return q; }

    /**
    * Returns the actual gradient.
    * @return actual gradient.
    */
    virtual yarp::sig::Vector get_grad() const { return grad; }

    /**
    * Returns the actual Jacobian used in computation.
    * @return actual Jacobian.
    */
    virtual yarp::sig::Matrix get_J() const { return J; }

    /**
    * Returns the actual distance from the target in cartesian space
    * (euclidean norm is used). 
    * @return actual distance from the target.
    */
    virtual double dist() const { return yarp::math::norm(e); }

    /**
    * Default destructor.
    */
    virtual ~iKinCtrl() { }
};


/**
* \ingroup iKinInv
*
* A class derived from iKinCtrl implementing two standard 
* algorithms based on steepest descent qdot=-Kp*grad. 
* 1) grad=-Jt*e 
* 2) grad=-pinv(J)*e. 
*/
class SteepCtrl : public iKinCtrl
{
private:
    // Default constructor: not implemented.
    SteepCtrl();
    // Copy constructor: not implemented.
    SteepCtrl(const SteepCtrl&);
    // Assignment operator: not implemented.
    SteepCtrl &operator=(const SteepCtrl&);

protected:
    double Ts;
    ctrl::Integrator *I;

    unsigned int type;
    bool constrained;

    double Kp;
    yarp::sig::Vector qdot;
    yarp::sig::Vector gpm;

    virtual void inTargetFcn()         { }
    virtual void deadLockRecoveryFcn() { }
    virtual void printIter(const unsigned int verbose);
    virtual yarp::sig::Vector update_qdot();

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _type one of the following: 
    *  IKINCTRL_STEEP_JT   => implements J transposed method.
    *  IKINCTRL_STEEP_PINV => implements J pseudo-inverse method.
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled.
    * @param _Ts is the controller sample time.
    * @param _Kp is constant gain.
    */
    SteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose,
              double _Ts, double _Kp);

    /**
    * Returns the further contribution to the qdot=pinvJ*xdot 
    * equation according to the Gradient Projection Method, i.e. 
    * qdot=pinvJ*xdot+(I-pinvJ*J)*w 
    * @return shall return the quantity (I-pinvJ*J)*w. 
    * @note This method shall be inherited and handled accordingly 
    *       (here a vector of 0s is returned). To do that, J and
    *       pinvJ are already computed when this method is called.
    */
    virtual yarp::sig::Vector computeGPM() { return yarp::sig::Vector(dim,0.0); }

    virtual void setChainConstraints(bool _constrained);
    virtual yarp::sig::Vector iterate(yarp::sig::Vector &xd, const unsigned int verbose=0);
    virtual void restart(const yarp::sig::Vector &q0);
    virtual bool test_convergence(const double tol_size) { return yarp::math::norm(grad)<tol_size; }
    virtual std::string getAlgoName()                    { return "steepest-descent";              }

    /**
    * Resets integral status at the current joint angles.
    */
    void resetInt() { I->reset(q); }

    /**
    * Returns the actual derivative of joint angles.
    * @return the actual derivative of joint angles. 
    */
    yarp::sig::Vector get_qdot() const { return qdot; }

    /**
    * Returns the actual value of Gradient Projected.
    * @return the actual value of Gradient Projected.
    */
    yarp::sig::Vector get_gpm() const { return gpm; }

    /**
    * Returns the gain.
    * @return the gain.
    */
    double get_Kp() const { return Kp; }

    /**
    * Destructor.
    */                                                         
    virtual ~SteepCtrl();
};


/**
* \ingroup iKinInv
*
* A class derived from SteepCtrl implementing the variable gain 
* algorithm 
*  
* r(k)=dist(k)/dist(k-1) 
* r(k)<1 => Kp(k)=Kp(k-1)*Kp_inc; 
* r(k)>max_per_inc => Kp(k)=Kp(k-1)*Kp_dec;
*/
class VarKpSteepCtrl : public SteepCtrl
{
private:
    // Default constructor: not implemented.
    VarKpSteepCtrl();
    // Copy constructor: not implemented.
    VarKpSteepCtrl(const VarKpSteepCtrl&);
    // Assignment operator: not implemented.
    VarKpSteepCtrl &operator=(const VarKpSteepCtrl&);

protected:
    double Kp0;
    double Kp_inc;
    double Kp_dec;
    double Kp_max;
    double max_perf_inc;

    double dist_old;

    void         reset_Kp();
    virtual void inTargetFcn() { reset_Kp(); }
    virtual yarp::sig::Vector update_qdot();

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _type one of the following: 
    *  IKINCTRL_STEEP_JT   => implements J transposed method.
    *  IKINCTRL_STEEP_PINV => implements J pseudo-inverse method.
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled.
    * @param _Ts is the controller sample time.
    * @param _Kp0 is the initial gain. 
    * @param _Kp_inc is the increasing factor.
    * @param _Kp_dec is the drecreasing factor.
    * @param _Kp_max is the maximum value for Kp.
    * @param _max_perf_inc is the threshold value to decreas Kp. 
                                                                */
    VarKpSteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose, double _Ts,
                   double _Kp0, double _Kp_inc, double _Kp_dec, double _Kp_max, double _max_perf_inc);

    virtual void restart(const yarp::sig::Vector &q0) { SteepCtrl::restart(q0); reset_Kp(); }
    virtual std::string getAlgoName() { return "variable-gain-steepest-descent"; }
};


/**
* \ingroup iKinInv
*
* A class derived from iKinCtrl implementing the
* Levenberg-Marquardt algorithm: 
*  
* qdot=-pinv(Jt*J+mu*I)*grad 
*
* r(k)=dist(k)/dist(k-1)
* r(k)<1 => mu(k)=mu(k-1)*mu_dec;
* r(k)>1 => mu(k)=mu(k-1)*mu_inc;
*
* H=Jt*J is the approximation of Hessian matrix
*/
class LMCtrl : public iKinCtrl
{
private:
    // Default constructor: not implemented.
    LMCtrl();
    // Copy constructor: not implemented.
    LMCtrl(const LMCtrl&);
    // Assignment operator: not implemented.
    LMCtrl &operator=(const LMCtrl&);

protected:
    double Ts;
    ctrl::Integrator *I;

    bool constrained;

    yarp::sig::Vector qdot;
    yarp::sig::Vector gpm;
    yarp::sig::Matrix pinvLM;

    double mu;
    double mu0;
    double mu_inc;
    double mu_dec;
    double mu_min;
    double mu_max;

    double dist_old;
    double svMin;
    double svThres;
    
    virtual double update_mu();
    virtual void   inTargetFcn()         { }
    virtual void   deadLockRecoveryFcn() { }
    virtual void   printIter(const unsigned int verbose);

    virtual yarp::sig::Matrix pinv(const yarp::sig::Matrix &A, const double tol=0.0);

public:
    /**
    * Constructor.
    * @param c is the Chain object on which the control operates. Do
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following:
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled.
    * @param _Ts is the controller sample time. 
    * @param _mu0 is the initial value for weighting factor mu.
    * @param _mu_inc is the increasing factor.
    * @param _mu_dec is the drecreasing factor.
    * @param _mu_min is the minimum value for mu.
    * @param _mu_max is the maximum value for mu. 
    * @param _sv_thres is the minimum singular value under which the
    *                mu is constantly kept equal to _mu_max.
    */
    LMCtrl(iKinChain &c, unsigned int _ctrlPose, double _Ts, double _mu0, double _mu_inc,
           double _mu_dec, double _mu_min, double _mu_max, double _sv_thres=1e-6);

    /**
    * Returns the further contribution to the qdot=pinvJ*xdot 
    * equation according to the Gradient Projection Method, i.e. 
    * qdot=pinvJ*xdot+(I-pinvJ*J)*w 
    * @return shall return the quantity (I-pinvJ*J)*w. 
    * @note This method shall be inherited and handled accordingly 
    *       (here a vector of 0s is returned). To do that, J and
    *       pinvJ are already computed when this method is called.
    *       The LM-inverse matrix pinvLM is also available.
    */
    virtual yarp::sig::Vector computeGPM() { return yarp::sig::Vector(dim,0.0); }

    virtual void setChainConstraints(bool _constrained);
    virtual yarp::sig::Vector iterate(yarp::sig::Vector &xd, const unsigned int verbose=0);
    virtual void restart(const yarp::sig::Vector &q0);
    virtual bool test_convergence(const double tol_size) { return yarp::math::norm(grad)<tol_size; }
    virtual std::string getAlgoName()                    { return "levenberg-marquardt";           }

    /**
    * Resets integral status at the current joint angles.
    */
    void resetInt() { I->reset(q); }

    /**
    * Returns the actual derivative of joint angles.
    * @return the actual derivative of joint angles. 
    */
    yarp::sig::Vector get_qdot() const { return qdot; }

    /**
    * Returns the actual value of Gradient Projected.
    * @return the actual value of Gradient Projected.
    */
    yarp::sig::Vector get_gpm() const { return gpm; }

    /**
    * Returns the current weighting factor mu.
    * @return the current weighting factor mu.
    */
    double get_mu() const { return mu; }

    /**
    * Sets the weighting factor mu equal to the initial value.
    */
    void reset_mu();

    /**
    * Destructor.
    */                                                         
    virtual ~LMCtrl();
};


/**
* \ingroup iKinInv
*
* A class derived from LMCtrl implementing the Gradient 
* Projection Method according to the paper available <a 
* href="http://robotics.hanyang.ac.kr/new/papers/TA02-4.pdf">here</a>. 
*/
class LMCtrl_GPM : public LMCtrl
{
private:
    // Default constructor: not implemented.
    LMCtrl_GPM();
    // Copy constructor: not implemented.
    LMCtrl_GPM(const LMCtrl_GPM&);
    // Assignment operator: not implemented.
    LMCtrl_GPM &operator=(const LMCtrl_GPM&);

protected:
    double safeAreaRatio;
    double K;

    yarp::sig::Vector span;
    yarp::sig::Vector alpha_min;
    yarp::sig::Vector alpha_max;
    yarp::sig::Matrix Eye;

public:
    /**
    * Constructor.
    */
    LMCtrl_GPM(iKinChain &c, unsigned int _ctrlPose, double _Ts, double _mu0, double _mu_inc,
               double _mu_dec, double _mu_min, double _mu_max, double _sv_thres=1e-6);

    virtual yarp::sig::Vector computeGPM();

    /**
    * Sets the GPM gain (shall be positive).
    * @param _K GPM gain.
    */
    void set_K(const double _K) { K=_K>0 ? _K : -_K; }

    /**
    * Returns the GPM gain (1.0 by default).
    * @return the GPM gain.
    */
    double get_K() const { return K; }

    /**
    * Sets the safe area ratio [0-1], which is for each joint the
    * ratio between the angle span within which the chain can be 
    * operated with 0-GPM and the overall angle span.
    * @param _safeAreaRatio the safe area ratio.
    */
    void set_safeAreaRatio(const double _safeAreaRatio);

    /**
    * Returns the safe area ratio (0.9 by default).
    * @return the safe area ratio.
    */
    double get_safeAreaRatio() const { return safeAreaRatio; }
};


/**
* \ingroup iKinInv
*
* A class derived from iKinCtrl implementing the 
* multi-referential approach (<a 
* href="http://wiki.icub.org/images/c/cf/CartesianControllersEvaluation.pdf">pdf</a>).
* @note Minimum-Jerk controllers in Task Space and Joint Space 
*/
class MultiRefMinJerkCtrl : public iKinCtrl
{
private:
    // Default constructor: not implemented.
    MultiRefMinJerkCtrl();
    // Copy constructor: not implemented.
    MultiRefMinJerkCtrl(const MultiRefMinJerkCtrl&);
    // Assignment operator: not implemented.
    MultiRefMinJerkCtrl &operator=(const MultiRefMinJerkCtrl&);

protected:
    ctrl::minJerkVelCtrl *mjCtrlJoint;
    ctrl::minJerkVelCtrl *mjCtrlTask;
    ctrl::Integrator     *I;

    yarp::sig::Vector q_set;
    yarp::sig::Vector qdot;
    yarp::sig::Vector xdot;
    yarp::sig::Matrix W;
    yarp::sig::Matrix Eye6;

    double Ts;
    double execTime;
    double gamma;
    double guardRatio;

    yarp::sig::Vector qGuard;
    yarp::sig::Vector qGuardMinInt, qGuardMinExt, qGuardMinCOG;
    yarp::sig::Vector qGuardMaxInt, qGuardMaxExt, qGuardMaxCOG;

    yarp::sig::Vector compensation;

    virtual void computeGuard();
    virtual void computeWeight();
    virtual yarp::sig::Vector iterate(yarp::sig::Vector &xd, yarp::sig::Vector &qd,
                                      yarp::sig::Vector *xdot_set, const unsigned int verbose);

    virtual void inTargetFcn()         { }
    virtual void deadLockRecoveryFcn() { }
    virtual void printIter(const unsigned int verbose);

    // disable unused father's methods
    virtual bool test_convergence(const double)                                  { return false; }
    virtual yarp::sig::Vector iterate(yarp::sig::Vector&, const unsigned int)    { return yarp::sig::Vector(0); }
    virtual yarp::sig::Vector solve(yarp::sig::Vector&, const double,
                                    const int, const unsigned int, int*, bool *) { return yarp::sig::Vector(0); }

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled.
    * @param _Ts is the nominal controller sample time. 
    * @param nonIdealPlant if true allocate a dedicated min-jerk 
    *                      controller for the configuration space
    *                      capable of compansating plants that
    *                      differ from pure integrators.
    */
    MultiRefMinJerkCtrl(iKinChain &c, unsigned int _ctrlPose, double _Ts, bool nonIdealPlant=false);

    /**
    * Executes one iteration of the control algorithm.
    * @param xd is the End-Effector target Pose to be tracked. 
    * @param qd is the target joint angles (it shall satisfy the
    *           forward kinematic function xd=f(qd)).
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @return current estimation of joints configuration. 
    * @note The reason why qd is provided externally instead of 
    *       computed here is to discouple the inverse kinematic
    *       problem (which may require some computational effort
    *       depending on the current pose xd) from the reaching
    *       issue. 
    */
    virtual yarp::sig::Vector iterate(yarp::sig::Vector &xd, yarp::sig::Vector &qd,
                                      const unsigned int verbose=0);

    /**
    * Executes one iteration of the control algorithm.
    * @param xd is the End-Effector target Pose to be tracked. 
    * @param qd is the target joint angles (it shall satisfy the
    *           forward kinematic function xd=f(qd)).
    * @param xdot_set is the Task Space reference velocity; the 
    *                 vector size is 7 (due to the axis-angle
    *                 notation) and units are [rad/s].
    * @param verbose is a integer whose 32 bits are intended as
    *                follows. The lowest word (16 bits)
    *                progressively enables different levels of
    *                warning messages or status dump: the larger
    *                this value the more detailed is the output
    *                (0x####0000=>off by default). The highest word
    *                indicates how many successive calls to the dump
    *                shall be skipped in order to reduce the amount
    *                of information on the screen (ex:
    *                0x0000####=>print all iterations,
    *                0x0001####=>print one iteration and skip the
    *                next one, 0x0002####=> print one iteration and
    *                skip the next two).
    * @return current estimation of joints configuration. 
    * @note The reason why qd is provided externally instead of 
    *       computed here is to discouple the inverse kinematic
    *       problem (which may require some computational effort
    *       depending on the current pose xd) from the reaching
    *       issue.  
    */
    virtual yarp::sig::Vector iterate(yarp::sig::Vector &xd, yarp::sig::Vector &qd,
                                      yarp::sig::Vector &xdot_set, const unsigned int verbose=0);

    virtual void restart(const yarp::sig::Vector &q0);

    virtual std::string getAlgoName() { return "multi-referential-minimum-jerk-controllers"; }

    /**
    * Returns the guard ratio for the joints span (0.1 by default). 
    * @note The weights W_theta^-1 are non-zero only within the 
    *       range [ q_min+0.5*guardRatio*D, q_max-0.5*guardRatio*D ]
    *       for each join, where D=q_max-q_min.
    * @return guard ratio.
    */
    double get_guardRatio() const { return guardRatio; }

    /**
    * Returns the parameter gamma which is used to blend the 
    * contribute of the task controller versus the contribute 
    * of the joint controller. 
    * @return gamma.
    */
    double get_gamma() const { return gamma; }

    /**
    * Returns the task execution time in seconds (1.0 by default). 
    * @return task execution time.
    */
    double get_execTime() const { return execTime; }

    /**
    * Returns the actual derivative of joint angles.
    * @return the actual derivative of joint angles. 
    */
    yarp::sig::Vector get_qdot() const { return qdot; }

    /**
    * Returns the actual derivative of End-Effector Pose (6 
    * components; xdot=J*qdot). 
    * @return the actual derivative of End-Effector Pose. 
    */
    yarp::sig::Vector get_xdot() const { return xdot; }

    /**
    * Sets the guard ratio (in [0 1]). 
    * @param _guardRatio. 
    */
    void set_guardRatio(double _guardRatio);

    /**
    * Sets the parameter gamma which is used to blend the contribute 
    * of the task controller versus the contribute of the joint
    * controller. 
    * @param _gamma. 
    */
    void set_gamma(double _gamma) { gamma=_gamma; }

    /**
    * Sets the joint angles values.
    * @param q0 is the joint angles vector. 
    */
    virtual void set_q(const yarp::sig::Vector &q0);

    /**
    * Sets the task execution time in seconds. 
    * @param _execTime. 
    * @param warn enable/disable warning message for thresholding 
    *             (disabled by default).
    * @return the actual execTime.
    * @note A lower bound equal to 10*Ts (Ts=controller's sample 
    *       time) is imposed.
    */
    double set_execTime(const double _execTime, const bool warn=false);

    /**
    * Adds to the controller input a further compensation term.
    * @param comp the compensation term. 
    * @note The compensation term holds for one iteration step, then 
    *       it is automatically set to zero for safety reasons.
    */
    void add_compensation(const yarp::sig::Vector &comp);

    /** 
    * Allows user to assign values to the parameters of plant under 
    * control (for the configuration space only). In case the 
    * controlled plant is not a pure integrator, then it can be 
    * modelled as the following transfer function: 
    * (Kp/s)*((1+Tz*s)/(1+2*Zeta*Tw*s+(Tw*s)^2)) 
    * @param parameters contains the set of plant parameters for 
    *                   each dimension in form of a Property object.
    *  
    * Available parameters are: 
    *  
    * \b dimension_# < list>: example (dimension_2 ((Kp 1.0) (Tw 
    *    0.1) ...)), specifies the Kp, Tz, Tw and Zeta parameters
    *    for a given dimension of the plant ("dimension_2" in the
    *    example). Dimensions are 0-based numbers.
    * @param entryTag specifies an entry tag different from 
    *                 "dimension". 
    */
    void setPlantParameters(const yarp::os::Property &parameters,
                            const std::string &entryTag="dimension");

    /**
    * Destructor.
    */
    virtual ~MultiRefMinJerkCtrl();
};

}

}

#endif


