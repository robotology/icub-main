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
 * \defgroup iKinIpOpt iKinIpOpt 
 *  
 * @ingroup iKin 
 *
 * Classes for inverse kinematics of serial-links chains and 
 * iCub limbs based on IpOpt. To install IpOpt see the <a
 * href="http://wiki.icub.org/wiki/Installing_IPOPT">wiki</a>.
 *
 * Date: first release 23/06/2008
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKINIPOPT_H__
#define __IKINIPOPT_H__

#include <iCub/iKin/iKinInv.h>


namespace iCub
{

namespace iKin
{

/**
* \ingroup iKinIpOpt
*
* Class for defining iteration callback
*/
class iKinIterateCallback
{
private:
    // Copy constructor: not implemented.
    iKinIterateCallback(const iKinIterateCallback&);
    // Assignment operator: not implemented.
    iKinIterateCallback &operator=(const iKinIterateCallback&);

public:
    iKinIterateCallback() { }

    /**
    * Defines the callback body to be called at each iteration.
    * @param xd current target.
    * @param q current estimation of joint angles. 
    */ 
    virtual void exec(const yarp::sig::Vector &xd, const yarp::sig::Vector &q) = 0;
};


/**
* \ingroup iKinIpOpt
*
* Class for defining Linear Inequality Constraints of the form 
* lB <= C*q <= uB for the nonlinear problem NLP. 
*/
class iKinLinIneqConstr
{
protected:
    yarp::sig::Matrix C;
    yarp::sig::Vector uB;
    yarp::sig::Vector lB;

    double lowerBoundInf;
    double upperBoundInf;
    bool   active;

    virtual void clone(const iKinLinIneqConstr *obj);

public:
    /**
    * Default Constructor. 
    */
    iKinLinIneqConstr();

    /**
    * Constructor. 
    * Defines linear inequality constraints of the form 
    * lB <= C*q <= uB. 
    * @param _lowerBoundInf specifies -inf when there is no lower
    *                       bound.
    * @param _upperBoundInf specifies +inf when there is no upper
    *                       bound.
    */
    iKinLinIneqConstr(const double _lowerBoundInf, const double _upperBoundInf);

    /**
    * Creates a new LinIneqConstr object from an already existing 
    * LinIneqConstr object. 
    * @param obj is the LinIneqConstr to be copied.
    */
    iKinLinIneqConstr(const iKinLinIneqConstr &obj);

    /**
    * Copies a LinIneqConstr object into the current one.
    * @param obj is a reference to an object of type 
    *            iKinLinIneqConstr.
    * @return a reference to the current object.
    */
    virtual iKinLinIneqConstr &operator=(const iKinLinIneqConstr &obj);

    /**
    * Returns a reference to the constraints matrix C.
    * @return constraints matrix C. 
    */
    yarp::sig::Matrix &getC() { return C; }

    /**
    * Returns a reference to the upper bounds vector uB.
    * @return upper bounds vector uB.
    */
    yarp::sig::Vector &getuB() { return uB; }

    /**
    * Returns a reference to the lower bounds vector lB.
    * @return lower bounds vector lB.
    */
    yarp::sig::Vector &getlB() { return lB; }

    /**
    * Returns a reference to the internal representation of -inf. 
    * @return -inf. 
    */
    double &getLowerBoundInf() { return lowerBoundInf; }

    /**
    * Returns a reference to the internal representation of +inf. 
    * @return +inf. 
    */
    double &getUpperBoundInf() { return upperBoundInf; }

    /**
    * Returns the state of inequality constraints evaluation.
    * @return true if inequality constraints are active.
    */
    bool isActive() { return active; }

    /**
    * Sets the state of inequality constraints evaluation.
    * @param if the new state is true the evaluation is performed.
    */
    void setActive(bool _active) { active=_active; }

    /**
    * Updates internal state. 
    *  
    * @note Useful when it is required to handle change in 
    *       inherited objects.
    */
    virtual void update(void*) { }
};


/**
* \ingroup iKinIpOpt
*
* Class for dealing with iCub shoulder's constraints due to the 
* cables length.
*/
class iCubShoulderConstr : public iKinLinIneqConstr
{
protected:    
    double     shou_m, shou_n;
    double     elb_m,  elb_n;
    iKinChain *chain;

    void clone(const iKinLinIneqConstr *obj);

public:
    /**
    * Constructor. 
    * @param arm the iCubArm object.
    */
    iCubShoulderConstr(iCubArm &arm);

    void update(void*);
};


/**
* \ingroup iKinIpOpt
*
* Class for inverting chain's kinematics based on IpOpt lib
*/
class iKinIpOptMin
{
private:
    // Default constructor: not implemented.
    iKinIpOptMin();
    // Copy constructor: not implemented.
    iKinIpOptMin(const iKinIpOptMin&);
    // Assignment operator: not implemented.
    iKinIpOptMin &operator=(const iKinIpOptMin&);    

protected:
    void *App;

    iKinChain &chain;
    iKinChain chain2ndTask;

    iKinLinIneqConstr  noLIC;
    iKinLinIneqConstr *pLIC;

    unsigned int ctrlPose;    

    double obj_scaling;
    double x_scaling;
    double g_scaling;
    double lowerBoundInf;
    double upperBoundInf;
    std::string posePriority;

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled. 
    * @param tol        cost function tolerance. 
    * @param constr_tol constraints tolerance.
    * @param max_iter   exits if iter>=max_iter (max_iter<0 disables
    *                   this check, IKINCTRL_DISABLED(==-1) by
    *                   default).
    * @param verbose    integer which progressively enables different
    *                   levels of warning messages or status dump. The
    *                   larger this value the more detailed is the
    *                   output (0=>off by default).
    * @param useHessian relies on exact Hessian computation or  
    *                   enable Quasi-Newton approximation (true by
    *                   default).
    */
    iKinIpOptMin(iKinChain &c, const unsigned int _ctrlPose,
                 const double tol, const double constr_tol,
                 const int max_iter=IKINCTRL_DISABLED,
                 const unsigned int verbose=0, bool useHessian=true);

    /**
    * Sets the state of Pose control settings.
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled.
    */
    void set_ctrlPose(const unsigned int _ctrlPose);

    /**
    * Returns the state of Pose control settings.
    * @return Pose control settings.
    */
    unsigned int get_ctrlPose() const { return ctrlPose; }

    /**
    * Sets the Pose priority for weighting more either position or 
    * orientation while reaching in full pose. 
    * @param priority can be "position" or "orientation". 
    * @return true/false on success/failure. 
    */
    bool set_posePriority(const std::string &priority);

    /**
    * Returns the Pose priority settings.
    * @return pose priority.
    */
    std::string get_posePriority() const { return posePriority; }

    /**
    * Attach a iKinLinIneqConstr object in order to impose 
    * constraints of the form lB <= C*q <= uB.
    * @param lic is the iKinLinIneqConstr object to attach.
    * @see iKinLinIneqConstr
    */
    void attachLIC(iKinLinIneqConstr &lic) { pLIC=&lic; }

    /**
    * Returns a reference to the attached Linear Inequality 
    * Constraints object.
    * @return Linear Inequality Constraints pLIC. 
    * @see iKinLinIneqConstr
    */
    iKinLinIneqConstr &getLIC() { return *pLIC; }

    /**
    * Selects the End-Effector of the 2nd task by giving the ordinal
    * number n of last joint pointing at it. 
    * @param n is the ordinal number of last joint pointing at the 
    *          2nd End-Effector.
    */
    void specify2ndTaskEndEff(const unsigned int n);

    /**
    * Retrieves the 2nd task's chain. 
    * @return a reference to the 2nd task's chain.  
    */
    iKinChain &get2ndTaskChain();

    /**
    * Sets Maximum Iteration.
    * @param max_iter exits if iter>=max_iter (max_iter<0 
    *                 (IKINCTRL_DISABLED) disables this check).
    */ 
    void setMaxIter(const int max_iter);

    /**
    * Retrieves the current value of Maximum Iteration.
    * @return max_iter. 
    */ 
    int getMaxIter() const;

    /**
    * Sets Maximum CPU seconds.
    * @param max_cpu_time exits if cpu_time>=max_cpu_time given in 
    *                     seconds.
    */ 
    void setMaxCpuTime(const double max_cpu_time);

    /**
    * Retrieves the current value of Maximum CPU seconds.
    * @return max_cpu_time.
    */ 
    double getMaxCpuTime() const;

    /**
    * Sets cost function tolerance.
    * @param tol tolerance.
    */
    void setTol(const double tol);

    /**
    * Retrieves cost function tolerance.
    * @return tolerance.
    */
    double getTol() const;

    /**
    * Sets constraints tolerance.
    * @param tol tolerance.
    */
    void setConstrTol(const double constr_tol);

    /**
    * Retrieves constraints tolerance.
    * @return tolerance.
    */
    double getConstrTol() const;

    /**
    * Sets Verbosity.
    * @param verbose is a integer number which progressively enables 
    *                different levels of warning messages or status
    *                dump. The larger this value the more detailed
    *                is the output.
    */
    void setVerbosity(const unsigned int verbose);

    /**
    * Selects whether to rely on exact Hessian computation or enable
    * Quasi-Newton approximation (Hessian is enabled at start-up by 
    * default). 
    * @param useHessian true if Hessian computation is enabled.
    */
    void setHessianOpt(const bool useHessian);

    /**
    * Enables/disables user scaling factors.
    * @param useUserScaling true if user scaling is enabled. 
    * @param obj_scaling user scaling factor for the objective 
    *                    function.
    * @param x_scaling user scaling factor for variables. 
    * @param g_scaling user scaling factor for constraints. 
    */
    void setUserScaling(const bool useUserScaling, const double _obj_scaling,
                        const double _x_scaling, const double _g_scaling);

    /**
    * Enable\disable derivative test at each call to solve method 
    * (disabled at start-up by default). Useful to check the 
    * derivatives implementation of NLP. 
    * @param enableTest true if derivative test shall be enabled. 
    * @param enable2ndDer true to enable second derivative test as 
    *                     well (false by default).
    */
    void setDerivativeTest(const bool enableTest, const bool enable2ndDer=false);

    /**
    * Returns the lower and upper bounds to represent -inf and +inf.
    * @param lower is a reference to return the lower bound.
    * @param upper is a reference to return the upper bound. 
    */
    void getBoundsInf(double &lower, double &upper);

    /**
    * Sets the lower and upper bounds to represent -inf and +inf.
    * @param lower is the new lower bound. 
    * @param upper is the new upper bound. 
    */
    void setBoundsInf(const double lower, const double upper);

    /**
    * Executes the IpOpt algorithm trying to converge on target. 
    * @param q0 is the vector of initial joint angles values. 
    * @param xd is the End-Effector target Pose to be attained. 
    * @param weight2ndTask weights the second task (disabled if 
    *                      0.0).
    * @param xd_2nd is the second target task traslational Pose to 
    *             be attained (typically a particular elbow xyz
    *             position).
    * @param w_2nd weights each components of the distance vector 
    *              xd_2nd-x_2nd. Hence, the follows holds as second
    *              task: min 1/2*norm2(((xd_i-x_i)*w_i)_i)
    * @param weight3rdTask weights the third task (disabled if 0.0).
    * @param qd_3rd is the third task joint angles target 
    *             positions to be attained.
    * @param w_3rd weights each components of the distance vector 
    *              qd-q. Hence, the follows holds as third task:
    *             min 1/2*norm2(((qd_i-q_i)*w_i)_i)
    * @param exit_code stores the exit code (NULL by default). Test 
    *                  for one of this:
    *                   SUCCESS
    *                   MAXITER_EXCEEDED
    *                   STOP_AT_TINY_STEP
    *                   STOP_AT_ACCEPTABLE_POINT
    *                   LOCAL_INFEASIBILITY
    *                   USER_REQUESTED_STOP
    *                   FEASIBLE_POINT_FOUND
    *                   DIVERGING_ITERATES
    *                   RESTORATION_FAILURE
    *                   ERROR_IN_STEP_COMPUTATION
    *                   INVALID_NUMBER_DETECTED
    *                   TOO_FEW_DEGREES_OF_FREEDOM
    *                   INTERNAL_ERROR
    * @param exhalt checks for an external request to exit (NULL by 
    *               default).
    * @param iterate pointer to a callback object (NULL by default). 
    * @return estimated joint angles.
    */
    virtual yarp::sig::Vector solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd,
                                    double weight2ndTask, yarp::sig::Vector &xd_2nd, yarp::sig::Vector &w_2nd,
                                    double weight3rdTask, yarp::sig::Vector &qd_3rd, yarp::sig::Vector &w_3rd,
                                    int *exit_code=NULL, bool *exhalt=NULL, iKinIterateCallback *iterate=NULL);

    /**
    * Executes the IpOpt algorithm trying to converge on target. 
    * @param q0 is the vector of initial joint angles values. 
    * @param xd is the End-Effector target Pose to be attained. 
    * @param weight2ndTask weights the second task (disabled if 
    *                      0.0).
    * @param xd_2nd is the second target task traslational Pose to 
    *             be attained (typically a particular elbow xyz
    *             position).
    * @param w_2nd weights each components of the distance vector 
    *              xd_2nd-x_2nd. Hence, the follows holds as second
    *              task: min 1/2*norm2(((xd_i-x_i)*w_i)_i)
    * @return estimated joint angles.
    */
    virtual yarp::sig::Vector solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd,
                                    double weight2ndTask, yarp::sig::Vector &xd_2nd, yarp::sig::Vector &w_2nd);

    /**
    * Executes the IpOpt algorithm trying to converge on target. 
    * @param q0 is the vector of initial joint angles values. 
    * @param xd is the End-Effector target Pose to be attained. 
    * @return estimated joint angles.
    */
    virtual yarp::sig::Vector solve(const yarp::sig::Vector &q0, yarp::sig::Vector &xd);

    /**
    * Default destructor.
    */
    virtual ~iKinIpOptMin();
};

}

}

#endif


