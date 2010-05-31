/**
 * \defgroup iKinIpOpt iKinIpOpt 
 *  
 * @ingroup iKin 
 *
 * Classes for inverse kinematics of serial-links chains and 
 * iCub limbs based on IpOpt. To install IpOpt see the <a
 * href="http://eris.liralab.it/wiki/Installing_IPOPT">wiki</a>.
 *
 * Date: first release 23/06/2008
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKINIPOPT_H__
#define __IKINIPOPT_H__

#include <iCub/iKin/iKinInv.h>
#include <IpTNLP.hpp>
#include <IpIpoptApplication.hpp>


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
    virtual void exec(yarp::sig::Vector xd, yarp::sig::Vector q) = 0;
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

    Ipopt::Number lowerBoundInf;
    Ipopt::Number upperBoundInf;

    bool active;

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
    iKinLinIneqConstr(Ipopt::Number _lowerBoundInf, Ipopt::Number _upperBoundInf);

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
    * @note default is -1e9.
    * @return -inf.
    */
    Ipopt::Number &getLowerBoundInf() { return lowerBoundInf; }

    /**
    * Returns a reference to the internal representation of +inf. 
    * @note default is +1e9. 
    * @return +inf.
    */
    Ipopt::Number &getUpperBoundInf() { return upperBoundInf; }

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
    * @note Useful when it is required to handle change in 
    *       inherited objects.
    */
    virtual void update(void*) { }
};


/**
* \ingroup iKinIpOpt
*
* Class for defining IpOpt NLP problem
*/
class iKin_NLP : public Ipopt::TNLP
{
private:
    // Copy constructor: not implemented.
    iKin_NLP(const iKin_NLP&);
    // Assignment operator: not implemented.
    iKin_NLP &operator=(const iKin_NLP&);

protected:
    iKinChain &chain;
    iKinChain &chain2ndTask;

    iKinLinIneqConstr &LIC;

    unsigned int dim;
    unsigned int dim_2nd;
    unsigned int ctrlPose;

    yarp::sig::Vector &xd;
    yarp::sig::Vector &xd_2nd;
    yarp::sig::Vector &w_2nd;
    yarp::sig::Vector &qd_3rd;
    yarp::sig::Vector &w_3rd;
    yarp::sig::Vector  qd;
    yarp::sig::Vector  q0;
    yarp::sig::Vector  q;
    bool   *exhalt;

    yarp::sig::Vector  e_xyz;
    yarp::sig::Vector  e_ang;
    yarp::sig::Vector  e_2nd;
    yarp::sig::Vector  e_3rd;
    yarp::sig::Matrix  J_xyz;
    yarp::sig::Matrix  J_ang;
    yarp::sig::Matrix  J_2nd;

    yarp::sig::Vector *e_1st;
    yarp::sig::Matrix *J_1st;

    yarp::sig::Vector linC;

    Ipopt::Number __obj_scaling;
    Ipopt::Number __x_scaling;
    Ipopt::Number __g_scaling;

    Ipopt::Number lowerBoundInf;
    Ipopt::Number upperBoundInf;

    Ipopt::Number translationalTol;

    iKinIterateCallback *callback;

    double weight2ndTask;
    double weight3rdTask;
    bool firstGo;

    virtual void computeQuantities(const Ipopt::Number *x);

public:
    /** default constructor */
    iKin_NLP(iKinChain &c, unsigned int _ctrlPose, const yarp::sig::Vector &_q0, yarp::sig::Vector &_xd,
             double _weight2ndTask, iKinChain &_chain2ndTask, yarp::sig::Vector &_xd_2nd,
             yarp::sig::Vector &_w_2nd, double _weight3rdTask, yarp::sig::Vector &_qd_3rd,
             yarp::sig::Vector &_w_3rd, iKinLinIneqConstr &_LIC, bool *_exhalt=NULL);

    /** returns the solution */
    yarp::sig::Vector get_qd() { return qd; }

    /** sets callback */
    void set_callback(iKinIterateCallback *_callback) { callback=_callback; }

    /** sets scaling factors */
    void set_scaling(Ipopt::Number _obj_scaling, Ipopt::Number _x_scaling, Ipopt::Number _g_scaling)
    {
        __obj_scaling=_obj_scaling;
        __x_scaling  =_x_scaling;
        __g_scaling  =_g_scaling;
    }

    /** sets scaling factors */
    void set_bound_inf(Ipopt::Number lower, Ipopt::Number upper)
    {
        lowerBoundInf=lower;
        upperBoundInf=upper;
    }

    /** sets translational tolerance */
    void set_translational_tol(Ipopt::Number tol) { translationalTol=tol; }

    /** default destructor */
    virtual ~iKin_NLP() { }

    /** Method to return some info about the nlp */
    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);
    
    /** Method to return the bounds for my problem */
    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);
    
    /** Method to return the starting point for the algorithm */
    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda,
                                    Ipopt::Number* lambda);
    
    /** Method to return the objective value */
    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number& obj_value);
    
    /** Method to return the gradient of the objective */
    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Number* grad_f);
    
    /** Method to return the constraint residuals */
    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x, Ipopt::Index m, Ipopt::Number* g);
    
    /** Method to return:
    *   1) The structure of the jacobian (if "values" is NULL)
    *   2) The values of the jacobian (if "values" is not NULL)
    */
    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,Ipopt::Index m,
                            Ipopt::Index nele_jac, Ipopt::Index* iRow, Ipopt::Index *jCol,
                            Ipopt::Number* values);
    
    /** Method to return:
    *   1) The structure of the hessian of the lagrangian (if "values" is NULL)
    *   2) The values of the hessian of the lagrangian (if "values" is not NULL)
    */
    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values);

    /** overload this method to return scaling parameters. This is
    *  only called if the options are set to retrieve user scaling.
    *  There, use_x_scaling (or use_g_scaling) should get set to true
    *  only if the variables (or constraints) are to be scaled.  This
    *  method should return true only if the scaling parameters could
    *  be provided.
    */
    virtual bool get_scaling_parameters(Ipopt::Number& obj_scaling,
                                        bool& use_x_scaling, Ipopt::Index n, Ipopt::Number* x_scaling,
                                        bool& use_g_scaling, Ipopt::Index m, Ipopt::Number* g_scaling);

    /** This method is called once per iteration, after the iteration
     *  summary output has been printed.
     */
    virtual bool intermediate_callback(Ipopt::AlgorithmMode mode, Ipopt::Index iter, Ipopt::Number obj_value,
                                       Ipopt::Number inf_pr, Ipopt::Number inf_du, Ipopt::Number mu, Ipopt::Number d_norm,
                                       Ipopt::Number regularization_size, Ipopt::Number alpha_du, Ipopt::Number alpha_pr,
                                       Ipopt::Index ls_trials, const Ipopt::IpoptData* ip_data,
                                       Ipopt::IpoptCalculatedQuantities* ip_cq);
    
    /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n, const Ipopt::Number* x,
                                   const Ipopt::Number* z_L, const Ipopt::Number* z_U, Ipopt::Index m,
                                   const Ipopt::Number* g, const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data, Ipopt::IpoptCalculatedQuantities* ip_cq);
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

    // cannot be accessed from outside
    Ipopt::IpoptApplication *App;

protected:
    iKinChain &chain;
    iKinChain chain2ndTask;

    iKinLinIneqConstr  noLIC;
    iKinLinIneqConstr *pLIC;

    unsigned int ctrlPose;    

    Ipopt::Number obj_scaling;
    Ipopt::Number x_scaling;
    Ipopt::Number g_scaling;

    Ipopt::Number lowerBoundInf;
    Ipopt::Number upperBoundInf;

    Ipopt::Number translationalTol;

    /**
    * Provides access to IpoptApplication's OptimizeTNLP method.
    */
    Ipopt::ApplicationReturnStatus optimize(const Ipopt::SmartPtr<Ipopt::TNLP>& tnlp);

    /**
    * Provides access to IpoptApplication's ReOptimizeTNLP method.
    */
    Ipopt::ApplicationReturnStatus reoptimize(const Ipopt::SmartPtr<Ipopt::TNLP>& tnlp);

public:
    /**
    * Constructor. 
    * @param c is the Chain object on which the control operates. Do 
    *          not change Chain DOF from this point onwards!!
    * @param _ctrlPose one of the following: 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled.
    *  IKINCTRL_POSE_ANG  => rotational part of pose controlled. 
    * @param tol exits if 0.5*norm(xd-x)^2<tol.
    * @param max_iter exits if iter>=max_iter (max_iter<0 disables
    *                 this check, IKINCTRL_DISABLED(==-1) by
    *                 default).
    * @param verbose is a integer number which progressively enables 
    *                different levels of warning messages or status
    *                dump. The larger this value the more detailed
    *                is the output (0=>off by default).
    * @param useHessian relies on exact Hessian computation or  
    *                enable Quasi-Newton approximation (true by
    *                default).
    */
    iKinIpOptMin(iKinChain &c, unsigned int _ctrlPose,
                 const double tol, const int max_iter=IKINCTRL_DISABLED,
                 const unsigned int verbose=0, bool useHessian=true);

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
    unsigned int get_ctrlPose() { return ctrlPose; }

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
    * Selects 2nd Task End-Effector by giving the ordinal number n 
    * of last joint pointing at it.
    * @param n is the ordinal number of last joint pointing at the 
    *          2nd End-Effector.
    * @return a reference to the secondary chain. 
    */
    iKinChain &specify2ndTaskEndEff(unsigned int n);

    /**
    * Sets Tolerance.
    * @param tol exits if norm(xd-x)<tol.
    */
    void setTol(const Ipopt::Number tol);

    /**
    * Sets Maximum Iteration.
    * @param max_iter exits if iter>=max_iter (max_iter<0 
    *                 (IKINCTRL_DISABLED) disables this check).
    */ 
    void setMaxIter(const Ipopt::Index max_iter);

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
    void setUserScaling(const bool useUserScaling, Ipopt::Number _obj_scaling,
                        Ipopt::Number _x_scaling, Ipopt::Number _g_scaling);

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
    * Returns the lower and upper bounds to represent -inf and +inf
    * @param lower is a reference to return the lower bound 
    * @param upper is a reference to return the upper bound 
    */
    void getBoundsInf(Ipopt::Number &lower, Ipopt::Number &upper);

    /**
    * Sets the lower and upper bounds to represent -inf and +inf
    * @param lower is the new lower bound 
    * @param upper is the new upper bound 
    */
    void setBoundsInf(Ipopt::Number lower, Ipopt::Number upper);

    /**
    * Returns the tolerance used by the algorithm for translational 
    * part when the complete pose is to be achieved. 
    * @note tolerance is applied to the squared norm 
    * @return the current translational tolerance (1e-6 by default).
    */
    Ipopt::Number getTranslationalTol() { return translationalTol; }

    /**
    * Sets the tolerance used by the algorithm for translational 
    * part when the complete pose is to be achieved (1e-6 by 
    * default). 
    * @note tolerance is applied to the squared norm 
    * @param tol is the new translational tolerance
    */
    void setTranslationalTol(const Ipopt::Number tol) { translationalTol=tol; }

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
                                    Ipopt::ApplicationReturnStatus *exit_code=NULL, bool *exhalt=NULL,
                                    iKinIterateCallback *iterate=NULL);

    /**
    * Default destructor.
    */
    virtual ~iKinIpOptMin();
};

}

#endif


