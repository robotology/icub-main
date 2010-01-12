/**
 * \defgroup iKinSlv iKinSlv 
 *  
 * @ingroup iKin 
 *
 * Classes for on-line solution of inverse kinematic of iCub 
 * limbs based on IpOpt. The solvers run as on-line daemons 
 * which connect to the robot to retrieve information on the 
 * current joints configuration (along with the bounds) and by 
 * requesting a desired pose with queries made through YARP 
 * ports they return the corresponding target joints.
 *
 * The task to be solved is:
 *
 * \f[
 * \mathbf{q}=\arg\min_{\mathbf{q}\in R^{n} }\left(\frac{1}{2}\left\|\mathbf{\alpha}_d-\mathit{K_{\alpha}}\left(\mathbf{q}\right)\right\|^2+\mathit{w}\cdot\frac{1}{2}\left\|\mathbf{w}_{rest}.*\left(\mathbf{q}_{rest}-\mathbf{q}\right)\right\|^2\right) \quad s.t.\,\left\{\begin{array}{l}\left\|\mathbf{x}_d-\mathit{K_x}\left(\mathbf{q}\right)\right\|^2<\epsilon\\\mathbf{q}_L<\mathbf{q}<\mathbf{q}_U\end{array}\right.
 * \f]
 *
 * Where the solution \f$ \mathbf{q} \f$ is the joints vector with n components (depending
 * on the task and the current dof configuration) that is guaranteed to be found within the physical 
 * bounds expressed by \f$ \mathbf{q}_L \f$ and \f$ \mathbf{q}_U \f$; \f$ \mathbf{x}_d\equiv\left(x,y,d\right)_d \f$
 * is the positional part of the desired pose, \f$ \mathbf{\alpha}_d \f$ is the
 * desired orientation and \f$ \mathit{K_x} \f$ and \f$ \mathit{K_{\alpha}} \f$ are
 * the forward kinematic maps for the position and orientation part, respectively;
 * \f$ \mathbf{q}_{rest} \f$ is used to keep the solution as close as possible to a given rest
 * position in the joints space (weighting with a positive factor \f$ \mathit{w} < 1 \f$ and also
 * through \f$ \mathbf{w}_{rest} \f$ which allows to select a specific weight for each joint)
 * and \f$ \epsilon \f$ is a small number in the range of \f$ \left[10^{-5},10^{-4}\right] \f$.
 *  
 * \section protocol_sec Solver protocol
 *  
 * Once created the solver will open three ports with which user 
 * can communicate and exchange information according to the 
 * following rules: 
 *  
 * <b> /<solverName>/in </b> accepts a properties-like bottle 
 *    containing the following requests in streaming mode:
 *  
 * \b xd request: example ([xd] (x y z ax ay az theta)), 
 *    specifies the target pose to be achieved as a 7-components
 *    vector for position and orientation parts. Note that if
 *    the current pose is of [xyz] type, then only the first
 *    3-components are meaningful (x,...,az are in meters, theta
 *    is in radians).
 *  
 * \b pose request: example ([pose] [full]), specifies the 
 *    end-effector pose the user wants to achieve; it can be
 *    [full] (position+orientation) or [xyz] (only position).
 *  
 * \b mode request: example ([mode] [cont]), selects the solver 
 *    mode between [cont] which implements a continuous tracking
 *    and [shot] which does not compensate for movements induced
 *    on unacatuated joints. Indeed, in continuous mode the
 *    solver yields a new solution whenever a detected movements
 *    on uncontrolled links determines a displacement from the
 *    desired target.
 *  
 * \b dof request: example ([dof] (1 1 0 1 ...)), specifies 
 *    which dof of the chain are actuated (by putting 1 in the
 *    corresponding position) and which not (with 0). The length
 *    of the provided list of 1's and 0's should match the
 *    number of chain's dof. The special value 2 is used to keep
 *    the link status unchanged and proceed to next link.
 *  
 * \b resp request: example ([resp] (20.0 0.0 0.0 ...)), 
 *    specifies for each joint the rest position in degrees. The
 *    reply will contain the rest position resulting from the
 *    application of joints limits.
 *  
 * \b resw request: example ([resw] (1.0 0.0 1.0 ...)), 
 *    specifies for each joint the weight used to compute the
 *    rest position minimization task. The reply will contain
 *    the weights as result of a saturation over zero.
 *  
 * \b tok synchro: a double may be added to the request which 
 *    will be in turn sent back by the solver for
 *    synchronization purpose.
 *  
 * \note One single command sent to the streaming input port can 
 *       contain multiple requests: e.g. ([pose] [xyz]) ([dof]
 *       (1 0 1 2 1)) ([xd] (-0.1 0.1 0.1) ([tok] 1.234) ...)
 *  
 * \note Remind that the intended use of this port is in 
 *       streaming mode, thus it might happen that one request
 *       is dropped in favour of the most recent one. Therefore,
 *       as a general remark, rely on this port for xd requests
 *       and use rpc for dof, mode, pose, ... requests (see
 *       below).
 *
 *  
 * <b> /<solverName>/rpc </b> accepts a vocab-like bottle 
 *    containing the following requests, executes them and
 *    replies with [ack]/[nack] and/or some useful info:
 *  
 * Commands issued through [set]/[get] vocab: 
 *  
 * \b pose request: example [set] [pose] [full]/[xyz], [get] 
 *    [pose].
 *  
 * \b mode request: example [set] [mode] [cont]/[shot], [get] 
 *    [mode].
 *  
 * \b lim request: example [set] [lim] axis min max, [get] [lim]
 *    axis. Set/return minimum and maximum values for the joint.
 *    Allowed range shall be a valid subset of the real control
 *    limits (unit is deg).
 *  
 * \b verbosity request: example [set] [verb] [on]/[off], [get] 
 *    [verb].
 *  
 * \b dof request: example [set] [dof] (1 2 1 0 ...), [get] 
 *    [dof]. The reply will contain the current dof as result of
 *    the reconfiguration. The result may differ from the
 *    request since on certain limb (e.g. arm) some links are
 *    considered to form a unique super-link (e.g. the shoulder)
 *    whose components cannot be actuated separately.
 *  
 * \b resp request: example [set] [resp] (20.0 0.0 0.0 ...), 
 *    [get] [resp]. Set/get for each joint the rest position in
 *    degrees. The reply will contain the rest position
 *    resulting from the application of joints limits.
 *  
 * \b resw request: example [set] [resw] (1.0 0.0 1.0 ...), 
 *    [get] [resw]. Set/get for each joint the weight used to
 *    compute the rest position minimization task. The reply
 *    will contain the weights as result of a saturation over
 *    zero.
 *  
 * Commands concerning the thread status: 
 *  
 * \b susp request: example [susp], suspend the thread. 
 *  
 * \b run request: example [run], let the thread run again. 
 *  
 * \b quit request: example [quit], close ports and quit thread. 
 *    [ack] is returned.
 *  
 * Commands concerning the solver configuration: 
 *  
 * \b cfg request: example [cfg] (robot icub) (type left) (pose 
 *    full) (tol 1e-3) (maxIter 150) ... Once instantiated the
 *    solver is not automatically configured. To do that user
 *    can call the open() method locally or can configure the
 *    solver remotely by sending to the rpc port the
 *    configuration properties. For a list of these properties
 *    please see the documentation of open() method.
 *  
 *  
 * <b> /<solverName>/out </b> streams out a bottle containing 
 *    the result of optimization instance. The format is ([xd]
 *    (...)) ([x] (...)) ([q] (...) ([tok] ...)) as following:
 *  
 * \b xd property: contains the desired cartesian pose to be 
 *    achieved (7-components vector).
 *  
 * \b x property: contains the real cartesian pose achieved 
 *    which in norm is the nearest one to xd (7-components
 *    vector).
 *  
 * \b q property: contains the joints configuration which 
 *    achieve x (DOF-components vector in deg).
 *  
 * \b tok property: contains the token that the client may have 
 *    added to the request.
 *  
 * Date: first release 20/06/2009
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKINSLV_H__
#define __IKINSLV_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/iKinHlp.h>
#include <iCub/iKinIpOpt.h>

#include <string>
#include <deque>


namespace iKin
{

class CartesianSolver;

class RpcProcessor : public yarp::os::PortReader
{
protected:
    CartesianSolver *slv;

    virtual bool read(yarp::os::ConnectionReader &connection);

public:
    RpcProcessor(CartesianSolver *_slv) : slv(_slv) { }
};


class InputPort : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    CartesianSolver *slv;

    bool contMode;
    bool isNew;
    bool dofChanged;
    int  maxLen;
    int  pose;

    double  token;
    double *pToken;

    yarp::sig::Vector dof;
    yarp::sig::Vector xd;
    yarp::sig::Vector xdOld;

    virtual void onRead(yarp::os::Bottle &b);

public:
    InputPort(CartesianSolver *_slv);
    void reset_xd(const yarp::sig::Vector &_xd);
    yarp::sig::Vector &get_dof() { return dof;      }
    yarp::sig::Vector &get_xd()  { return xd;       }
    int    &get_pose()           { return pose;     }
    bool   &get_contMode()       { return contMode; }
    double *get_tokenPtr()       { return pToken;   }

    bool isNewDataEvent();
    bool handleTarget(yarp::os::Bottle *b);
    bool handleDOF(yarp::os::Bottle *b);
    bool handlePose(const int newPose);
    bool handleMode(const int newMode);    
};


class SolverCallback : public iKinIterateCallback
{
protected:
    CartesianSolver *slv;

public:
    SolverCallback(CartesianSolver *_slv) : slv(_slv) { }

    virtual void exec(yarp::sig::Vector xd, yarp::sig::Vector q);
};


struct PartDescriptor
{
    iKinLimb                      *lmb;
    iKinChain                     *chn;
    iKinLinIneqConstr             *cns;
    std::deque<yarp::os::Property> prp;
    std::deque<bool>               rvs;
    int                            num;
};


/**
* \ingroup iKinSlv
*
* Abstract class defining the core of on-line solvers.
*/
class CartesianSolver : public yarp::os::RateThread,
                        public CartesianHelper
{
protected:
    PartDescriptor                        *prt;
    std::deque<yarp::dev::PolyDriver*>     drv;
    std::deque<yarp::dev::IControlLimits*> lim;
    std::deque<yarp::dev::IEncoders*>      enc;
    std::deque<int>                        jnt;
    std::deque<int*>                       rmp;

    iKinIpOptMin   *slv;
    SolverCallback *clb;

    RpcProcessor                             *cmdProcessor;
    yarp::os::Port                           *rpcPort;
    InputPort                                *inPort;
    yarp::os::BufferedPort<yarp::os::Bottle> *outPort;
    yarp::os::Semaphore                       mutex;

    std::string   slvName;
    std::string   type;
    unsigned int  period;
    unsigned int  ctrlPose;
    bool          fullDOF;
    bool          configured;
    bool          closed;
    bool          verbosity;
    int           maxPartJoints;
    int           unctrlJointsNum;
    double        token;
    double       *pToken;

    yarp::sig::Vector unctrlJointsOld;
    yarp::sig::Vector dof;

    yarp::sig::Vector restJntPos;
    yarp::sig::Vector restWeights;

    double weightRestTask;
    yarp::sig::Vector qd_RestTask;
    yarp::sig::Vector w_RestTask;

    yarp::os::Bottle solutionBottle;

    void *dofEvent;

    virtual PartDescriptor *getPartDesc(yarp::os::Searchable &options)=0;
    virtual yarp::sig::Vector solve(yarp::sig::Vector &xd);

    virtual yarp::sig::Vector &encodeDOF();
    virtual bool decodeDOF(const yarp::sig::Vector &_dof);

    virtual bool handleJointsRestPosition(const yarp::os::Bottle *options,
                                          yarp::os::Bottle *reply=NULL);
    virtual bool handleJointsRestWeights(const yarp::os::Bottle *options,
                                         yarp::os::Bottle *reply=NULL);    
    
    bool   isNewDOF(const yarp::sig::Vector &_dof);
    bool   changeDOF(const yarp::sig::Vector &_dof);

    void   alignJointsBounds();
    bool   setLimits(int axis, double min, double max);
    void   countUncontrolledJoints();
    void   latchUncontrolledJoints(yarp::sig::Vector &joints);
    void   getFeedback(const bool wait=false);    
    void   initPos();
    void   lock();
    void   unlock();    

    void   waitDOFHandling();
    void   postDOFHandling();
    void   fillDOFInfo(yarp::os::Bottle &reply);
    double getNorm(const yarp::sig::Vector &v, const std::string &typ);    
    void   send(const yarp::sig::Vector &xd, double *tok);
    void   send(const yarp::sig::Vector &xd, const yarp::sig::Vector &x, const yarp::sig::Vector &q, double *tok);
    void   printInfo(const yarp::sig::Vector &xd, const yarp::sig::Vector &x, const yarp::sig::Vector &q,
                     const double t);    

    virtual void prepareJointsRestTask();
    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
    virtual bool threadInit();
    virtual void afterStart(bool);
    virtual void run();
    virtual void threadRelease();

    friend class RpcProcessor;
    friend class InputPort;
    friend class SolverCallback;

public:
    /**
    * Constructor. 
    * @param _slvName specifies the base name for all ports created 
    *                 as below:
    *  
    * /<_slvName>/in : the input port which accepts requests in 
    * streaming mode. 
    *  
    * /<_slvName>/rpc : the input port where to send requests and
    * wait for replies. 
    *  
    * /<_slvName>/out : the port which streams out the results of 
    * optimization. 
    */
    CartesianSolver(const std::string &_slvName);

    /**
    * Configure the solver and start it up. 
    * @param options contains the set of options in form of a 
    *                Property object.
    *  
    * Available options are: 
    *  
    * \b robot : example (robot icub), specifies the name of the 
    *    robot to connect to.
    *  
    * \b type : example (type right), specifies the type of the limb 
    *    to be instantiated; it can be "left" or "right".
    *  
    * \b dof : example (dof (1 1 0 1 ...)), specifies which dof of 
    *    the chain are actuated (by putting 1 in the corresponding
    *    position) and which not (with 0). The length of the
    *    provided list of 1's and 0's should match the number of
    *    chain's links. The special value 2 is used to keep the link
    *    status unchanged and proceed to next link.
    *  
    * \b rest_pos : example (rest_pos (20.0 0.0 0.0 ...)), specifies
    *    in degrees the joints rest position used as secondary task
    *    in the minimization. The lenght of the provided list should
    *    match the number of chain's links. Default values are (0.0
    *    0.0 0.0 ...).
    *  
    * \b rest_weights : example (rest_weights (1.0 0.0 0.0 1.0 
    *    ...)), specifies for each link the weights used for the
    *    secondary task minimization. The length of the provided
    *    list should match the number of the chain's links. Default
    *    values are (0.0 0.0 0.0 ...).
    *  
    * \b period : example (period 30), specifies the thread period 
    *    in ms.
    *  
    * \b pose : example (pose full), specifies the end-effector pose
    *    the user wants to achieve; it can be [full]
    *    (position+orientation) or [xyz] (only position).
    *  
    * \b mode : example (mode cont), selects the solver mode between 
    *    [cont] which implements a continuous tracking and [shot]
    *    which does not compensate for movements induced on
    *    unacatuated joints.
    *  
    * \b verbosity : example (verbosity on), selects whether to 
    *    report or not on the screen the result of each optimization
    *    instance; allowed values are [on] or [off].
    *  
    * \b maxIter : example (maxIter 200), specifies the maximum 
    *    number of iterations allowed for one optimization instance.
    *  
    * \b tol : example (tol 1e-3), specifies the desired tolerance 
    *    on the task function to be minimized.
    *  
    * \b xyzTol : example (xyzTol 1e-6), specifies the desired 
    *    tolerance on the positional part of the task function to be
    *    minimized whenever the "full" pose requirement is raised.
    *  
    * \b interPoints : example (interPoints on), selects whether to 
    *    force or not the solver to output on the port all
    *    intermediate points of optimization instance; allowed
    *    values are [on] or [off].
    *  
    * @return true/false if successful/failed
    */
    virtual bool open(yarp::os::Searchable &options);

    /**
    * Stop the solver and dispose it. Called by destructor.
    */
    virtual void close();

    /**
    * To be called to check whether the solver has received a [quit]
    * request. 
    * @return true/false if closed or not. 
    */
    virtual bool isClosed() { return closed; }

    /**
    * Default destructor.
    */
    virtual ~CartesianSolver();
};


/**
* \ingroup iKinSlv
*
* Derived class which implements the on-line solver for the 
* chain torso+arm. 
*/
class iCubArmCartesianSolver : public CartesianSolver
{
protected:
    virtual PartDescriptor *getPartDesc(yarp::os::Searchable &options);
    virtual bool decodeDOF(const yarp::sig::Vector &_dof);
    virtual yarp::sig::Vector solve(yarp::sig::Vector &xd);

public:
    /**
    * Constructor. 
    * @param _slvName specifies the base name for all ports created. 
    *                 By default is "armCartSolver". See the
    *                 father's class constructor for the description
    *                 of the ports
    */
    iCubArmCartesianSolver(const std::string &_slvName="armCartSolver") : CartesianSolver(_slvName) { }

    virtual bool open(yarp::os::Searchable &options);
};


/**
* \ingroup iKinSlv
*
* Derived class which implements the on-line solver for the leg 
* chain. 
*/
class iCubLegCartesianSolver : public CartesianSolver
{
protected:
    virtual PartDescriptor *getPartDesc(yarp::os::Searchable &options);

public:
    /**
    * Constructor. 
    * @param _slvName specifies the base name for all ports created. 
    *                 By default is "legCartSolver". See the
    *                 father's class constructor for the description
    *                 of the ports
    */
    iCubLegCartesianSolver(const std::string &_slvName="legCartSolver") : CartesianSolver(_slvName) { }
};

}

#endif



