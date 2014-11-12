/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/**
 * \defgroup iKinSlv iKinSlv 
 *  
 * @ingroup iKin 
 *
 * Classes for on-line solution of inverse kinematic of iCub 
 * limbs based on <a 
 * href="http://wiki.icub.org/wiki/Installing_IPOPT">IpOpt</a>.
 * The solvers run as on-line daemons which connect to the robot 
 * to retrieve information on the current joints configuration 
 * (along with the bounds) and by requesting a desired pose with 
 * queries made through YARP ports they return the corresponding 
 * target joints. 
 *
 * The task to be solved is:
 *
 * \f[
 * \mathbf{q}=\arg\min_{\mathbf{q}\in R^{n} }\left(\left\|\mathbf{\alpha}_d-\mathit{K_{\alpha}}\left(\mathbf{q}\right)\right\|^2+\mathit{w}\cdot\left(\mathbf{q}_r-\mathbf{q}\right)^{\top}\mathit{W}_r\left(\mathbf{q}_r-\mathbf{q}\right)\right) \quad s.t.\,\left\{\begin{array}{l}\left\|\mathbf{x}_d-\mathit{K_x}\left(\mathbf{q}\right)\right\|^2<\epsilon\\\mathbf{q}_L<\mathbf{q}<\mathbf{q}_U\end{array}\right.
 * \f]
 *
 * Where the solution \f$ \mathbf{q} \f$ is the joints vector with n components (depending
 * on the task and the current dof configuration) that is guaranteed to be found within the physical 
 * bounds expressed by \f$ \mathbf{q}_L \f$ and \f$ \mathbf{q}_U \f$; \f$ \mathbf{x}_d\equiv\left(x,y,d\right)_d \f$
 * is the positional part of the desired pose, \f$ \mathbf{\alpha}_d \f$ is the
 * desired orientation and \f$ \mathit{K_x} \f$ and \f$ \mathit{K_{\alpha}} \f$ are
 * the forward kinematic maps for the position and orientation part, respectively;
 * \f$ \mathbf{q}_r \f$ is used to keep the solution as close as possible to a given rest
 * position in the joint space (weighting with a positive factor \f$ \mathit{w} < 1 \f$ and also
 * through the diagonal matrix \f$ \mathit{W}_r \f$ which allows to select a specific weight for each joint)
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
 *    the link status unchanged and proceed to the next link.
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
 * \b pose priority request: example [set] [prio] [xyz]/[ang], 
 *    [get] [prio]. For example, setting priority to [ang]
 *    allows considering the reaching in orientation as a
 *    constraint while reaching in position is handled as an
 *    objective.
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
 *    resulting from the application of joints limits (lim
 *    request does affect the range).
 *  
 * \b resw request: example [set] [resw] (1.0 0.0 1.0 ...), 
 *    [get] [resw]. Set/get for each joint the weight used to
 *    compute the rest position minimization task. The reply
 *    will contain the weights as result of a saturation over
 *    zero.
 *  
 * \b tsk2 request: example [set] [tsk2] (6 (0.0 0.0 -1.0) (0.0 
 *    0.0 1.0), [get] [tsk2]. Set/get the options for the
 *    secondary task, where the first integer accounts for the
 *    joints to control, then come the desired x-y-z coordinates
 *    of the secondary end-effector and finally the
 *    corresponding three weights.
 *  
 * \b conv request: example [set] [conv] ((max_iter 200) (tol 
 *    0.001) (translationalTol 0.000001)), [get] [conv]. Set/get
 *    the options for specifying solver's convergence.
 *  
 * Commands issued through the [ask] vocab: 
 *  
 * \b xd request: example [ask] ([xd] (x y z ax ay az theta))
 *    ([pose] [xyz]) ([q] (...)). Ask to solve for the target
 *    xd. User can specifies a different starting joint
 *    configuration q and the pose mode. The reply will contain
 *    something like [ack] ([q] (...)) ([x] (...)), where the
 *    found configuration q is returned as well as the final
 *    attained pose x.
 *  
 * Commands concerning the thread status: 
 *  
 * \b susp request: example [susp], suspend the thread. 
 *  
 * \b run request: example [run], let the thread run again.  
 *  
 * \b status request: example [status], returns [ack] 
 *    "not_started"|"running"|"suspended".
 *  
 * \b quit request: example [quit], close ports and quit the
 *    thread. [ack] is returned.
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
 *    achieves x (DOF-components vector in degrees).
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

#include <string>
#include <deque>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Event.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/iKin/iKinHlp.h>
#include <iCub/iKin/iKinIpOpt.h>


namespace iCub
{

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

    yarp::os::Mutex mutex;

    bool contMode;
    bool isNew;
    int  maxLen;
    int  pose;

    double  token;
    double *pToken;

    yarp::sig::Vector dof;
    yarp::sig::Vector xd;

    virtual void onRead(yarp::os::Bottle &b);

public:
    InputPort(CartesianSolver *_slv);

    int               &get_pose()     { return pose;     }
    bool              &get_contMode() { return contMode; }
    double            *get_tokenPtr() { return pToken;   }
    yarp::sig::Vector  get_dof();
    yarp::sig::Vector  get_xd();
    
    void set_dof(const yarp::sig::Vector &_dof);
    void reset_xd(const yarp::sig::Vector &_xd);
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

    virtual void exec(const yarp::sig::Vector &xd, const yarp::sig::Vector &q);
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
class CartesianSolver : public    yarp::os::RateThread,
                        protected CartesianHelper
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
    yarp::os::Mutex                           mutex;

    std::string   slvName;
    std::string   type;
    unsigned int  period;
    unsigned int  ctrlPose;
    bool          fullDOF;
    bool          contModeOld;
    bool          configured;
    bool          closing;
    bool          closed;
    bool          interrupting;
    bool          verbosity;
    bool          timeout_detected;
    int           maxPartJoints;
    int           unctrlJointsNum;
    double        ping_robot_tmo;
    double        token;
    double       *pToken;

    yarp::sig::Vector unctrlJointsOld;
    yarp::sig::Vector dof;

    yarp::sig::Vector restJntPos;
    yarp::sig::Vector restWeights;

    yarp::sig::Vector xd_2ndTask;
    yarp::sig::Vector w_2ndTask;

    yarp::sig::Vector qd_3rdTask;
    yarp::sig::Vector w_3rdTask;
    yarp::sig::Vector idx_3rdTask;

    yarp::os::Event dofEvent;

    virtual PartDescriptor *getPartDesc(yarp::os::Searchable &options)=0;
    virtual yarp::sig::Vector solve(yarp::sig::Vector &xd);

    virtual yarp::sig::Vector &encodeDOF();
    virtual bool decodeDOF(const yarp::sig::Vector &_dof);

    virtual bool handleJointsRestPosition(const yarp::os::Bottle *options,
                                          yarp::os::Bottle *reply=NULL);
    virtual bool handleJointsRestWeights(const yarp::os::Bottle *options,
                                         yarp::os::Bottle *reply=NULL);

    yarp::dev::PolyDriver *waitPart(const yarp::os::Property &partOpt);
    
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
    void   send(const yarp::sig::Vector &xd, const yarp::sig::Vector &x, const yarp::sig::Vector &q, double *tok);
    void   printInfo(const std::string &typ, const yarp::sig::Vector &xd, const yarp::sig::Vector &x,
                     const yarp::sig::Vector &q, const double t);    

    virtual void prepareJointsRestTask();
    virtual void respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
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
    * \b robot <string>: example (robot icub), specifies the name of
    *    the robot to connect to.
    *  
    * \b type <string>: example (type right), specifies the type of 
    *    the limb to be instantiated; it can be "left" or "right".
    *  
    * \b dof <(int int ...)>: example (dof (1 1 0 1 ...)), specifies
    *    which dof of the chain are actuated (by putting 1 in the
    *    corresponding position) and which not (with 0). The length
    *    of the provided list of 1's and 0's should match the number
    *    of chain's links. The special value 2 is used to keep the
    *    link status unchanged and proceed to the next link.
    *  
    * \b rest_pos <(double double ...)>: example (rest_pos (20.0 0.0
    *    0.0 ...)), specifies in degrees the joints rest position
    *    used as secondary task in the minimization. The length of
    *    the provided list should match the number of chain's links.
    *    Default values are (0.0 0.0 0.0 ...).
    *  
    * \b rest_weights <(double double ...)>: example (rest_weights 
    *    (1.0 0.0 0.0 1.0 ...)), specifies for each link the weights
    *    used for the secondary task minimization. The length of the
    *    provided list should match the number of the chain's links.
    *    Default values are (0.0 0.0 0.0 ...).
    *  
    * \b period <int>: example (period 30), specifies the thread 
    *    period in ms.
    *  
    * \b pose <vocab>: example (pose full), specifies the 
    *    end-effector pose the user wants to achieve; it can be
    *    [full] (position+orientation) or [xyz] (only position).
    *  
    * \b mode : example (mode cont), selects the solver mode between 
    *    [cont] which implements a continuous tracking and [shot]
    *    which does not compensate for movements induced on
    *    unacatuated joints.
    *  
    * \b verbosity <vocab>: example (verbosity on), selects whether 
    *    to report or not on the screen the result of each
    *    optimization instance; allowed values are [on] or [off].
    *  
    * \b maxIter <int>: example (maxIter 200), specifies the maximum
    *    number of iterations allowed for one optimization instance.
    *  
    * \b tol <double>: example (tol 1e-3), specifies the desired 
    *    tolerance on the task function to be minimized.
    *  
    * \b xyzTol <double>: example (xyzTol 1e-6), specifies the 
    *    desired tolerance on the positional part of the task
    *    function to be minimized.
    *  
    * \b interPoints <vocab>: example (interPoints on), selects 
    *    whether to force or not the solver to output on the port
    *    all intermediate points of optimization instance; allowed
    *    values are [on] or [off].
    *  
    * \b ping_robot_tmo <double>: example (ping_robot_tmo 2.0), 
    *    specifies a timeout in seconds during which robot state
    *    ports are pinged prior to connecting; a timeout equal to
    *    zero disables this option.
    *  
    * @return true/false if successful/failed
    */
    virtual bool open(yarp::os::Searchable &options);

    /**
    * Interrupt the open() method waiting for motor parts to be 
    * ready. 
    */
    virtual void interrupt();

    /**
    * Stop the solver and dispose it. Called by destructor.
    */
    virtual void close();

    /**
    * To be called to check whether the solver has received a [quit]
    * request. 
    * @return true/false if closed or not. 
    */
    virtual bool isClosed() const { return closed; }

    /**
    * To be called to check whether communication timeout has been 
    * detected. 
    * @return reference to the internal flag. 
    *  
    * @note At first timeout detection the flag is set and is never 
    *       reset again.
    */
    virtual bool &getTimeoutFlag() { return timeout_detected; }

    /**
    * Suspend the solver's main loop.
    */
    virtual void suspend();

    /**
    * Resume the solver's main loop.
    */
    virtual void resume();

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

}

#endif



