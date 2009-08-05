/**
 * \defgroup iKinSlv iKinSlv 
 *  
 * @ingroup iKin 
 *
 * Classes for on-line solution of inverse kinematic of iCub 
 * limbs based on IpOpt. The solvers run as an on-line daemons 
 * which connect to the robot to retrieve information on the 
 * current joints configuration (along with the bounds) and by 
 * requesting a desired pose with queries made through YARP 
 * ports they return the corresponding target joints. 
 *  
 *  
 * \section protocol_sec Solver protocol
 *  
 * Once created the solver will open three ports with which user 
 * can communicate and exchange information according to the 
 * following rules: 
 *  
 * \b "/<solverName>/in" accepts a properties-like bottle 
 *    containing the following requests in streaming mode:
 *  
 * \b xd request: example ([xd] (x y z ax ay az theta)), 
 *    specifies the target pose to be achieved as a 7-components
 *    vector for position and orientation parts. Note that if
 *    the current pose is of [xyz] type, then only the first
 *    3-components are meaningful.
 *  
 * \b pose request: example ([pose] [full]), specifies the 
 *    end-effector pose the user wants to achieve; it can be
 *    [full] (position+orientation) or [xyz] (only position).
 *  
 * \b mode request: example ([mode] [cont]), selects the solver 
 *    mode between [cont] which implements a continuous tracking
 *    and [shot] which does not compensate for movements induced
 *    on unacatuated joints.
 *  
 * \b dof request: example ([dof] (1 1 0 1 ...)), specifies 
 *    which dof of the chain are actuated (by putting 1 in the
 *    corresponding position) and which not (with 0). The length
 *    of the provided list of 1's and 0's should match the
 *    number of chain's dof. The special value 2 is used to keep
 *    the link status unchanged and proceed to next link.
 *  
 * \note One single command sent to the streaming input port can 
 *       contain multiple requests: e.g. ([pose] [xyz]) ([dof]
 *       (1 0 1 2 1)) ([xd] (-0.1 0.1 0.1))
 *
 *  
 * \b "/<solverName>/rpc" accepts a vocab-like bottle containing
 *    the following requests, executes them and replies with
 *    [ack]/[nack] and/or some usuful info:
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
 *    limits.
 *  
 * \b verbosity request: example [set] [verb] [on]/[off], [get] 
 *    [verb].
 *  
 * \b dof request: example [set] [dof] (1 2 1 0 ...), [get] 
 *    [dof]. The reply will contain in both case the current dof
 *    as result of the reconfiguration. The result may differ
 *    from the request since on certain limb (e.g. arm) some
 *    links are considered to form a unique super-link (e.g. the
 *    shoulder) whose components cannot be actuated or not
 *    separately.
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
 * \b "/<solverName>/out" stream out a bottle containing the 
 *    result of optimization instance. The format is ([xd]
 *    (...)) ([x] (...)) ([q] (...)) as following:
 *  
 * \b xd property: contains the desired cartesian pose to be 
 *    achieved (7-components vector).
 *  
 * \b x property: contains the real cartesian pose achieved 
 *    which in norm is the nearest one to xd (7-components
 *    vector).
 *  
 * \b q property: contains the joints configuration which 
 *    achieve x (DOF-components vector).
 *  
 * Date: first release 20/06/2009
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKINISLV_H__
#define __IKINISLV_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iCub/iKinIpOpt.h>

#include <string>
#include <deque>

#define IKINSLV_VOCAB_CMD_HELP          VOCAB4('h','e','l','p')
#define IKINSLV_VOCAB_CMD_SUSP          VOCAB4('s','u','s','p')
#define IKINSLV_VOCAB_CMD_RUN           VOCAB3('r','u','n')
#define IKINSLV_VOCAB_CMD_CFG           VOCAB3('c','f','g')
#define IKINSLV_VOCAB_CMD_GET           VOCAB3('g','e','t')
#define IKINSLV_VOCAB_CMD_SET           VOCAB3('s','e','t')
#define IKINSLV_VOCAB_CMD_QUIT          VOCAB4('q','u','i','t')
#define IKINSLV_VOCAB_OPT_MODE          VOCAB4('m','o','d','e')
#define IKINSLV_VOCAB_OPT_POSE          VOCAB4('p','o','s','e')
#define IKINSLV_VOCAB_OPT_DOF           VOCAB3('d','o','f')
#define IKINSLV_VOCAB_OPT_LIM           VOCAB3('l','i','m')
#define IKINSLV_VOCAB_OPT_XD            VOCAB2('x','d')
#define IKINSLV_VOCAB_OPT_X             VOCAB1('x')
#define IKINSLV_VOCAB_OPT_Q             VOCAB1('q')
#define IKINSLV_VOCAB_OPT_VERB          VOCAB4('v','e','r','b')
#define IKINSLV_VOCAB_VAL_POSE_FULL     VOCAB4('f','u','l','l')
#define IKINSLV_VOCAB_VAL_POSE_XYZ      VOCAB3('x','y','z')
#define IKINSLV_VOCAB_VAL_MODE_TRACK    VOCAB4('c','o','n','t')
#define IKINSLV_VOCAB_VAL_MODE_SINGLE   VOCAB4('s','h','o','t')
#define IKINSLV_VOCAB_VAL_ON            VOCAB2('o','n')
#define IKINSLV_VOCAB_VAL_OFF           VOCAB3('o','f','f')
#define IKINSLV_VOCAB_REP_ACK           VOCAB3('a','c','k')
#define IKINSLV_VOCAB_REP_NACK          VOCAB4('n','a','c','k')


namespace iKin
{
    class  RpcProcessor;
    class  InputPort;
    class  SolverCallback;
    struct PartDescriptor;
    class  CartesianSolver;
    class  ArmCartesianSolver;
    class  LegCartesianSolver;
}


using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;


class iKin::RpcProcessor : public yarp::os::PortReader
{
protected:
    CartesianSolver *slv;

    virtual bool read(yarp::os::ConnectionReader &connection);

public:
    RpcProcessor(CartesianSolver *_slv) : slv(_slv) { }
};


class iKin::InputPort : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    CartesianSolver *slv;

    bool isNew;
    bool contMode;
    int  maxLen;
    int  pose;

    yarp::sig::Vector dof;
    yarp::sig::Vector xd;
    yarp::sig::Vector xdOld;

    virtual void onRead(yarp::os::Bottle &b);

public:
    InputPort(CartesianSolver *_slv);
    void    reset_xd(const yarp::sig::Vector &_xd);
    yarp::sig::Vector &get_dof() { return dof;      }
    yarp::sig::Vector &get_xd()  { return xd;       }
    int  &get_pose()             { return pose;     }
    bool &get_contMode()         { return contMode; }

    bool isNewDataEvent();
    bool handleTarget(yarp::os::Bottle *b);
    bool handleDOF(yarp::os::Bottle *b);
    bool handlePose(const int newPose);
    bool handleMode(const int newMode);    
};


class iKin::SolverCallback : public iKin::iKinIterateCallback
{
protected:
    CartesianSolver *slv;

public:
    SolverCallback(CartesianSolver *_slv) : slv(_slv) { }

    virtual void exec(yarp::sig::Vector xd, yarp::sig::Vector q);
};


struct iKin::PartDescriptor
{
    iKinLimb             *lmb;
    iKinChain            *chn;
    iKinLinIneqConstr    *cns;
    std::deque<Property>  prp;
    std::deque<bool>      rvs;
    int                   num;
};


/**
* \ingroup iKinSlv
*
* Abstract class defining the core of on-line solvers.
*/
class iKin::CartesianSolver : public yarp::os::RateThread
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

    std::string  slvName;
    std::string  type;
    unsigned int period;
    unsigned int ctrlPose;
    bool         fullDOF;
    bool         configured;
    bool         closed;
    bool         verbosity;
    int          maxPartJoints;
    int          unctrlJointsNum;

    yarp::sig::Vector unctrlJointsOld;
    yarp::sig::Vector dof;
    yarp::sig::Vector fb;

    yarp::os::Bottle solutionBottle;

    void *dofEvent;

    virtual PartDescriptor *getPartDesc(yarp::os::Searchable &options)=0;
    virtual bool decodeDOF(const yarp::sig::Vector &_dof);
    virtual yarp::sig::Vector solve(yarp::sig::Vector &xd);

    yarp::sig::Vector &encodeDOF();
    bool   isNewDOF(const yarp::sig::Vector &_dof);
    bool   changeDOF(const yarp::sig::Vector &_dof);

    void   alignJointsBounds();
    bool   setLimits(int axis, double min, double max);
    void   countUncontrolledJoints();
    void   latchUncontrolledJoints(yarp::sig::Vector &joints);
    void   getFeedback(yarp::sig::Vector &_fb);
    void   initPos();
    void   lock();
    void   unlock();    

    void   waitDOFHandling();
    void   postDOFHandling();
    void   fillDOFInfo(yarp::os::Bottle &reply);
    double getNorm(const yarp::sig::Vector &v, const string &typ);
    bool   respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);
    void   send(const yarp::sig::Vector &xd);
    void   send(const yarp::sig::Vector &xd, const yarp::sig::Vector &x, const yarp::sig::Vector &q);
    void   printInfo(yarp::sig::Vector &xd, yarp::sig::Vector &x, yarp::sig::Vector &q, const double t);

    static void addVectorOption(yarp::os::Bottle &b, const int vcb, const yarp::sig::Vector &v);

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
    *    chain's dof. The special value 2 is used to keep the link
    *    status unchanged and proceed to next link.
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
    * Appends to a bottle all data needed to command a target.
    * @param b is the bottle where to append the data.
    * @param xd is the target [7-components vector].
    */
    static void addTargetOption(yarp::os::Bottle &b, const yarp::sig::Vector &xd);

    /**
    * Appends to a bottle all data needed to reconfigure chain's 
    * dof. 
    * @param b is the bottle where to append the data.
    * @param dof is the vector of new chain's dof configuration. 
    */
    static void addDOFOption(yarp::os::Bottle &b, const yarp::sig::Vector &dof);

    /**
    * Appends to a bottle all data needed to change the pose mode.
    * @param b is the bottle where to append the data.
    * @param pose is the new pose mode. 
    *  IKINCTRL_POSE_FULL => complete pose control.
    *  IKINCTRL_POSE_XYZ  => translational part of pose controlled. 
    */
    static void addPoseOption(yarp::os::Bottle &b, const unsigned int pose);

    /**
    * Appends to a bottle all data needed to change the tracking 
    * mode. 
    * @param b is the bottle where to append the data.
    * @param tracking true to enable tracking mode. 
    */
    static void addModeOption(yarp::os::Bottle &b, const bool tracking);

    /**
    * Retrieves commanded target data from a bottle.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static Bottle *getTargetOption(yarp::os::Bottle &b);

    /**
    * Retrieves the end-effector pose data.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static Bottle *getEndEffectorPoseOption(yarp::os::Bottle &b);

    /**
    * Retrieves the joints configuration data.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static Bottle *getJointsOption(yarp::os::Bottle &b);

    /**
    * Stop the solver and dispose it. Called by destructor.
    */
    virtual void close();

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
class iKin::ArmCartesianSolver : public iKin::CartesianSolver
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
    ArmCartesianSolver(const std::string &_slvName="armCartSolver") : CartesianSolver(_slvName) { }

    virtual bool open(yarp::os::Searchable &options);
};


/**
* \ingroup iKinSlv
*
* Derived class which implements the on-line solver for the leg 
* chain. 
*/
class iKin::LegCartesianSolver : public iKin::CartesianSolver
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
    LegCartesianSolver(const std::string &_slvName="legCartSolver") : CartesianSolver(_slvName) { }
};


#endif



