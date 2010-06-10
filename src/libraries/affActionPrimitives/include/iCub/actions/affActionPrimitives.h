/**
 * \defgroup affActionPrimitives affActionPrimitives
 *  
 * @ingroup icub_libraries 
 *  
 * Definition of primitive actions for dealing with affordances 
 * and more. 
 *
 * \author Ugo Pattacini
 *
 * \section intro_sec Description
 *
 * The library relies on the Yarp Cartesian Interface and 
 * provides the user a collection of action primitives in task 
 * space and joint space along with an easy way to combine them
 * together forming higher level actions (e.g. grasp(), tap(), 
 * …) in order to eventually execute more sophisticated tasks 
 * without reference to the motion control details. 
 *  
 * \image html affActionPrimitives.jpg 
 *  
 * Central to the library's implementation is the concept of 
 * @b action. An action is a "request" for an execution of three
 * different tasks according to its internal selector: 
 *  
 * -# It can ask to steer the arm to a specified pose, hence 
 * performing a motion in the operational space.
 * -# It can command the execution of some predefined 
 * fingers sequences in the joint space identified by a tag. 
 * -# It can ask the system to wait for a specified time 
 * interval.
 *  
 * Besides, there exists the possibility of generating one 
 * action for the execution of a task of type 1 simultaneously 
 * to a task of type 2. 
 *  
 * Moreover, whenever an action is produced from within the code 
 * the corresponding request item is pushed at the bottom of 
 * <b>actions queue</b>. Therefore, user can identify suitable 
 * fingers movements in the joint space, associate proper 
 * grasping 3d points together with hand posture and finally 
 * execute the grasping task as a harmonious combination of a 
 * reaching movement and fingers actuations, complying with the 
 * time requirements of the synchronized sequence. 
 *  
 * It is also given the option to execute a callback whenever an
 * action is accomplished. 
 *  
 * To detect contacts among fingers and objects the \ref 
 * icub_graspDetector module is employed.
 */ 

#ifndef __AFFACTIONPRIMITIVES_H__
#define __AFFACTIONPRIMITIVES_H__

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>

#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynInv.h>
#include <iCub/iDyn/iDynTransform.h>

#include <string>
#include <deque>
#include <set>
#include <map>

#define ACTIONPRIM_DISABLE_EXECTIME    -1


namespace actions
{

/**
* \ingroup affActionPrimitives
*
* Class for defining routines to be called when action is 
* completed. 
*/
class affActionPrimitivesCallback
{
public:
    affActionPrimitivesCallback() { }

    /**
    * Defines the callback body to be called at the action end.
    */ 
    virtual void exec() = 0;
};


/**
* \ingroup affActionPrimitives
*
* The base class defining actions. 
*  
* It allows to execute arm (in task-space, e.g. reach()) and 
* hand (in joint-space) primitive actions and to combine them in 
* the actions queue. 
*/
class affActionPrimitives : public yarp::os::RateThread
{
protected:
    std::string robot;
    std::string local;
    std::string part;

    yarp::dev::PolyDriver        *polyHand;
    yarp::dev::PolyDriver        *polyCart;
    yarp::dev::IEncoders         *encCtrl;
    yarp::dev::IPositionControl  *posCtrl;
    yarp::dev::ICartesianControl *cartCtrl;

    yarp::os::BufferedPort<yarp::os::Bottle> graspDetectionPort;

    yarp::os::RateThread *armWaver;
    yarp::os::Semaphore  *mutex;
    yarp::os::Event       motionStartEvent;
    yarp::os::Event       motionDoneEvent;

    bool armMoveDone;
    bool handMoveDone;
    bool latchArmMoveDone;
    bool latchHandMoveDone;
    bool handSeqTerminator;
    bool fingersInPosition;

    bool configured;
    bool closed;
    bool checkEnabled;
    bool tracking_mode;
    bool torsoActive;
    bool verbose;

    double default_exec_time;
    double waitTmo;
    double latchTimer;
    double t0;

    int jHandMin;
    int jHandMax;

    yarp::sig::Vector      enableTorsoSw;
    yarp::sig::Vector      disableTorsoSw;

    yarp::sig::Vector      curHandFinalPoss;
    yarp::sig::Vector      curHandTols;
    yarp::sig::Vector      curGraspDetectionThres;
    std::set<int>          fingersJntsSet;
    std::set<int>          fingersMovingJntsSet;
    std::multimap<int,int> fingers2JntsMap;

    struct HandWayPoint
    {
        std::string       tag;
        yarp::sig::Vector poss;
        yarp::sig::Vector vels;
        yarp::sig::Vector tols;
        yarp::sig::Vector thres;
    };

    struct Action
    {
        // wait action
        bool waitState;
        double tmo;
        // reach action
        bool execArm;
        yarp::sig::Vector x;
        yarp::sig::Vector o;
        double execTime;
        bool oEnabled;
        // hand action
        bool execHand;
        HandWayPoint handWP;
        bool handSeqTerminator;
        // action callback
        affActionPrimitivesCallback *clb;
    };

    affActionPrimitivesCallback *actionClb;

    std::deque<Action> actionsQueue;
    std::map<std::string,std::deque<HandWayPoint> > handSeqMap;

    virtual std::string toCompactString(const yarp::sig::Vector &v);
    virtual int  printMessage(const char *format, ...);
    virtual bool handleTorsoDOF(yarp::os::Property &opt, const std::string &key,
                                const int j);
    virtual bool configHandSeq(yarp::os::Property &opt);
    virtual bool _pushAction(const bool execArm, const yarp::sig::Vector &x,
                             const yarp::sig::Vector &o, const double execTime,
                             const bool oEnabled, const bool execHand, const HandWayPoint &handWP,
                             const bool handSeqTerminator, affActionPrimitivesCallback *clb);
   virtual bool  _pushAction(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                             const std::string &handSeqKey, const double execTime,
                             affActionPrimitivesCallback *clb, const bool oEnabled);
    virtual bool stopJntTraj(const int jnt);
    virtual bool handCheckMotionDone(const int jnt);
    virtual void enableTorsoDof();
    virtual void disableTorsoDof();
    virtual bool wait(const Action &action);
    virtual bool cmdArm(const Action &action);
    virtual bool cmdHand(const Action &action);
    virtual bool isHandSeqEnded();
    virtual void postReachCallback();

    virtual void init();
    virtual bool execQueuedAction();
    virtual bool execPendingHandSequences();
    virtual void run();

public:
    /**
    * Default Constructor. 
    */
    affActionPrimitives();

    /**
    * Constructor. 
    * @param opt the Property used to configure the object after its
    *            creation.
    */
    affActionPrimitives(yarp::os::Property &opt);

    /**
    * Destructor. 
    *  
    * @note it calls the close() method. 
    */
    virtual ~affActionPrimitives();

    /**
    * Configure the object.
    * @param opt the Property used to configure the object after its
    *            creation.
    *  
    * @note To be called after object creation. 
    *  
    * @note Available options are: 
    *  
    * @b local <string>: specify a stem name used to open local 
    *    ports and to highlight messages printed on the screen. 
    *  
    * @b robot <string>: the robot name to connect to (e.g. icub). 
    *  
    * @b part <string>:  the arm to be controlled (e.g. left_arm). 
    *  
    * @b thread_period <int>: the thread period [ms] which selects 
    *    the time granularity as well.
    *  
    * @b default_exec_time <double>: the arm movement execution time
    *    [s].
    *  
    * @b reach_tol <double>: the reaching tolerance [m]. 
    *  
    * @b jntmotiondone_tol <double>: the tolerance [deg] for 
    *    detecting the end of fingers motion in joint space.
    *  
    * @b tracking_mode <string>: enable/disable the tracking mode; 
    *    possible values: "on"/"off".
    * @note In tracking mode the cartesian position is mantained on 
    *       the reached target.
    *  
    * @b verbosity <string>: enable/disable the verbose mode; 
    *    possible values: "on"/"off".
    *
    * @b torso_pitch <string>: if "on" it enables the control of the
    *    pitch of the torso.
    *  
    * @b torso_pitch_min <double>: set the pitch minimum value 
    *    [deg].
    *  
    * @b torso_pitch_max <double>: set the pitch maximum value 
    *    [deg].
    *  
    * @b torso_roll <string>: if "on" it enables the control of the 
    *    roll of the torso.
    *  
    * @b torso_roll_min <double>: set the roll minimum value [deg]. 
    *  
    * @b torso_roll_max <double>: set the roll maximum value [deg].
    *  
    * @b torso_yaw <string>: if "on" it enables the control of the 
    *    yaw of the torso.
    *  
    * @b torso_yaw_min <double>: set the yaw minimum value [deg]. 
    *  
    * @b torso_yaw_max <double>: set the yaw maximum value [deg]. 
    *  
    * @b hand_sequences_file <string>: complete path to the file 
    *    containing the hand motions sequences.<br />Here is the
    *    format of motion sequences:
    *  
    *  @code
    *  [GENERAL]
    *  numSequences ***
    *  
    *  [SEQ_0]
    *  key ***
    *  numWayPoints ***
    *  wp_0  (poss (10 ...)) (vels (20 ...)) (tols (30 ...)) (thres
    *  (1 2 3 4 5)) wp_1  *** ...
    *  
    *  [SEQ_1]
    *  ...
    *  
    *  // the "poss", "vels" and "tols" keys specify 9 joints
    *  // positions, velocities and tolerances whereas the "thres"
	*  // key specifies 5 fingers thresholds used for model-based
	*  // contact detection. The "tols" key serves to detect the end
	*  // motion condition
    *  @endcode
    *  
    * @note A port called <i> /<local>/<part>/detectGrasp:i </i> is 
    *       open to acquire data provided by \ref icub_graspDetector
    *       module.
    */
    virtual bool open(yarp::os::Property &opt);

    /**
    * Check if the object is initialized correctly. 
    * @return true/fail on success/fail. 
    */
    virtual bool isValid();

    /**
    * Deallocate the object.
    */
    virtual void close();

    /**
    * Insert a combination of arm and hand primitive actions in the 
    * actions queue. 
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation: ax ay az angle in rad).
    * @param handSeqKey the hand sequence key. 
    * @param execTime the arm action execution time [s] (to be 
    *          specified iff different from default value).
    * @param clb action callback that is executed when the action 
    *            ends; none by default.
    * @return true/false on success/fail. 
    *  
    * @note Some examples: 
    *  
    * the call @b pushAction(x,o,"close_hand") pushes the combined 
    * action of reachPose(x,o) and hand "close_hand" sequence into 
    * the queue; the action will be executed as soon as all the 
    * previous items in the queue will have been served. 
    */
    virtual bool pushAction(const yarp::sig::Vector &x, const yarp::sig::Vector &o, 
                            const std::string &handSeqKey,
                            const double execTime=ACTIONPRIM_DISABLE_EXECTIME,
                            affActionPrimitivesCallback *clb=NULL);

    /**
    * Insert a combination of arm and hand primitive actions in the 
    * actions queue. 
    * @param x the 3-d target position [m]. 
    * @param handSeqKey the hand sequence key. 
    * @param execTime the arm action execution time [s] (to be 
    *          specified iff different from default value).
    * @param clb action callback that is executed when the action 
    *            ends; none by default.
    * @return true/false on success/fail. 
    *  
    * @note Some examples: 
    *  
    * the call @b pushAction(x,"close_hand") pushes the combined 
    * action of reachPosition(x) and hand "close_hand" sequence into
    * the queue; the action will be executed as soon as all the 
    * previous items in the queue will have been served. 
    */
    virtual bool pushAction(const yarp::sig::Vector &x, 
                            const std::string &handSeqKey,
                            const double execTime=ACTIONPRIM_DISABLE_EXECTIME,
                            affActionPrimitivesCallback *clb=NULL);

    /**
    * Insert the arm-primitive action reach for target in the 
    * actions queue. 
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation: ax ay az angle in rad).
    * @param execTime the arm action execution time [s] (to be 
    *          specified iff different from default value).
    * @param clb action callback that is executed when the action 
    *            ends; none by default.
    * @return true/false on success/fail. 
    */
    virtual bool pushAction(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                            const double execTime=ACTIONPRIM_DISABLE_EXECTIME,
                            affActionPrimitivesCallback *clb=NULL);

    /**
    * Insert the arm-primitive action reach for target in the 
    * actions queue. 
    * @param x the 3-d target position [m]. 
    * @param execTime the arm action execution time [s] (to be 
    *          specified iff different from default value).
    * @param clb action callback that is executed when the action 
    *            ends; none by default.
    * @return true/false on success/fail. 
    */
    virtual bool pushAction(const yarp::sig::Vector &x,
                            const double execTime=ACTIONPRIM_DISABLE_EXECTIME,
                            affActionPrimitivesCallback *clb=NULL);

    /**
    * Insert a hand-primitive action in the actions queue.
    * @param handSeqKey the hand sequence key. 
    * @param clb action callback that is executed when the action 
    *            ends; none by default. 
    * @return true/false on success/fail. 
    */
    virtual bool pushAction(const std::string &handSeqKey,
                            affActionPrimitivesCallback *clb=NULL);

    /**
    * Insert a wait state in the actions queue.
    * @param tmo is the wait timeout [s]. 
    * @param clb callback that is executed when the timeout expires;
    *            none by default.
    * @return true/false on success/fail. 
    */
    virtual bool pushWaitState(const double tmo,
                               affActionPrimitivesCallback *clb=NULL);

    /**
    * Immediately update the current reaching target (without 
    * affecting the actions queue) or initiate a new reach (if the 
    * actions queue is empty).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation: ax ay az angle in rad).
    * @param execTime the arm action execution time [s] (to be 
    *          specified iff different from default value).
    * @return true/false on success/fail. 
    *  
    * @note The intended use is for tracking moving targets. 
    */
    virtual bool reachPose(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                           const double execTime=ACTIONPRIM_DISABLE_EXECTIME);

    /**
    * Immediately update the current reaching target (without 
    * affecting the actions queue) or initiate a new reach (if the 
    * actions queue is empty).
    * @param x the 3-d target position [m]. 
    * @param execTime the arm action execution time [s] (to be 
    *          specified iff different from default value).
    * @return true/false on success/fail. 
    *  
    * @note The intended use is for tracking moving targets. 
    */
    virtual bool reachPosition(const yarp::sig::Vector &x,
                               const double execTime=ACTIONPRIM_DISABLE_EXECTIME);

    /**
    * Empty the actions queue. 
    * @return true/false on success/fail.
    */
    virtual bool clearActionsQueue();

    /**
    * Define an hand WayPoint (WP) to be added at the bottom of the 
    * hand motion sequence pointed by the key. 
    * @param handSeqKey the hand sequence key.
    * @param poss the 9 fingers joints WP positions to be attained 
    *             [deg].
    * @param vels the 9 fingers joints velocities [deg/s]. 
    * @param tols the 9 fingers joints tolerances [deg] used to 
    *             detect end motion condition (motion is considered
    *             finished if |des(i)-fb(i)|<tols(i)).
    * @param thres the 5 fingers thresholds used for grasp 
    *              detection.
    * @return true/false on success/fail. 
    *  
    * @note this method creates a new empty sequence referred by the
    *       passed key if the key does not point to any valid
    *       sequence; hence the triplet (poss,vels,thres) will be
    *       the first WP of the new sequence.
    */
    virtual bool addHandSeqWP(const std::string &handSeqKey, const yarp::sig::Vector &poss,
                              const yarp::sig::Vector &vels, const yarp::sig::Vector &tols,
                              const yarp::sig::Vector &thres);

    /**
    * Check whether a sequence key is defined.
    * @param handSeqKey the hand sequence key.
    * @return true iff valid key. 
    */
    virtual bool isValidHandSeq(const std::string &handSeqKey);

    /**
    * Remove an already existing hand motion sequence.
    * @param handSeqKey the hand sequence key.
    * @return true/false on success/fail. 
    */
    virtual bool removeHandSeq(const std::string &handSeqKey);

    /**
    * Return the whole list of available hand sequence keys.
    * @return the list of available hand sequence keys.
    */
    std::deque<std::string> getHandSeqList();

    /**
    * Query if fingers are moving. 
    * @param f the result of the check.
    * @return true/false on success/fail. 
    */
    virtual bool areFingersMoving(bool &f);

    /**
    * Query if fingers are in position. 
    * @param f the result of the check. 
    * @return true/false on success/fail. 
    *  
    * @note Fingers are intended to be in position if they have 
    *       attained the desired position or while moving they
    *       follow the desired trajectory. Any possible contact
    *       among fingers or with objects causes the method to
    *       return false.
    */
    virtual bool areFingersInPosition(bool &f);

    /**
    * Return the cartesian interface used internally to control the 
    * limb. 
    * @param ctrl the cartesian interface.
    * @return true/false on success/fail. 
    *  
    * @note Useful to access solver's options through the <a 
    *       href="http://eris.liralab.it/yarpdoc/dd/de6/classyarp_1_1dev_1_1ICartesianControl.html">Cartesian
    *       Interface</a>.
    */
    virtual bool getCartesianIF(yarp::dev::ICartesianControl *&ctrl);

    /**
    * Stop any ongoing arm/hand movements.
    * @return true/false on success/fail. 
    *  
    * @note it empty out the actions queue. 
    */
    virtual bool stopControl();

    /**
    * Set the task space controller in tracking or non-tracking 
    * mode. 
    * @param f: true for tracking mode, false otherwise. 
    * @note In tracking mode the cartesian position is mantained on 
    *       the reached target.
    * @return true/false on success/failure.
    */
    virtual bool setTrackingMode(const bool f);

    /**
    * Get the current controller mode.
    * @return true/false on tracking/non-tracking mode. 
    */
    virtual bool getTrackingMode();

    /**
    * Enable the waving mode that keeps on moving the arm around a 
    * predefined position. 
    * @param restPos: the 3-d position around which to wave. 
    * @note useful to give a kind of more human perception when the 
    *       arm is homed. This mode is automatically disabled each
    *       time a new action is commanded.
    * @return true/false on success/failure.
    */
    virtual bool enableArmWaving(const yarp::sig::Vector &restPos);

    /**
    * Disable the waving mode.
    * @return true/false on success/failure.
    */
    virtual bool disableArmWaving();

    /**
    * Check whether all the actions in queue are accomplished.
    * @param f the result of the check. 
    * @param sync if true wait until all actions are accomplished 
    *             (blocking call).
    * @return true/false on success/fail. 
    *  
    * @note As specified the check is performed on the content of 
    *       the actions queue so that the blocking call returns as
    *       soon as the queue is empty.
    */
    virtual bool checkActionsDone(bool &f, const bool sync=false);

    /**
    * Check whether an action is still ongoing.
    * @param f the result of the check. 
    * @param sync if true wait that at least one action has just 
    *             started (blocking call).
    * @return true/false on success/fail. 
    *  
    * @note Sometimes it might be helpful to wait in the calling 
    *       module until the beginning of the action inserted in the
    *       queue. For example:
    * @code 
    * pushAction(A); 
    * checkActionOnGoing(f,true); 
    * // perform some computations that require the motion has been 
    * // started off 
    * checkActionsDone(f,true); 
    * @endcode 
    */
    virtual bool checkActionOnGoing(bool &f, const bool sync=false);

    /**
    * Suddenly interrupt any blocking call that is pending on 
    * querying the action status. 
    * @param disable disable the blocking feature for future 
    * calls with sync switch on; useful to allow a graceful stop of 
    * the application. @see syncCheckReinstate to reinstate it.
    * @return true/false on success/fail. 
    */
    virtual bool syncCheckInterrupt(const bool disable=false);

    /**
    * Reinstate the blocking feature for future calls with sync 
    * switch on. 
    * @return true/false on success/fail. 
    */
    virtual bool syncCheckReinstate();    
};


/**
* \ingroup affActionPrimitives
*
* A derived class defining a first abstraction layer on top of 
* @ref affActionPrimitives father class. 
*  
* It internally predeclares (without actually defining) a set of
* hand sequence motions key ("open_hand", "close_hand" and 
* "karate_hand" :) that are used for grasp(), touch() and tap() 
* actions. 
*  
* @note Given as an example of how primitive actions can be 
*       combined in higher level actions, thus how further
*       layers can be inherited from the base class.
*/
class affActionPrimitivesLayer1 : public affActionPrimitives
{
public:
    /**
    * Default Constructor. 
    */
    affActionPrimitivesLayer1() : affActionPrimitives() { }

    /**
    * Constructor. 
    * @param opt the Property used to configure the object after its
    *            creation.
    */
    affActionPrimitivesLayer1(yarp::os::Property &opt) : affActionPrimitives(opt) { }

    /**
    * Grasp the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @return true/false on success/fail. 
    *  
    * @note internal implementation (pseudo-code): 
    * @code 
    * ... 
    * pushAction(x+d,o,"open_hand"); 
    * pushAction(x,o); 
    * pushAction("close_hand"); 
    * ... 
    * @endcode 
    *  
    * It reachs for (x+d,o) opening the hand, then reachs for (x,o)
    * and finally closes the hand. 
    */
    virtual bool grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * Touch the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/touching
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          touching.
    * @return true/false on success/fail. 
    *  
    * @note internal implementation (pseudo-code): 
    * @code 
    * ... 
    * pushAction(x+d,o,"karate_hand"); 
    * pushAction(x,o); 
    * ... 
    * @endcode 
    *  
    * It reachs for (x+d,o), then reachs for (x,o). 
    * Similar to grasp but without final hand action. 
    */
    virtual bool touch(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * Tap the given target (combined action).
    * @param x1 the fisrt 3-d target position [m]. 
    * @param o1 the first 4-d hand orientation (given in axis-angle 
    *           representation: ax ay az angle in rad).
    * @param x2 the second 3-d target position [m]. 
    * @param o2 the second 4-d hand orientation (given in axis-angle
    *           representation: ax ay az angle in rad).
    * @param execTime the arm action execution time only while 
    *          tapping [s] (to be specified iff different from
    *          default value).
    * @return true/false on success/fail. 
    *  
    * @note internal implementation (pseudo-code): 
    * @code 
    * ...
    * pushAction(x1,o1,"karate_hand");
    * pushAction(x2,o2,execTime);
    * pushAction(x1,o1); 
    * ... 
    * @endcode 
    *  
    * It reachs for (x1,o1), then reachs for (x2,o2) and then again
    * for (x1,o1).
    */
    virtual bool tap(const yarp::sig::Vector &x1, const yarp::sig::Vector &o1,
                     const yarp::sig::Vector &x2, const yarp::sig::Vector &o2,
                     const double execTime=ACTIONPRIM_DISABLE_EXECTIME);
};


// forward declaration
class affActionPrimitivesLayer2;


// callback for executing final grasp after contact
class liftAndGraspCallback : public affActionPrimitivesCallback
{
protected:
    affActionPrimitivesLayer2 *action;

public:
    liftAndGraspCallback(affActionPrimitivesLayer2 *_action) :
                         action(_action) { }

    virtual void exec();
};


/**
* \ingroup affActionPrimitives
*
* A class that inherits from @ref affActionPrimitivesLayer1 and 
* integrates the force-torque sensing in order to stop the limb 
* while reaching as soon as a contact with external objects is 
* detected. 
*/
class affActionPrimitivesLayer2 : public affActionPrimitivesLayer1
{
protected:
    bool skipFatherPart;
    bool meConfigured;
    bool contact;

    double ext_force_thres;

    yarp::dev::PolyDriver  *polyTorso;
    yarp::dev::IEncoders   *encTorso;

    ctrl::AWLinEstimator   *velEst;
    ctrl::AWQuadEstimator  *accEst;
    iDyn::iCubArmDyn       *dynArm;
    iDyn::iDynInvSensor    *dynSensor;
    iDyn::iFTransformation *dynTransformer;

    liftAndGraspCallback   *execLiftAndGrasp;

    yarp::os::BufferedPort<yarp::sig::Vector> *ftPortIn;

    yarp::sig::Vector q;
    yarp::sig::Vector dq;
    yarp::sig::Vector d2q;

    yarp::sig::Vector wrenchModel;
    yarp::sig::Vector wrenchOffset;
    yarp::sig::Vector wrenchMeasured;
    yarp::sig::Vector wrenchExternal;

    yarp::sig::Vector grasp_d2;
    yarp::sig::Vector grasp_o;

    yarp::sig::Vector encDataTorso;
    yarp::sig::Vector encDataArm;

    friend class liftAndGraspCallback;

    virtual void init();
    virtual void postReachCallback();
    virtual void run();

public:
    /**
    * Default Constructor. 
    */
    affActionPrimitivesLayer2();

    /**
    * Constructor. 
    * @param opt the Property used to configure the object after its
    *            creation.
    */
    affActionPrimitivesLayer2(yarp::os::Property &opt);

    /**
    * Destructor. 
    *  
    * @note it calls the close() method. 
    */
    virtual ~affActionPrimitivesLayer2();

    /**
    * Configure the object.
    * @param opt the Property used to configure the object after its
    *            creation.
    *  
    * @note To be called after object creation. 
    *  
    * Further available options are: 
    *  
    * @b ext_force_thres <double>: specify the maximum external 
    *    force magnitude applied to the end-effector in order to
    *    detect contact between end-effector and objects while
    *    reaching.
    *  
    * @note A port called <i> /<local>/<part>/ft:i </i> is open to
    *       acquire data provided by \ref force/torque sensor.
    */
    virtual bool open(yarp::os::Property &opt);

    /**
    * More evolute version of grasp. It exploits the contact 
    * detection in order to lift up a bit the hand prior to 
    * grasping (this happens only after contact).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d1 the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @param d2 the displacement [m] that identifies the amount of 
    *           the lift after the contact.
    * @return true/false on success/fail. 
    */
    virtual bool grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d1, const yarp::sig::Vector &d2);

    /**
    * The usual grasp is still available.
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation: ax ay az angle
    *          in rad).
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @return true/false on success/fail. 
    */
    virtual bool grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * Retrieve the current wrench on the end-effector.
    * @param wrench a vector containing the external forces/moments 
    *               acting on the end-effector.
    * @return true/false on success/fail. 
    */
    virtual bool getExtWrench(yarp::sig::Vector &wrench);

    /**
    * Retrieve the current threshold on the external force used to 
    * stop the limb while reaching. 
    * @param thres where to return the threshold.
    * @return true/false on success/fail. 
    */
    virtual bool getExtForceThres(double &thres);

    /**
    * Set the threshold on the external force used to stop the limb
    * while reaching. 
    * @param thres the new threshold.
    * @return true/false on success/fail. 
    */
    virtual bool setExtForceThres(const double thres);

    /**
    * Check whether the reaching has been stopped due to a contact 
    * with external objects. 
    * @param f the result of the check. 
    * @return true/false on success/fail. 
    */
    virtual bool checkContact(bool &f);
};

}

#endif


