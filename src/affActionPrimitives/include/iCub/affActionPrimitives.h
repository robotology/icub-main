/**
 * \defgroup affActionPrimitives affActionPrimitives
 *  
 * @ingroup icub_module 
 *  
 * Definition of primitive actions for dealing with affordances.
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __AFFACTIONPRIMITIVES_H__
#define __AFFACTIONPRIMITIVES_H__

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/sig/Vector.h>

#include <iCub/handMetrics.h>

#include <deque>
#include <map>
#include <set>


/**
* \ingroup affActionPrimitives
*
* The base class defining affordance actions. 
*  
* It allows to execute arm and hand primitive actions (such as 
* reach) and to combine them in an actions queue.
*/
class affActionPrimitives : public yarp::os::RateThread
{
protected:
    yarp::dev::PolyDriver        *polyHand;
    yarp::dev::PolyDriver        *polyCart;
    yarp::dev::IEncoders         *encCtrl;
    yarp::dev::IPidControl       *pidCtrl;
    yarp::dev::IAmplifierControl *ampCtrl;
    yarp::dev::IPositionControl  *posCtrl;
    yarp::dev::ICartesianControl *cartCtrl;

    yarp::os::Semaphore *mutex;
    void                *motionDoneEvent;

    bool armMoveDone;
    bool handMoveDone;
    bool latchArmMoveDone;
    bool latchHandMoveDone;

    bool configured;
    bool closed;
    bool checkEnabled;

    double waitTmo;
    double latchTimer;
    double t0;

    int jHandMin;
    int jHandMax;

    yarp::sig::Vector xd;
    yarp::sig::Vector od;
    yarp::sig::Vector smallOffs;

    yarp::sig::Vector thresholds;
    std::set<int> enabledJoints;
    std::set<int> activeJoints;
    std::map<const std::string, yarp::sig::Matrix> sensingConstants;
    HandMetrics      *handMetrics;
    FunctionSmoother *fs;

    typedef struct
    {
        yarp::sig::Vector poss;
        yarp::sig::Vector vels;
    } HandWayPoint;

    typedef struct
    {
        // wait action
        bool waitState;
        double tmo;
        // reach action
        bool execArm;
        yarp::sig::Vector x;
        yarp::sig::Vector o;
        // hand action
        bool execHand;
        HandWayPoint handWP;
    } Action;

    std::deque<Action> actionsQueue;
    std::map<std::string,std::deque<HandWayPoint> > handSeqMap;

    virtual bool handleTorsoDOF(yarp::os::Property &opt, const std::string &key,
                                const int j);
    virtual bool configHandSeq(yarp::os::Property &opt);
    virtual bool pushAction(const bool execArm, const yarp::sig::Vector &x,
                            const yarp::sig::Vector &o, const bool execHand,
                            const HandWayPoint &handWP);
    virtual bool cmdArm(const yarp::sig::Vector &x, const yarp::sig::Vector &o);
    virtual bool cmdHand(const HandWayPoint &handWP);
    virtual bool wait(const double tmo);
    virtual void stopBlockedJoints(std::set<int> &activeJoints);
    virtual bool handMotionDone(const std::set<int> &joints);

    void init();    
    bool execQueuedAction();
    bool execPendingHandAction();
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
    * \note it calls the close() method. 
    */
    virtual ~affActionPrimitives();

    /**
    * Configure the object.
    * @param opt the Property used to configure the object after its
    *            creation.
    *  
    * \note  to be called after object creation. 
    *  
    * \note Available options are: 
    *  
    * \b robot <string>: the robot name to connect to (e.g. icub). 
    *  
    * \b part <string>:  the arm to be controlled (e.g. left_arm). 
    *  
    * \b thread_period <int>: the thread period [ms] which selects 
    *    the time granularity as well.
    *  
    * \b traj_time <double>: the arm movement execution time [s]. 
    *  
    * \b reach_tol <double>: the reaching tolerance [m]. 
    *  
    * \b local <string>: specify a name used to open local ports. 
    *
    * \b torso_pitch <string>: if "on" it enables the control of the 
    *    pitch of the torso.
    *  
    * \b torso_pitch_min <double>: set the pitch minimum value 
    *    [deg].
    *  
    * \b torso_pitch_max <double>: set the pitch maximum value 
    *    [deg].
    *  
    * \b torso_roll <string>: if "on" it enables the control of the 
    *    roll of the torso.
    *  
    * \b torso_roll_min <double>: set the roll minimum value [deg]. 
    *  
    * \b torso_roll_max <double>: set the roll maximum value [deg].
    *  
    * \b torso_yaw <string>: if "on" it enables the control of the 
    *    yaw of the torso.
    *  
    * \b torso_yaw_min <double>: set the yaw minimum value [deg]. 
    *  
    * \b torso_yaw_max <double>: set the yaw maximum value [deg]. 
    *  
    * \b hand_calibration_file <string>: complete path to the hand 
    *    calibration file.
    *  
    * \b hand_sequences_file <string>: complete path to the file 
    *    containing the hand motions sequences.<br />Here is the
    *    format of motion sequences:
    *  
    *  \code
    *  [GENERAL]
    *  numSequences ***
    *  
    *  [SEQ_0]
    *  key ***
    *  numWayPoints ***
    *  wp_0  (poss (10.0 20.0 ...)) (vels (20.0 20.0 ...))
    *  wp_1  ***
    *  ...
    *  
    *  [SEQ_1]
    *  ...
    *  \endcode
    */
    bool open(yarp::os::Property &opt);

    /**
    * Check if the object is initialized correctly. 
    * @return true/fail on success/fail. 
    */
    virtual bool isValid();

    /**
    * Deallocate the object.
    */
    void close();

    /**
    * Insert a combination of arm and hand primitive actions in the 
    * actions queue. 
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation) [rad].
    * @param handSeqKey the hand sequence key.  
    * @return true/false on success/fail. 
    *  
    * \note Some examples: 
    *  
    * the call \b pushAction(x,o,"close_hand") pushes the combined 
    * action of reach(x,o) and hand "close_hand" sequence into the 
    * queue; the action will be executed as soon as all the previous 
    * items in the queue will have been served. 
    */
    bool pushAction(const yarp::sig::Vector &x, const yarp::sig::Vector &o, 
                    const std::string &handSeqKey);

    /**
    * Insert the arm-primitive action reach for target in the 
    * actions queue. 
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation) [rad].
    * @return true/false on success/fail. 
    */
    bool pushAction(const yarp::sig::Vector &x, const yarp::sig::Vector &o);

    /**
    * Insert a hand-primitive action in the actions queue.
    * @param handSeqKey the hand sequence key.   
    * @return true/false on success/fail. 
    */
    bool pushAction(const std::string &handSeqKey);

    /**
    * Insert a wait state in the actions queue.
    * @param tmo is the wait timeout [s]. 
    * @return true/false on success/fail. 
    */
    bool pushWaitState(const double tmo);

    /**
    * Empty the actions queue. 
    * @return true/false on success/fail.
    */
    bool clearActionsQueue();

    /**
    * Define an hand WayPoint (WP) to be added at the bottom of the 
    * hand motion sequence pointed by the key. 
    * @param handSeqKey the hand sequence key.
    * @param poss the fingers WP positions to be attained [deg].
    * @param vels the fingers velocities [deg/s].
    * @return true/false on success/fail. 
    *  
    * \note this method creates a new empty sequence referred by the
    *       passed key if the key does not point to any valid
    *       sequence; hence the couple (poss,vels) will be the first
    *       WP of the new sequence.
    */
    bool addHandSeqWP(const std::string &handSeqKey, const yarp::sig::Vector &poss,
                      const yarp::sig::Vector vels);

    /**
    * Check whether a sequence key is defined.
    * @param handSeqKey the hand sequence key.
    * @return true iff valid key. 
    */
    bool isValidHandSeq(const std::string &handSeqKey);

    /**
    * Remove an already existing hand motion sequence.
    * @param handSeqKey the hand sequence key.
    * @return true/false on success/fail. 
    */
    bool removeHandSeq(const std::string &handSeqKey);

    /**
    * Return the whole list of available hand sequence keys.
    * @return the list of available hand sequence keys.
    */
    std::deque<std::string> getHandSeqList();

    /**
    * Return the current end-effector position. 
    * @param x the current 3-d hand position [m].
    * @param o the current 4-d hand orientation (given in the 
    *          axis-angle representation) [rad].
    * @return true/false on success/fail.
    */
    virtual bool getPose(yarp::sig::Vector &x, yarp::sig::Vector &o);

    /**
    * Stop any ongoing arm/hand movements.
    * @return true/false on success/fail. 
    *  
    * \note it empty out the actions queue. 
    */
    virtual bool stopControl();

    /**
    * Check whether the action is accomplished or still ongoing.
    * @param f the result of the check. 
    * @param sync if true wait until the action is accomplished 
    *             (blocking call).
    * @return true/false on success/fail. 
    *  
    * \note Actually the check is performed on the content of the 
    *       actions queue so that the blocking call returns as soon
    *       as the queue is empty.
    */
    virtual bool checkActionsDone(bool &f, const bool sync=false);

    /**
    * Suddenly interrupt any blocking call that is pending on 
    * querying the action status. 
    * @param disable disable the blocking feature for future 
    * calls with sync switch on; useful to allow a graceful stop of 
    * the application. \see syncCheckReinstate to reinstate it.
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
* affActionPrimitives father class. 
*  
* It internally predeclares (without actually defining) a set of
* hand sequence motions key ("open_hand" and "close_hand") that 
* are used for grasp(), touch() and tap() actions. 
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
    *          (given in axis-angle representation) [rad].
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @return true/false on success/fail. 
    *  
    * \note internal implementation: 
    * ... 
    * pushAction(x+d,o,"open_hand"); 
    * pushAction(x,o); 
    * pushAction("close_hand") 
    * ...
    *  
    * It reachs for (x+d,o) opening the hand, then reachs for (x,o)
    * closing the hand at the same time. 
    */
    virtual bool grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * Touch the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/touching
    *          (given in axis-angle representation) [rad].
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          touching.
    * @return true/false on success/fail. 
    *  
    * \note internal implementation: 
    * ... 
    * pushAction(x+d,o,"open_hand"); 
    * pushAction(x,o); 
    * ... 
    *  
    * It reachs for (x+d,o), then reachs for (x,o). 
    * Similar to grasp but without hand action. 
    */
    virtual bool touch(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d);

    /**
    * Tap the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while tapping (given in
    *          axis-angle representation) [rad].
    * @param d the displacement [m]; see the note below.
    * @return true/false on success/fail. 
    *  
    * \note internal implementation: 
    * ...
    * pushAction(x,o,"open_hand");
    * pushAction(x+d,o);
    * pushAction(x,o);
    * ...
    *  
    * It reachs for (x,o), then reachs for (x+d,o) and then again
    * for (x,o).
    */
    virtual bool tap(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                     const yarp::sig::Vector &d);

};


#endif


