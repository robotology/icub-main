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
* It allows to yield primitive actions such as reach(), 
* closeHand(); it defines some combined actions such as grasp(),
* tap(); it allows to handle an actions queue in order to 
* specify a desired combination of primitive actions. 
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

    int iMin, iMax;

    yarp::sig::Vector xd;
    yarp::sig::Vector od;

    yarp::sig::Vector thresholds;
    yarp::sig::Vector fingerOpenPos;
    yarp::sig::Vector fingerClosePos;
    std::set<int> enabledJoints;
    std::set<int> activeJoints;
    std::map<const std::string, yarp::sig::Matrix> sensingConstants;
    HandMetrics      *handMetrics;
    FunctionSmoother *fs;

    typedef struct
    {
        // wait action
        bool waitState;
        double tmo;
        // reach action
        bool execReach;
        yarp::sig::Vector x;
        yarp::sig::Vector o;
        // hand action
        bool (affActionPrimitives::*handAction)(const bool);
    } Action;

    std::deque<Action> actionsQueue;

    virtual bool handleTorsoDOF(yarp::os::Property &opt, const std::string &key, const int j);
    virtual bool getVector(yarp::os::Property &opt, const std::string &key, yarp::sig::Vector &v, const int offs);
    virtual void stopBlockedJoints(std::set<int> &activeJoints);
    virtual bool handMotionDone(const std::set<int> &joints);
    virtual bool moveHand(const yarp::sig::Vector &fingerPos, const bool sync);
    virtual bool wait(const double tmo);

    void init();    
    bool execQueuedAction();
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
    * \b local <string>: specify a name used to open local ports. 
    *  
    * \b hand_calibration_file <string>: complete path to the hand 
    *    calibration file.
    *  
    * \b torso_pitch <string>: if "on" it enables the control of the 
    *    pitch of the torso.
    * \b torso_pitch_min <double>: set the pitch minimum value 
    *    [deg].
    * \b torso_pitch_max <double>: set the pitch maximum value 
    *    [deg].
    *  
    * \b torso_roll <string>: if "on" it enables the control of the 
    *    roll of the torso.
    * \b torso_roll_min <double>: set the roll minimum value [deg].
    * \b torso_roll_max <double>: set the roll maximum value [deg].
    *  
    * \b torso_yaw <string>: if "on" it enables the control of the 
    *    yaw of the torso.
    * \b torso_yaw_min <double>: set the yaw minimum value [deg].
    * \b torso_yaw_max <double>: set the yaw maximum value [deg].
    *  
    * \b fingers_open_poss <list>: to get the positions [deg] used 
    *    for opening the hand.
    *  
    * \b fingers_close_poss <list>: to get the positions [deg] used 
    *    for closing the hand.
    *  
    * \b fingers_vel <double>: set the fingers velocity [deg/s].
    */
    bool open(yarp::os::Property &opt);

    /**
    * Check if the object has been correctly open. 
    * @return true/fail on success/fail. 
    */
    virtual bool isValid();

    /**
    * Deallocate the object.
    */
    void close();

    /**
    * Reach for a given target (arm primitive action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation) [rad].
    * @param sync if true wait until the action is accomplished. 
    * @return true/false on success/fail. 
    */
    virtual bool reach(const yarp::sig::Vector &x, const yarp::sig::Vector &o, const bool sync=false);

    /**
    * No action for the hand (hand primitive action).
    * @param sync if true wait until the action is accomplished. 
    * @return true/false on success/fail. 
    *  
    * \note useful when inserting a new item in the action queue. 
    */
    virtual bool nopHand(const bool sync=false);

    /**
    * Open the hand (hand primitive action).
    * @param sync if true wait until the action is accomplished. 
    * @return true/false on success/fail. 
    */
    virtual bool openHand(const bool sync=false);

    /**
    * Close the hand (hand primitive action).
    * @param sync if true wait until the action is accomplished. 
    * @return true/false on success/fail. 
    */
    virtual bool closeHand(const bool sync=false);

    /**
    * Grasp the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/grasping
    *          (given in axis-angle representation) [rad].
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          grasping.
    * @param sync if true wait until the action is accomplished. 
    * @return true/false on success/fail. 
    *  
    * \note It reachs for (x+d,o), then reachs for (x,o) closing the
    *       hand at the same time.
    */
    virtual bool grasp(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d, const bool sync=false);

    /**
    * Touch the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching/touching
    *          (given in axis-angle representation) [rad].
    * @param d the displacement [m] wrt the target position that 
    *          identifies a location to be reached prior to
    *          touching.
    * @param sync if true wait until the action is accomplished. 
    * @return true/false on success/fail. 
    *  
    * \note It reachs for (x+d,o), then reachs for (x,o). Similar to 
    *       grasp but without hand action.
    */
    virtual bool touch(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                       const yarp::sig::Vector &d, const bool sync=false);

    /**
    * Tap the given target (combined action).
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while tapping (given in
    *          axis-angle representation) [rad].
    * @param d the displacement [m]; see the note below.
    * @param sync if true wait until the action is accomplished. 
    * @return true/false on success/fail. 
    *  
    * \note It reachs for (x,o), then reachs for (x+d,o) and then 
    *       again for (x,o).
    */
    virtual bool tap(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                     const yarp::sig::Vector &d, const bool sync=false);

    /**
    * Insert a combination of arm and hand primitive actions in the 
    * actions queue. 
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation) [rad].
    * @param handAction a pointer to the member function that yields 
    *                   the action (e.g. closeHand)
    * @return true/false on success/fail. 
    *  
    * \note Some examples: 
    *  
    * the call \b pushAction(x,o,&affActionPrimitives::closeHand) 
    * pushes the combined action of reach(x,o) and closeHand() into 
    * the queue; the action will be executed as soon as all the 
    * previous items in the queue will have been accomplished. 
    */
    bool pushAction(const yarp::sig::Vector &x, const yarp::sig::Vector &o, 
                    bool (affActionPrimitives::*handAction)(const bool));

    /**
    * Insert an arm primitive action in the actions queue.
    * @param x the 3-d target position [m]. 
    * @param o the 4-d hand orientation used while reaching (given 
    *          in axis-angle representation) [rad].
    * @return true/false on success/fail. 
    */
    bool pushAction(const yarp::sig::Vector &x, const yarp::sig::Vector &o);

    /**
    * Insert a hand primitive action in the actions queue.
    * @param handAction a pointer to the member function that yields 
    *                   the action (e.g. closeHand)
    * @return true/false on success/fail. 
    */
    bool pushAction(bool (affActionPrimitives::*handAction)(const bool));

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
    * querying the action status; also disable this blocking feature
    * for future calls with sync switch on. 
    * @return true/false on success/fail. 
    *  
    * \note useful to allow a graceful stop of the application.
    */
    virtual bool syncCheckInterrupt();

    /**
    * Reinstate the blocking feature for future calls with sync 
    * switch on. 
    * @return true/false on success/fail. 
    */
    virtual bool syncCheckReinstate();
};

#endif


