/**
 * \defgroup iKinHlp iKinHlp
 *  
 * @ingroup iKin 
 *
 * Helper class to deal with Cartesian Solver options.
 *  
 * Date: first release 12/10/2009
 *
 * \author Ugo Pattacini
 *
 */ 

#ifndef __IKINHLP_H__
#define __IKINHLP_H__

#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>


namespace iKin
{

/**
* \ingroup iKinHlp
*
* Helper class providing useful methods to deal with Cartesian 
* Solver options. 
*/
class CartesianHelper
{
protected:
    static void addVectorOption(yarp::os::Bottle &b, const int vcb, const yarp::sig::Vector &v);

public:
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
    * Appends to a bottle all data needed to modify torso rest 
    * position. 
    * @param b is the bottle where to append the data.
    * @param torsoRest is the vector of new torso rest position for 
    *                  pitch/roll/yaw angles [deg].
    */
    static void addTorsoRestOption(yarp::os::Bottle &b, const yarp::sig::Vector &torsoRest);

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
    * Appends to a bottle a token to be exchanged with the solver.
    * @param token. 
    *  
    * \note useful for some synchronization. 
    */
    static void addTokenOption(yarp::os::Bottle &b, const double token);    

    /**
    * Retrieves commanded target data from a bottle.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static yarp::os::Bottle *getTargetOption(yarp::os::Bottle &b);

    /**
    * Retrieves the end-effector pose data.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static yarp::os::Bottle *getEndEffectorPoseOption(yarp::os::Bottle &b);

    /**
    * Retrieves the joints configuration data.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static yarp::os::Bottle *getJointsOption(yarp::os::Bottle &b);

    /**
    * Retrieves the token from the bottle. 
    * @param b is the bottle containing the data to be retrieved. 
    * @param token is the pointer where to return the token.
    * @return true iff the token property is present within the 
    *         bottle b
    */
    static bool getTokenOption(yarp::os::Bottle &b, double *token);
};

}

#endif

