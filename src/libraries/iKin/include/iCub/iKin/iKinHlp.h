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

#define IKIN_ALMOST_ZERO    1e-6

#include <yarp/os/Bottle.h>
#include <yarp/sig/all.h>

#include <iCub/iKin/iKinFwd.h>


namespace iCub
{

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
    static bool getDesiredOption(const yarp::os::Bottle &reply, yarp::sig::Vector &xdhat,
                                 yarp::sig::Vector &odhat, yarp::sig::Vector &qdhat);

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
    * Appends to a bottle all data needed to modify joints rest 
    * position. 
    * @param b is the bottle where to append the data.
    * @param restPos is the vector of new joints rest position 
    *                expressed in [deg].
    */
    static void addJointsResPosOption(yarp::os::Bottle &b, const yarp::sig::Vector &restPos);

    /**
    * Appends to a bottle all data needed to modify joints rest 
    * weights. 
    * @param b is the bottle where to append the data.
    * @param restWeights is the vector of new joints rest weights. 
                                                                    */
    static void addJointsRestWeightsOption(yarp::os::Bottle &b, const yarp::sig::Vector &restWeights);

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
    * @note useful for some synchronization. 
    */
    static void addTokenOption(yarp::os::Bottle &b, const double token);    

    /**
    * Retrieves commanded target data from a bottle.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static yarp::os::Bottle *getTargetOption(const yarp::os::Bottle &b);

    /**
    * Retrieves the end-effector pose data.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static yarp::os::Bottle *getEndEffectorPoseOption(const yarp::os::Bottle &b);

    /**
    * Retrieves the joints configuration data.
    * @param b is the bottle containing the data to be retrieved.
    * @return a pointer to the sub-bottle containing the retrieved 
    *         data.
    */
    static yarp::os::Bottle *getJointsOption(const yarp::os::Bottle &b);

    /**
    * Retrieves the token from the bottle. 
    * @param b is the bottle containing the data to be retrieved. 
    * @param token is the pointer where to return the token.
    * @return true iff the token property is present within the 
    *         bottle b
    */
    static bool getTokenOption(const yarp::os::Bottle &b, double *token);

    /**
    * Retrieves current fixation point given the current kinematics
    * configuration of the eyes. 
    * @param eyeL the current configuration of the left eye. 
    * @param eyeR the current configuration of the right eye. 
    * @param fp the Vector where to store the fixation point 
    *           coordinates.
    * @return true iff the computation of the quantities went well, 
    *         false otherwise.
    */
    static bool computeFixationPointData(iKinChain &eyeL, iKinChain &eyeR,
                                         yarp::sig::Vector &fp);

    /**
    * Retrieves current fixation point and its Jacobian wrt eyes 
    * tilt-pan-vergence dofs given the current kinematics 
    * configuration of the eyes. 
    * @param eyeL the current configuration of the left eye. 
    * @param eyeR the current configuration of the right eye. 
    * @param fp the Vector where to store the fixation point 
    *           coordinates.
    * @param J the fixation point Jacobian wrt eyes configuration. 
    * @return true iff the computation of the quantities went well, 
    *         false otherwise.
    */
    static bool computeFixationPointData(iKinChain &eyeL, iKinChain &eyeR,
                                         yarp::sig::Vector &fp, yarp::sig::Matrix &J);
};

}

}

#endif

