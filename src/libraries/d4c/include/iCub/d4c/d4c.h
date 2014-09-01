/* 
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
 * Authors: Ilaria Gori, Ugo Pattacini
 * email:  ilaria.gori@iit.it, ugo.pattacini@iit.it
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
 * \defgroup d4c D4C
 *  
 * @ingroup icub_libraries 
 *  
 * Abstract class to deal with <b>Dynamic Force Field 
 * Control</b> (<a 
 * href="http://wiki.icub.org/images/5/5a/DForC.pdf">D4C</a>). 
 *  
 * For further details please refer to the <a 
 * href="http://wiki.icub.org/wiki/D4C Framework">wiki</a>.
 *  
 * \author Ilaria Gori, Ugo Pattacini 
 *  
 */ 

#ifndef __D4C_H__
#define __D4C_H__

#include <deque>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#define D4C_DEFAULT_MAXITERATIONS       1000
#define D4C_DEFAULT_TS_DISABLED         0.0

namespace iCub
{

namespace d4c
{

/**
* @ingroup d4c
*
* The Definition of the D4C Interface.
*/
class D4C
{
public:
    /**
    * Configure the client or the server part of d4c framework.
    * @param options contains the set of options in form of a 
    *                Property object.
    *  
    * Available options for the client are: 
    *  
    * \b remote <string>: example (remote /d4c_server), specifies 
    *    the server port stem-name to connect to.
    *  
    * \b local <string>: example (local /d4c_client), specifies the 
    *    the client stem-name to be used for opening ports.
    *  
    * \b carrier <string>: example (carrier udp), specifies the 
    *    protocol used to connect yarp streaming ports.
    *  
    * \b verbosity <int>: example (verbosity 3), specifies the 
    *    verbosity level of print-outs messages.
    *  
    * Available options for the server are: 
    *  
    * \b device <string>: example (device 
    *    cartesiancontrollerclient), specifies the name of the
    *    device that implements the cartesian interface.
    *  
    * \b name <string>: example (name d4c_server), specifies the 
    *    name of the server to be launched.
    *  
    * \b robot <string>: example (robot icub), specifies the name of
    *    the robot to connect to.
    *  
    * \b part <string>: example (part right_arm), specifies the 
    *    robot part to be controlled.
    *  
    * \b period <int>: example (period 20), specifies the period 
    *    given in [ms] to generate the d4c field.
    *  
    * \b offline: example (offline), lets the client/server start in 
    *    offline mode.
    *  
    * \b verbosity <int>: example (verbosity 3), specifies the 
    *    verbosity level of print-outs messages.
    *  
    * @return true/false if successful/failed.
    */
    virtual bool open(const yarp::os::Property &options) = 0;

    /**
    * Stop the client/server and dispose it. Called by destructor.
    */
    virtual void close() = 0;

    /**
    * Request for the creation of a new d4c entity which can be
    * either a target or an obstacle. 
    * @param options contains the set of options in form of a 
    *                Property object.
    * @param item here is returned the unique item identifier to be
    *             used for later access.
    *  
    * Options common to all items: 
    *  
    * \b name <string>: example (name item_name), specifies the item
    *    name.
    *  
    * \b type <string>: example (type tag), specifies the item type 
    *    on whose basis the configuration is properly carried out.
    *  
    * \b active <string>: example (active on), specifies whether the 
    *    item should be taken into account for generating the field
    *    ("on") or not ("off").
    *  
    * \b center <(double(x) double(y) double(z))>: example (center 
    *    (-0.3 0.0 0.1)), specifies the xyz position of the item
    *    wrt the robot's root reference frame.
    *  
    * \b orientation <(double(ax) double(ay) double(az) 
    *    double(theta))>: example (orientation (1.0 0.0 0.0 3.14)),
    *    specifies the orientation of the item's frame in axis/angle
    *    representation given wrt the robot's root reference frame.
    *  
    * \b color <(double(r) double(g) double(b))>: example (color 
    *    (255.0 0.0 0.0)), specifies the color of the ellipsoid in
    *    RGB format.
    *  
    * \b radius <(double(rx) double(ry) double(rz))>: example 
    *    (radius (0.01 0.02 0.03)), specifies the dimension of the
    *    ellipsoid radii (in [m]) representing the item wrt item's
    *    reference frame.
    *  
    * The type "target_msd" implements a target acting as a 
    * mass-spring-damper system on the end-effector within the d4c 
    * framework. 
    *  
    * Options specific to the "target_msd" item: 
    *  
    * \b K <double>: example (K 0.1), specifies the spring 
    *    stiffness.
    *  
    * \b D <double>: example (D 0.1), specifies the damping factor. 
    *  
    * The type "obstacle_gaussian" implements an obstacle acting as 
    * a multivariate gaussian repulsive field within the d4c 
    * framework. 
    *  
    * Options specific to the "obstacle_gaussian" item: 
    *  
    * \b G <double>: example (G 0.1), specifies the gain of the 
    *    gaussian field.
    *  
    * \b cut_tails <string>: example (cut_tails on), specifies 
    *    whether to cut or not the tails of the gaussian field out
    *    of the space occupied by the obstacle.
    *  
    * @return true/false if successful/failed.
    */
    virtual bool addItem(const yarp::os::Property &options, int &item) = 0;

    /**
    * Remove one item from the items table.
    * @param item the item identifier to be erased. 
    * @return true/false if successful/failed.
    */
    virtual bool eraseItem(const int item) = 0;

    /**
    * Purge all the items contained in the table.
    * @return true/false if successful/failed.
    */
    virtual bool clearItems() = 0;

    /**
    * Retrieve the list of all available items.
    * @param items the Bottle containing the retrieved list.
    * @return true/false if successful/failed.
    */
    virtual bool getItems(yarp::os::Bottle &items) = 0;

    /**
    * Allow changing the properties of an existing item.
    * @param item the item identifier. 
    * @param options the Property object containing the new item 
    *                values.
    * @return true/false if successful/failed.
    */
    virtual bool setProperty(const int item, const yarp::os::Property &options) = 0;

    /**
    * Retrieve the properties of an existing item.
    * @param item the item identifier. 
    * @param options the Property object containing the item 
    *                properties.
    * @return true/false if successful/failed.
    */
    virtual bool getProperty(const int item, yarp::os::Property &options) = 0;

    /**
    * Enable the generation of the d4c field.
    * @return true/false if successful/failed.
    */
    virtual bool enableField() = 0;

    /**
    * Disable the generation of the d4c field.
    * @return true/false if successful/failed.
    */
    virtual bool disableField() = 0;

    /**
    * Retrieve the current status of the d4c field. 
    * @param status true/false on enabled/disabled d4c field. 
    * @return true/false if successful/failed.
    */
    virtual bool getFieldStatus(bool &status) = 0;

    /**
    * Enable the control of the robot part.
    * @return true/false if successful/failed.
    *  
    * @note if simulation is on, it gets automatically disabled.
    */
    virtual bool enableControl() = 0;

    /**
    * Disable the control of the robot part.
    * @return true/false if successful/failed.
    */
    virtual bool disableControl() = 0;

    /**
    * Retrieve the current status of the robot part control. 
    * @param status true/false on controlled/uncontrolled robot 
    *               part.
    * @return true/false if successful/failed.
    */
    virtual bool getControlStatus(bool &status) = 0;

    /**
    * Enable the simulation of the robot part.
    * @return true/false if successful/failed. 
    *  
    * @note if control is on, it gets automatically disabled.
    */
    virtual bool enableSimulation() = 0;

    /**
    * Disable the simulation of the robot part.
    * @return true/false if successful/failed.
    */
    virtual bool disableSimulation() = 0;

    /**
    * Retrieve the current status of the robot part simulation.
    * @param status true/false.
    * @return true/false if successful/failed.
    */
    virtual bool getSimulationStatus(bool &status) = 0;

    /**
    * Allow changing the thread period for d4c field generation. 
    * @param period the new thread period in [ms].
    * @return true/false if successful/failed.
    */
    virtual bool setPeriod(const int period) = 0;

    /**
    * Retrieve the current thread period for d4c field generation.
    * @param period the current thread period in [ms].
    * @return true/false if successful/failed.
    */
    virtual bool getPeriod(int &period) = 0;

    /**
    * Allow setting the trajectory state equal to the currently 
    * attached tool state (or end-effector state if no tool is 
    * attached). 
    * @return true/false if successful/failed.
    */
    virtual bool setPointStateToTool() = 0;

    /**
    * Attach a tool frame to the end-effector. 
    * @param x a 3-d vector describing the position of the tool wrt 
    *          the end-effector (meters).
    * @param o a 4-d vector describing the orientation of the tool 
    *          wrt the end-effector (axis-angle notation).
    * @return true/false if successful/failed.
    */
    virtual bool attachToolFrame(const yarp::sig::Vector &x, const yarp::sig::Vector &o) = 0;

    /**
    * Retrieve the frame of the tool currently attached to the 
    * end-effector. 
    * @param x a 3-d vector containing the position of the tool wrt 
    *          the end-effector (meters).
    * @param o a 4-d vector containing the orientation of the tool 
    *          wrt the end-effector (axis-angle notation).
    * @return true/false if successful/failed.
    */
    virtual bool getToolFrame(yarp::sig::Vector &x, yarp::sig::Vector &o) = 0;

    /**
    * Remove the tool currently attached to the end-effector.  
    * @return true/false if successful/failed. 
    *  
    * @note The end-effector is the tool again.
    */
    virtual bool removeToolFrame() = 0;

    /**
    * Retrieve the coordinates of the tool currently attached to the
    * end-effector. 
    * @param x a 3-d vector containing the position of the tool 
    *          (meters).
    * @param o a 4-d vector containing the orientation of the tool 
    *          (axis-angle notation).
    * @return true/false if successful/failed. 
    *  
    * @note if no tool is attached, clearly the end-effector pose is
    *       returned.
    */
    virtual bool getTool(yarp::sig::Vector &x, yarp::sig::Vector &o) = 0;

    /**
    * Allow chainging the trajectory state generated by the d4c 
    * framework. 
    * @param x a 3-d Vector containing the new position [m]. 
    * @param o a 4-d Vector containing the new orientation 
    *          (axis/angle).
    * @param xdot a 3-d Vector containing the new translational 
    *             velocity [m/s].
    * @param odot a 4-d Vector containing the new rotational 
    *             velocity (axis/angle).
    * @return true/false if successful/failed.
    */
    virtual bool setPointState(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                               const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot) = 0;

    /**
    * Allow changing the orientation state of the trajectory 
    * generated by the d4c framework. 
    * @param o a 4-d Vector containing the new orientation 
    *          (axis/angle).
    * @param odot a 4-d Vector containing the new rotational 
    *             velocity (axis/angle).
    * @return true/false if successful/failed.
    */
    virtual bool setPointOrientation(const yarp::sig::Vector &o, const yarp::sig::Vector &odot) = 0;

    /**
    * Retrieve the current trajectory state generated by the d4c 
    * framework. 
    * @param x a 3-d Vector containing the current position [m]. 
    * @param o a 4-d Vector containing the current orientation 
    *          (axis/angle).
    * @param xdot a 3-d Vector containing the current translational 
    *             velocity [m/s].
    * @param odot a 4-d Vector containing the current rotational 
    *             velocity (axis/angle).
    * @return true/false if successful/failed.
    */
    virtual bool getPointState(yarp::sig::Vector &x, yarp::sig::Vector &o,
                               yarp::sig::Vector &xdot, yarp::sig::Vector &odot) = 0;

    /**
    * Retrieve the current field as generated by the d4c framework.
    * @param field a 7-d Vector containing the current field. 
    * @return true/false if successful/failed.
    */
    virtual bool getField(yarp::sig::Vector &field) = 0;

    /**
    * Sample step by step the trajectory the end-effector will 
    * follow as constrained by the d4c field. 
    * @param xhat a 3-d Vector containing the simulated position 
    *             [m].
    * @param ohat a 4-d Vector containing the simulated orientation 
    *          (axis/angle).
    * @param qhat a Vector containing the corresponding part joints 
    *          [degrees].
    * @return true/false if successful/failed.
    */
    virtual bool getSimulation(yarp::sig::Vector &xhat, yarp::sig::Vector &ohat,
                               yarp::sig::Vector &qhat) = 0;

    /**
    * Retrieve the interface currently active that controls the 
    * limb. 
    * @param activeIF a string containing "right" or "left" depending on the
    *           active interface.
    * @return true/false if successful/failed.
    */
    virtual bool getActiveIF(std::string &activeIF) = 0;

    /**
    * Set the active interface for limb control.
    * @param activeIF a string containing "right" or "left" depending on the
    *           interface to enable.
    * @return true/false if successful/failed.
    */
    virtual bool setActiveIF(const std::string &activeIF) = 0;

    /**
    * Retrieve the complete simulated trajectory as evolved from the
    * current state point. 
    * @param trajPos a list containing the whole trajectory of 
    *            points (position).
    * @param trajOrien a list containing the whole trajectory of 
    *            points (orientation).
    * @param maxIterations maximum number of iterations performed to reach 
    *            the target.
    * @param Ts integration period [s]. If Ts<=0.0 then the 
    *           configuration option "period" is used.
    * @return true/false if successful/failed.
    */
    virtual bool getTrajectory(std::deque<yarp::sig::Vector> &trajPos,
                               std::deque<yarp::sig::Vector> &trajOrien,
                               const unsigned int maxIterations=D4C_DEFAULT_MAXITERATIONS,
                               const double Ts=D4C_DEFAULT_TS_DISABLED) = 0;

    /**
    * Execute the simulated trajectory provided by the user.
    * @param trajPos a list containing the whole trajectory of 
    *            points (position).
    * @param trajOrien a list containing the whole trajectory of 
    *            points (orientation).
    * @param trajTime the trajectory duration [s]. 
    * @return true/false if successful/failed.
    */
    virtual bool executeTrajectory(const std::deque<yarp::sig::Vector> &trajPos,
                                   const std::deque<yarp::sig::Vector> &trajOrien,
                                   const double trajTime) = 0;

    /**
     * Destructor.
     */
    virtual ~D4C() { }
};

}

}

#endif


