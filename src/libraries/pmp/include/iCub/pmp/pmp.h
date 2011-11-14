/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * \defgroup pmp pmp
 *  
 * @ingroup icub_libraries 
 *  
 * Abstract class for dealing with the Passive Motion Paradigm 
 * (PMP) control. 
 *  
 * For further details please refer to the <a 
 * href="http://eris.liralab.it/wiki/Pmp_Library">wiki</a>. 
 *  
 * \author Ilaria Gori, Ugo Pattacini 
 *  
 */ 

#ifndef __PMP_H__
#define __PMP_H__

#include <deque>
#include <yarp/os/Property.h>
#include <yarp/sig/Vector.h>

#define PMP_MAX_ITERATIONS      1000
#define PMP_TS                  0.0

namespace iCub
{

namespace pmp
{

/**
* @ingroup pmp
*
* The Definition of the Pmp Interface.
*/
class Pmp
{
public:
    /**
    * Configure the client or the server part of Pmp framework.
    * @param options contains the set of options in form of a 
    *                Property object.
    *  
    * Available options for the client are: 
    *  
    * \b remote <string>: example (remote /pmp_server), specifies 
    *    the server port stem-name to connect to.
    *  
    * \b local <string>: example (local /pmp_client), specifies the 
    *    the client stem-name to be used for opening ports.
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
    * \b name <string>: example (name pmp_server), specifies the 
    *    name of the server to be launched.
    *  
    * \b robot <string>: example (robot icub), specifies the name of
    *    the robot to connect to.
    *  
    * \b part <string>: example (part right_arm), specifies the 
    *    robot part to be controlled.
    *  
    * \b period <int>: example (period 20), specifies the period 
    *    given in [ms] to generate the pmp field.
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
    * Request for the creation of a new Pmp entity which can be
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
    * mass-spring-damper system on the end-effector within the Pmp 
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
    * a multivariate gaussian repulsive field within the Pmp 
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
    virtual bool getItems(yarp::os::Bottle &items) const = 0;

    /**
    * Allow to change the properties of an existing item.
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
    virtual bool getProperty(const int item, yarp::os::Property &options) const = 0;

    /**
    * Enable the generation of the Pmp field.
    * @return true/false if successful/failed.
    */
    virtual bool enableField() = 0;

    /**
    * Disable the generation of the Pmp field.
    * @return true/false if successful/failed.
    */
    virtual bool disableField() = 0;

    /**
    * Retrieve the current status of the Pmp field. 
    * @param status true/false on enabled/disabled Pmp field. 
    * @return true/false if successful/failed.
    */
    virtual bool getFieldStatus(bool &status) const = 0;

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
    virtual bool getControlStatus(bool &status) const = 0;

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
    virtual bool getSimulationStatus(bool &status) const = 0;

    /**
    * Allow to change the thread period for Pmp field generation. 
    * @param period the new thread period in [ms].
    * @return true/false if successful/failed.
    */
    virtual bool setPeriod(const int period) = 0;

    /**
    * Retrieve the current thread period for Pmp field generation.
    * @param period the current thread period in [ms].
    * @return true/false if successful/failed.
    */
    virtual bool getPeriod(int &period) const = 0;

    /**
    * Allow to set the trajectory state equal to the currently 
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
    virtual bool getToolFrame(yarp::sig::Vector &x, yarp::sig::Vector &o) const = 0;

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
    virtual bool getTool(yarp::sig::Vector &x, yarp::sig::Vector &o) const = 0;

    /**
    * Allow to change the trajectory state generated by the Pmp 
    * framework. 
    * @param x a Vector containing the new position [m]. 
    * @param o a Vector containing the new orientation (axis/angle).
    * @param xdot a Vector containing the new translational velocity
    *             [m/s].
    * @param odot a Vector containing the new rotational velocity 
    *             (axis/angle).
    * @return true/false if successful/failed.
    */
    virtual bool setPointState(const yarp::sig::Vector &x, const yarp::sig::Vector &o,
                               const yarp::sig::Vector &xdot, const yarp::sig::Vector &odot) = 0;

    /**
    * Retrieve the current trajectory state generated by the Pmp 
    * framework. 
    * @param x a Vector containing the current position [m]. 
    * @param o a Vector containing the current orientation 
    *          (axis/angle).
    * @param xdot a Vector containing the current translational 
    *             velocity [m/s].
    * @param odot a Vector containing the current rotational 
    *             velocity (axis/angle).
    * @return true/false if successful/failed.
    */
    virtual bool getPointState(yarp::sig::Vector &x, yarp::sig::Vector &o,
                               yarp::sig::Vector &xdot, yarp::sig::Vector &odot) const = 0;

    /**
    * Retrieve the current field as generated by the Pmp framework.
    * @param field a Vector containing the current field. 
    * @return true/false if successful/failed.
    */
    virtual bool getField(yarp::sig::Vector &field) const = 0;

    /**
    * Retrieve the trajectory the end-effector will follow as 
    * constrained by the Pmp field.
    * @param xhat a Vector containing the simulated position [m]. 
    * @param ohat a Vector containing the simulated orientation 
    *          (axis/angle).
    * @param qhat a Vector containing the corresponding part joints 
    *          [degrees].
    * @return true/false if successful/failed.
    */
    virtual bool getSimulation(yarp::sig::Vector &xhat, yarp::sig::Vector &ohat,
                               yarp::sig::Vector &qhat) const = 0;

    /**
    * Retrieve the active interface.
    * @param activeIF a string containing "right" or "left" depending on the
    *           active interface.
    * @return true/false if successful/failed.
    */
    virtual bool getActiveIF(std::string &activeIF) const = 0;

    /**
    * Set the active interface.
    * @param activeIF a string containing "right" or "left" depending on the
    *           interface to enable.
    * @return true/false if successful/failed.
    */
    virtual bool setActiveIF(const std::string &activeIF) = 0;

    /**
    * Retrieve the complete simulated trajectory from an initial 
    * point. 
    * @param trajPos a deque contained the whole trajectory of points 
    *            (position).
    * @param trajOrien a deque contained the whole trajectory of points 
    *            (orientation).
    * @param maxIterations maximum number of iterations performed to reach 
    *            the target.
    * @param Ts integration period [s]. If Ts<=0.0 then the 
    *           configuration option "period" is used.
    * @return true/false if successful/failed.
    */
    virtual bool getTrajectory(std::deque<yarp::sig::Vector> &trajPos,
                               std::deque<yarp::sig::Vector> &trajOrien,
                               const unsigned int maxIterations=PMP_MAX_ITERATIONS,
                               const double Ts=PMP_TS) = 0;

    /**
     * Destructor.
     */
    virtual ~Pmp() { }
};

}

}

#endif


