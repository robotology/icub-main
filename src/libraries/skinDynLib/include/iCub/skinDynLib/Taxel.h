/**
 * Copyright (C) 2015 iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@iit.it
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
 *
 *
 * This file contains the definition of a Taxel, i.e. the atomic element the iCub skin is composed of.
 * It is empowered by a Position and a Normal (both relative to the body part it belong to), and some more
 * useful members.
 *
 * \section intro_sec Description
 * 
 * \section tested_os_sec Tested OS
 * 
 * Windows, Linux
 *
 * \author Alessandro Roncone
 * 
 **/

#ifndef __TAXEL_H__
#define __TAXEL_H__

#include <yarp/os/Log.h>
#include <yarp/math/Math.h>

#include "iCub/skinDynLib/skinContact.h"

#include <sstream>

namespace iCub
{
namespace skinDynLib
{

/** 
* @ingroup skinDynLib 
*
* Class that encloses everything relate to a Taxel, i.e. the atomic element the iCub skin is composed of.
* It is empowered by a Position and a Normal (both relative to the body part it belong to), and some more
* useful members (such as its 2D coordinates into the image frame of one of the eyes, its Frame of Reference
* base on the Position and Normal members, its position into the World Reference Frame)
*/
class Taxel
{
  protected:
    int ID;                        // taxels' ID
    yarp::sig::Vector Position;    // taxel's position w.r.t. the limb
    yarp::sig::Vector Normal;      // taxel's normal   w.r.t. the limb
    yarp::sig::Vector WRFPosition; // taxel's position w.r.t. the root FoR
    yarp::sig::Vector px;          // (u,v) projection in the image plane
    yarp::sig::Matrix FoR;         // taxel's reference Frame (computed from Pos and Norm)

  protected:
    /**
    * init function
    **/
    void init();

    /**
    * Compute and set the taxel's reference frame
    * (from its position and its normal vector)
    **/
    void setFoR();

  public:
    /**
    * Default Constructor
    **/
    Taxel();

    /**
     * Constructor with position and normal vectors
     * @param _position is the position of the taxel
     * @param _normal   is the normal vector of the taxel
     */
    Taxel(const yarp::sig::Vector &_position, const yarp::sig::Vector &_normal);

    /**
     * Constructor with position, normal and ID
     * @param _position is the position of the taxel
     * @param _normal   is the normal vector of the taxel
     * @param _id       is the ID of the taxel
     */
    Taxel(const yarp::sig::Vector &_position, const yarp::sig::Vector &_normal, const int &_id);

    /**
    * Copy Constructor
    * @param _t is the Taxel to copy from
    **/
    Taxel(const Taxel &_t);

    /**
    * Copy Operator
    * @param _t is the Taxel to copy from
    **/
    Taxel &operator=(const Taxel &_t);

    /**
     * Gets the ID of the taxel
     * @return int with the ID of the taxel
     */
    int getID();

    /**
     * Gets the Position of the taxel in the limb's FoR
     * @return Vector with the Position of the taxel
     */
    yarp::sig::Vector getPosition(); 

    /**
     * Gets the Normal of the taxel in the limb's FoR
     * @return Vector with the Normal of the taxel
     */
    yarp::sig::Vector getNormal();

    /**
     * Gets the Position of the taxel in the root FoR
     * @return Vector with the Position of the taxel
     */
    yarp::sig::Vector getWRFPosition();

    /**
     * Gets the u,v position of the taxel in one of the eyes
     * @return Vector with the 2D position of the
     *                taxel in one of the eyes
     */
    yarp::sig::Vector getPx();

    /**
     * Gets the Frame of Reference of the taxel
     * @return Matrix with the Frame of Reference
     */
    yarp::sig::Matrix getFoR();

    /**
     * Sets the ID of the taxel
     * @return true/false in case of success/failure
     */
    bool setID(int _ID);

    /**
     * Sets the Position of the taxel in the limb's FoR
     * @return true/false in case of success/failure
     */
    bool setPosition(const yarp::sig::Vector &_Position); 

    /**
     * Sets the Normal of the taxel in the limb's FoR
     * @return true/false in case of success/failure
     */
    bool setNormal(const yarp::sig::Vector &_Normal);

    /**
     * Sets the Position of the taxel in the root FoR
     * @return true/false in case of success/failure
     */
    bool setWRFPosition(const yarp::sig::Vector &_WRFPosition);

    /**
     * Sets the u,v position of the taxel in one of the eyes
     * @return true/false in case of success/failure
     */
    bool setPx(const yarp::sig::Vector &_px);

    /**
    * Print Method
    * @param verbosity is the verbosity level
    **/
    virtual void print(int verbosity=0);

    /**
    * toString Method
    * @param verbosity is the verbosity level
    **/
    virtual std::string toString(int verbosity=0);

};

}

}//end namespace

#endif

// empty line to make gcc happy
