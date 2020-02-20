/*
 * Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <yarp/os/Property.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/sig/Vector.h>


struct PointingFar
{
public:
    /*!
     * \author Ugo Pattacini (15/01/2017)
     *
     * \param iarm the Cartesian Interface.
     * \param requirements a Property specifying: <br>"point":(x y z) [m]<br>"finger-joints":(...) [deg].
     * \param q Vector containing the joints: torso(3) ... arm(7) [deg].
     * \param x Vector containing the pose: x y z ax ay az [m]/[rad].
     *
     * \return true/false on success/fail.
     */
    static bool compute(yarp::dev::ICartesianControl *iarm,
                        const yarp::os::Property& requirements,
                        yarp::sig::Vector& q, yarp::sig::Vector& x);

    /*!
     * \author Ugo Pattacini (18/01/2017)
     *
     * \param iarm the Cartesian Interface.
     * \param q Vector containing the joints.
     * \param x Vector containing the pose.
     *
     * \return true/false on success/fail.
     */
    static bool point(yarp::dev::ICartesianControl *iarm,
                      const yarp::sig::Vector& q, const yarp::sig::Vector& x);
};

