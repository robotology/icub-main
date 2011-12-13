/*
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * author:  Arjan Gijsberts
 * email:   arjan.gijsberts@iit.it
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
 */

#ifndef LM_SERIALIZATION__
#define LM_SERIALIZATION__

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>

namespace iCub {
namespace learningmachine {
namespace serialization {

/**
 * \ingroup icub_libLM_support
 *
 * Helper functions for serialization in bottles for use in the learningMachine
 * library.
 *
 * \author Arjan Gijsberts
 *
 */

/**
 * Pushes a serialization of a vector to the end of a Bottle.
 *
 * @param v  a reference to the vector
 * @param bot  the bottle
 */
//void push(const yarp::sig::Vector& v, yarp::os::Bottle& bot);

/**
 * Pushes a serialization of a matrix to the end of a Bottle.
 *
 * @param M  a reference to the matrix
 * @param bot  the bottle
 */
//void push(const yarp::sig::Matrix& M, yarp::os::Bottle& bot);

/**
 * Pops a deserialization of a vector from the end of a Bottle.
 *
 * @param v  a reference to the vector
 * @param bot  the bottle
 */
//void pop(const yarp::sig::Vector& v, yarp::os::Bottle& bot);

/**
 * Pops a deserialization of a matrix from the end of a Bottle.
 *
 * @param M  a reference to the matrix
 * @param bot  the bottle
 */
//void pop(const yarp::sig::Matrix& M, yarp::os::Bottle& bot);

/**
 * Pushes an integer to the end of a Bottle using the insertion operator.
 *
 * @param bot  a reference to the bottle
 * @param val  the integer
 * @return a reference to the bottle
 */
yarp::os::Bottle& operator<<(yarp::os::Bottle &out, int val);

/**
 * Pushes a double to the end of a Bottle using the insertion operator.
 *
 * @param bot  a reference to the bottle
 * @param val  the double
 * @return a reference to the bottle
 */
yarp::os::Bottle& operator<<(yarp::os::Bottle &out, double val);

/**
 * Pushes a serialization of a vector to the end of a Bottle using the insertion
 * operator.
 *
 * @param bot  a reference to the bottle
 * @param v  a reference to the vector
 * @return a reference to the bottle
 */
yarp::os::Bottle& operator<<(yarp::os::Bottle &out, const yarp::sig::Vector& v);

/**
 * Pushes a serialization of a matrix to the end of a Bottle using the insertion
 * operator.
 *
 * @param bot  a reference to the bottle
 * @param M  a reference to the matrix
 * @return a reference to the bottle
 */
yarp::os::Bottle& operator<<(yarp::os::Bottle &out, const yarp::sig::Matrix& M);

/**
 * Pops an integer from the end of a Bottle using the extraction operator.
 *
 * @param bot  a reference to the bottle
 * @param val  a reference to the integer
 * @return  a reference to the bottle
 */
yarp::os::Bottle& operator>>(yarp::os::Bottle &in, int& val);

/**
 * Pops a double from the end of a Bottle using the extraction operator.
 *
 * @param bot  a reference to the bottle
 * @param val  a reference to the double
 * @return  a reference to the bottle
 */
yarp::os::Bottle& operator>>(yarp::os::Bottle &in, double& val);

/**
 * Pops a deserialization of a vector from the end of a Bottle using the
 * extraction operator.
 *
 * @param bot  a reference to the bottle
 * @param v  a reference to the vector
 * @return  a reference to the bottle
 */
yarp::os::Bottle& operator>>(yarp::os::Bottle &in, yarp::sig::Vector& v);

/**
 * Pops a deserialization of a matrix from the end of a Bottle using the
 * extraction operator.
 *
 * @param bot  a reference to the bottle
 * @param M  a reference to the matrix
 * @return  a reference to the bottle
 */
yarp::os::Bottle& operator>>(yarp::os::Bottle &in, yarp::sig::Matrix& M);

} // serialization
} // learningmachine
} // iCub

#endif
