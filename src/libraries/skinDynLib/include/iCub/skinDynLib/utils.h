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
 * Misc function used throughout the library
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <yarp/math/Math.h>

#include "iCub/skinDynLib/skinContact.h"

#include <sstream>
#include <list>

namespace iCub
{

namespace skinDynLib
{

/**
 * Converts a matrix to a vector with rowsxcolumns elements.
 * It loses the matrix dimensions, so please use with care.
 * @param  m is the matrix to convert
 * @return   the converted vector
 */
yarp::sig::Vector toVector(yarp::sig::Matrix m);

/**
 * Closes properly a given port
 * @param _port is the port to close
**/
void closePort(yarp::os::Contactable *_port);

/**
 * Retrieves a matrix from a bottle:
 * @param b  is the bottle
 * @param in is the index from which start acquiring values
 * @param r  is the number of rows of the matrix
 * @param c  is the number of cols of the matrix
**/
yarp::sig::Matrix matrixFromBottle(const yarp::os::Bottle b,
                                   int in, const int r, const int c);

/**
 * Puts a matrix into a bottle, by cycling through its elements
 * and adding them as double into the bottle. It loses the matrix dimensions,
 * so please use with care.
 * @param m is the matrix to be converted
 * @param b is the bottle the matrix has been converted into
**/
void      matrixIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b);

/**
 * Puts a matrix into a bottle, by cycling through its elements
 * and adding them as int into the bottle. It loses the matrix dimensions,
 * so please use with care.
 * @param m is the matrix to be converted
 * @param b is the bottle the matrix has been converted into
**/
void matrixOfIntIntoBottle(const yarp::sig::Matrix m, yarp::os::Bottle &b);

/**
 * Puts a vector into a bottle, by cycling through its elements
 * and adding them as double.
 * @param v is the vector to be converted
 * @param b is the bottle the vector has been converted into
**/
void vectorIntoBottle(const yarp::sig::Vector v, yarp::os::Bottle &b);

/**
* Converts an int to a string
* @param a is the int to be converted
* @return  the string the int has been converted into
**/
std::string int_to_string( const int a );

/**
* Computes the factorial using a recursive method
**/
unsigned int factorial(unsigned int n);

}

}//end namespace

#endif

// empty line to make gcc happy
