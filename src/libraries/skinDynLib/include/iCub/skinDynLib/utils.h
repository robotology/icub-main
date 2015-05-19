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

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::skinDynLib;

using namespace std;

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
 * Retrieves a vector from a bottle:
 * @param b    is the bottle
 * @param in   is the index from which start acquiring values
 * @param size is the size of the vector
**/
yarp::sig::Vector vectorFromBottle(const Bottle b, int in, const int size);

/**
 * Puts a matrix into a bottle, by cycling through its elements
 * and adding them as double into the bottle. It loses the matrix dimensions,
 * so please use with care.
 * @param m is the matrix to be converted
 * @param b is the bottle the matrix has been converted into
**/
void      matrixIntoBottle(const yarp::sig::Matrix m, Bottle &b);

/**
 * Puts a matrix into a bottle, by cycling through its elements
 * and adding them as int into the bottle. It loses the matrix dimensions,
 * so please use with care.
 * @param m is the matrix to be converted
 * @param b is the bottle the matrix has been converted into
**/
void matrixOfIntIntoBottle(const yarp::sig::Matrix m, Bottle &b);

/**
 * Puts a vector into a bottle, by cycling through its elements
 * and adding them as double.
 * @param v is the vector to be converted
 * @param b is the bottle the vector has been converted into
**/
void vectorIntoBottle(const yarp::sig::Vector v, Bottle &b);

/**
* Converts an int to a string
* @param a is the int to be converted
* @return  the string the int has been converted into
**/
string int_to_string( const int a );

/**
* Computes the factorial using a recursive method
**/
unsigned int factorial(unsigned int n);

/**
* Struct that encloses all the information related to a taxel.
**/
struct IncomingEvent
{
    yarp::sig::Vector Pos;
    yarp::sig::Vector Vel;
    double Radius;          // average radius of the object
    string Src;             // the source of information the event is coming from

    double NRM;
    double TTC;

    /**
    * Constructors
    **/    
    IncomingEvent();
    IncomingEvent(const Vector &p, const Vector &v, const double r, const string &s);
    IncomingEvent(const Bottle &b);
    IncomingEvent(const IncomingEvent &e);

    /**
    * 
    **/    
    Bottle toBottle();

    /**
    * 
    **/
    bool fromBottle(const Bottle &b);

    /**
    * Copy Operator
    **/
    IncomingEvent &operator=(const IncomingEvent &e);

    /**
    * Print Method
    **/
    void print();

    /**
    * toString Method
    **/
    string toString() const;
};

/**
* It has only a couple more stuff
**/
struct IncomingEvent4TaxelPWE : public IncomingEvent
{
    double NRM;
    double TTC;

    /**
    * Constructors
    **/    
    IncomingEvent4TaxelPWE();
    IncomingEvent4TaxelPWE(const Vector &p, const Vector &v, const double r, const string &s);
    IncomingEvent4TaxelPWE(const IncomingEvent &e);
    IncomingEvent4TaxelPWE(const IncomingEvent4TaxelPWE &e);

    /**
    * Copy Operators
    **/
    IncomingEvent4TaxelPWE &operator=(const IncomingEvent &e);
    IncomingEvent4TaxelPWE &operator=(const IncomingEvent4TaxelPWE &e);

    /**
    * Compute the NRM and TTC from Pos and Vel
    */
    void computeNRMTTC();

    /**
     * Return norm and TTC in a pwe-compliant way
    */
    std::vector<double> getNRMTTC();

    /**
    * Print Method
    **/
    void print();

    /**
    * toString Method
    **/
    string toString() const;
};

}

}//end namespace

#endif

// empty line to make gcc happy
