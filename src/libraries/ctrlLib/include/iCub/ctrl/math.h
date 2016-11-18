/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/**
 * \defgroup ctrlLib ctrlLib
 *  
 * @ingroup icub_libraries 
 *  
 * Classes for control engineering: filtering, fitting, Kalman
 * estimation, PIDs and more topics are expected to be covered 
 * :) 
 *  
 * \author Ugo Pattacini, Serena Ivaldi, Francesco Nori
 *  
 * \defgroup Maths Maths 
 *  
 * @ingroup ctrlLib
 *
 * Collection of mathematical functions.
 *
 * \author Ugo Pattacini, Serena Ivaldi, Francesco Nori
 * 
 * Copyright (C) 2010 RobotCub Consortium
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0. 
 *  
 */ 


#ifndef __CTRLMATH_H__
#define __CTRLMATH_H__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>


namespace iCub
{

namespace ctrl
{

/**
 * The PI constant.
 */
extern const double CTRL_PI;

/**
 * 180/PI.
 */
extern const double CTRL_RAD2DEG;

/**
 * PI/180.
 */
extern const double CTRL_DEG2RAD;

/**
* \ingroup Maths
*
* Returns the dot product between two vectors given in the form: 
* matrix(:,col). 
* @param A is the first input vector given as a=A(:,colA). 
* @param colA is the column for the first vector. 
* @param B is the second input vector given as b=B(:,colB). 
* @param colB is the column for the second vector. 
* @return <a,b>.
*/
double dot(const yarp::sig::Matrix &A, int colA,
           const yarp::sig::Matrix &B, int colB);

/**
* \ingroup Maths
*
* Returns the squared norm of the vector given in the form: 
* matrix(:,col). 
* @param M is the input vector given as m=M(:,col). 
* @param col is the column for the vector. 
* @return ||v||^2.
*/
inline double norm2(const yarp::sig::Matrix &M, int col)
{
    return dot(M,col,M,col);
}

/**
* \ingroup Maths
*
* Returns the norm of the vector given in the form: 
* matrix(:,col). 
* @param M is the input vector given as m=M(:,col). 
* @param col is the column for the vector. 
* @return ||v||.
*/
double norm(const yarp::sig::Matrix &M, int col);

/**
* \ingroup Maths
*
* Returns the cross product between two vectors given in the 
* form: matrix(:,col). 
* @param A is the first input vector given as a=A(:,colA). 
* @param colA is the column for the first vector. 
* @param B is the second input vector given as b=B(:,colB). 
* @param colB is the column for the second vector. 
* @return axb.
*/
yarp::sig::Vector cross(const yarp::sig::Matrix &A, int colA,
                        const yarp::sig::Matrix &B, int colB);

/**
* \ingroup Maths
*
* Returns the derivatice of cross product between two vectors. 
* @param a is the first input vector. 
* @param Da is the derivative of first input vector.  
* @param b is the second input vector. 
* @param Db is the derivative of second input vector.  
* @return D(axb).
*/
yarp::sig::Vector Dcross(const yarp::sig::Vector &a, const yarp::sig::Vector &Da,
                         const yarp::sig::Vector &b, const yarp::sig::Vector &Db);

/**
* \ingroup Maths
*
* Returns the derivative of cross product between two vectors 
* given in the form: matrix(:,col). 
* @param A is the first input vector given as a=A(:,colA). 
* @param DA is the derivative of first input vector.   
* @param colA is the column for the first vector. 
* @param B is the second input vector given as b=B(:,colB). 
* @param DB is the derivative of second input vector.   
* @param colB is the column for the second vector. 
* @return D(axb).
*/
yarp::sig::Vector Dcross(const yarp::sig::Matrix &A, const yarp::sig::Matrix &DA, int colA,
                         const yarp::sig::Matrix &B, const yarp::sig::Matrix &DB, int colB);

}
 
}
 
#endif



