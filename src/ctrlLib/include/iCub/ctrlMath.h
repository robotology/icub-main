/**
 * \defgroup ctrlLib ctrlLib
 *  
 * @ingroup icub_module 
 *  
 * Classes for control engineering: filtering, fitting, Kalman
 * estimation, PIDs and more topics are expected to be covered 
 * :) 
 *  
 * \author Ugo Pattacini 
 *  
 * \defgroup Maths Maths 
 *  
 * @ingroup ctrlLib
 *
 * Collection of mathematical functions
 *
 * \author Ugo Pattacini
 *
 */ 


#ifndef __CTRLMATH_H__
#define __CTRLMATH_H__

#include <gsl/gsl_math.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>


namespace ctrl
{

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
* Returns the Euclidean squared norm of the vector. 
* @param v is the input vector. 
* @return ||v||^2. 
*/
inline double norm2(const yarp::sig::Vector &v)
{
    return yarp::math::dot(v,v);
}


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
* Returns the Euclidean norm of the vector. 
* @param v is the input vector. 
* @return ||v||. 
*/
inline double norm(const yarp::sig::Vector &v)
{
    return sqrt(norm2(v));
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
inline double norm(const yarp::sig::Matrix &M, int col)
{
    return sqrt(norm2(M,col));
}


/**
* \ingroup Maths
*
* Returns the the cross product between two vectors. 
* @param a is the first input vector. 
* @param b is the second input vector. 
* @param verbose sets some verbosity. 
* @return axb.
*/
yarp::sig::Vector cross(const yarp::sig::Vector &a, const yarp::sig::Vector &b,
                        unsigned int verbose=0);


/**
* \ingroup Maths
*
* Returns the cross product between two vectors given in the 
* form: matrix(:,col). 
* @param A is the first input vector given as a=A(:,colA). 
* @param colA is the column for the first vector. 
* @param B is the second input vector given as b=B(:,colB). 
* @param colB is the column for the second vector. 
* @param verbose sets some verbosity.  
* @return axb.
*/
yarp::sig::Vector cross(const yarp::sig::Matrix &A, int colA,
                        const yarp::sig::Matrix &B, int colB,
                        unsigned int verbose=0);


/**
* \ingroup Maths
*
* Returns the derivatice of cross product between two vectors. 
* @param a is the first input vector. 
* @param Da is the derivative of first input vector.  
* @param b is the second input vector. 
* @param Db is the derivative of second input vector.  
* @param verbose sets some verbosity. 
* @return D(axb).
*/
yarp::sig::Vector Dcross(const yarp::sig::Vector &a, const yarp::sig::Vector &Da,
                         const yarp::sig::Vector &b, const yarp::sig::Vector &Db,
                         unsigned int verbose=0);


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
* @param verbose sets some verbosity.  
* @return D(axb).
*/
yarp::sig::Vector Dcross(const yarp::sig::Matrix &A, const yarp::sig::Matrix &DA, int colA,
                         const yarp::sig::Matrix &B, const yarp::sig::Matrix &DB, int colB,
                         unsigned int verbose=0);


/**
* \ingroup Maths
*
* Converts a rotation matrix R to axis/angle representation.
* @param R is the input matrix.
* @param verbose sets some verbosity.  
* @return 4 by 1 vector for the axis/angle representation.
*/
yarp::sig::Vector dcm2axis(const yarp::sig::Matrix &R, unsigned int verbose=0);


/**
* \ingroup Maths
*
* Returns a rotation matrix R from axis/angle representation
* @param v is the axis/angle vector.
* @param verbose sets some verbosity.  
* @return 4 by 4 rotation matrix of the form [R,1].
*/
yarp::sig::Matrix axis2dcm(const yarp::sig::Vector &v, unsigned int verbose=0);


/**
* \ingroup Maths
*
* Returns the inverse of a 4 by 4 rototranslational matrix 
* @param H is the 4 by 4 rototranslational matrix.
* @param verbose sets some verbosity.  
* @return inverse of 4 by 4 rototranslational matrix. 
*  
* \note about 5 times faster than pinv() 
*/
yarp::sig::Matrix SE3inv(const yarp::sig::Matrix &H, unsigned int verbose=0);

}
 
#endif



