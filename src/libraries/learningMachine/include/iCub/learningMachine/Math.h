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

#ifndef LM_MATH__
#define LM_MATH__

#include <gsl/gsl_linalg.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/RandScalar.h>
#include <yarp/math/RandnScalar.h>


namespace iCub {
namespace learningmachine {
namespace math {

/**
 * \ingroup icub_libLM_support
 *
 * Mathematical helper functions for use in the learningMachine library.
 *
 * \author Arjan Gijsberts
 *
 */

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*
 * Rank-1 update of a Cholesky factor
 *
 * Input:
 *   r: upper triangular cholesky factor (ldr x p, typically ldr == p)
 *   x: update vector
 *   z: set of nz column vectors to be updated (ldz x nz, typically ldz == p)
 *   y: nz dimensional vector
 *   rho: nz dimensional vector of norms of residuals
 * Output:
 *   r: updated cholesky factor
 *   z: updated column vectors
 *   rho: updated residual norm
 *   c: cosines of performed givens rotations
 *   s: sines of performed givens rotations
 *
 *
 * Note: Follows LINPACKs' dchud, but with the following differences
 * - accepts transposed Cholesky factor R (non-transposed is faster)
 * - uses CBLAS' drot and drotg for Given's rotations (as in Mattias Seeger's
 *   implementation)
 * - accepts transposed Z (non-transposed is faster)
 * - optional: enforces positive elements on the diagonal of R (as in Mattias
 *   Seeger's implementation, uncomment to enable)
 *
 * Please see the LINPACK User Guide for more detailed information on the
 * update routine and a less terse description of the parameters.
 */
void dchud(double* r, int ldr, int p, double* x, double* z, int ldz, int nz,
           double* y, double* rho, double* c, double* s,
           unsigned char rtrans = 0, unsigned char ztrans = 0);

/*
 * GSL type wrapper function for C implementation of dchud.
 */
void gsl_linalg_cholesky_update(gsl_matrix* R, gsl_vector* x, gsl_vector* c, gsl_vector* s,
                                gsl_matrix* Z = NULL, gsl_vector* y = NULL, gsl_vector* rho = NULL,
                                unsigned char rtrans = 0, unsigned char ztrans = 0);
#endif /* DOXYGEN_SHOULD_SKIP_THIS */

/**
 * Perform a rank-1 update to a Cholesky factor, while updating additional
 * vectors using the used Given's rotations and updating the norm of residuals.
 *
 * For more information, please see chapter 10.2 of the LINPACK User's Guide.
 *
 * @param R  an upper triangular Cholesky factor
 * @param x  the vector used to update the Cholesky factor
 * @param c  on output, the cosines of the Given's rotations
 * @param s  on output, the sines of the Given's rotations
 * @param Z  a number of column vectors updated along with R
 * @param y  a vector containing scalars used to update Z
 * @param rho  the norm of the residuals
 * @param rtrans  flag indicating whether R is provided transposed
 * @param ztrans  flag indicating whether Z is provided transposed
 */
void cholupdate(yarp::sig::Matrix& R, const yarp::sig::Vector& x, yarp::sig::Vector& c, yarp::sig::Vector& s,
                yarp::sig::Matrix& Z, const yarp::sig::Vector& y, yarp::sig::Vector& rho, bool rtrans = 0, bool ztrans = 0);

/**
 * Perform a rank-1 update to a Cholesky factor.
 *
 * For more information, please see chapter 10.2 of the LINPACK User's Guide.
 *
 * @param R  an upper triangular Cholesky factor
 * @param x  the vector used to update the Cholesky factor
 * @param rtrans  flag indicating whether R is provided transposed
 */
void cholupdate(yarp::sig::Matrix& R, const yarp::sig::Vector& x, bool rtrans = 0);

/**
 * Solves a system A*x=b for multiple row vectors in B using a precomputed
 * Cholesky factor R.
 *
 * @param R  the Cholesky factor
 * @param B  a matrix containing any number of row vectors b
 * @param X  a matrix containing the same number of solutions x on its rows
 */
void cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Matrix& B, yarp::sig::Matrix& X);

/**
 * Solves a system A*x=b for multiple row vectors in B using a precomputed
 * Cholesky factor R.
 *
 * @param R  the Cholesky factor
 * @param B  a matrix containing any number of column vectors b
 * @return  a matrix containing the same number of solutions x on its rows
 */
yarp::sig::Matrix cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Matrix& B);

/**
 * Solves a system A*x=b for using a precomputed Cholesky factor R.
 *
 * @param R  the Cholesky factor
 * @param b  the vector b
 * @param x  the solution x
 */
void cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Vector& b, yarp::sig::Vector& x);

/**
 * Solves a system A*x=b for using a precomputed Cholesky factor R.
 *
 * @param R  the Cholesky factor
 * @param b  the vector b
 * @return  the solution x
 */
yarp::sig::Vector cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Vector& b);

/**
 * Computes the outer product of two vectors.
 *
 * @param v1  the first vector
 * @param v2  the second vector
 * @return  the outer product
 */
yarp::sig::Matrix outerprod(const yarp::sig::Vector& v1, const yarp::sig::Vector& v2);

/**
 * Adds a scalar to a vector inplace.
 *
 * @param v  the vector
 * @param val  the scalar
 * @return  the vector
 */
yarp::sig::Vector& addvec(yarp::sig::Vector& v, double val);

/**
 * Solves a triangular linear system Ax=b where A is triangular.
 *
 * @param A  the matrix A
 * @param b  the vector b
 * @param x  the vector x
 * @param transa whether A should be transposed
 */
void trsolve(const yarp::sig::Matrix& A, const yarp::sig::Vector& b, yarp::sig::Vector& x, bool transa = false);


/**
 * Solves a linear system Ax=b where A is triangular.
 *
 * @param A  the matrix A
 * @param b  the matrix b
 * @param transa whether A should be transposed
 * @return x  the vector x
 */
yarp::sig::Vector trsolve(const yarp::sig::Matrix& A, const yarp::sig::Vector& b, bool transa = false);

/**
 * Fills an entire vector using the provided pseudo random number generator.
 *
 * @param v  a reference to the vector
 * @param prng  a reference to the pseudo random number generator
 */
void fillrandom(yarp::sig::Vector& v, yarp::math::RandScalar& prng);

/**
 * Fills an entire matrix using the provided pseudo random number generator.
 *
 * @param M  a reference to the matrix
 * @param prng  a reference to the pseudo random number generator
 */
void fillrandom(yarp::sig::Matrix& M, yarp::math::RandScalar& prng);

/**
 * Fills an entire vector using the provided pseudo random number generator.
 *
 * @param v  a reference to the vector
 * @param prng  a reference to the pseudo random number generator
 */
void fillrandom(yarp::sig::Vector& v, yarp::math::RandnScalar& prng);

/**
 * Fills an entire matrix using the provided pseudo random number generator.
 *
 * @param M  a reference to the matrix
 * @param prng  a reference to the pseudo random number generator
 */
void fillrandom(yarp::sig::Matrix& M, yarp::math::RandnScalar& prng);

/**
 * Returns a random vector with given dimensionality.
 *
 * @param length  the desired dimensionality of the vector
 * @param prng  a reference to the pseudo random number generator
 * @return the random vector
 */
yarp::sig::Vector random(int length, yarp::math::RandScalar& prng);

/**
 * Returns a random matrix with given dimensionality.
 *
 * @param rows  the desired number of rows for the matrix
 * @param cols  the desired number of columns for the matrix
 * @param prng  a reference to the pseudo random number generator
 * @return the random matrix
 */
yarp::sig::Matrix random(int rows, int columns, yarp::math::RandScalar& prng);

/**
 * Returns a random vector with given dimensionality.
 *
 * @param length  the desired dimensionality of the vector
 * @param prng  a reference to the pseudo random number generator
 * @return the random vector
 */
// why do RandScalar and RandnScalar not share a common base class?
yarp::sig::Vector random(int length, yarp::math::RandnScalar& prng);

/**
 * Returns a random matrix with given dimensionality.
 *
 * @param rows  the desired number of rows for the matrix
 * @param cols  the desired number of columns for the matrix
 * @param prng  a reference to the pseudo random number generator
 * @return the random matrix
 */
yarp::sig::Matrix random(int rows, int columns, yarp::math::RandnScalar& prng);

/**
 * Performs a unary operator inplace on each element of a vector.
 *
 * @param v  a reference to the vector
 * @param op  the operator
 * @return  a reference to the vector
 */
yarp::sig::Vector& map(yarp::sig::Vector& v, double (op)(double));

/**
 * Performs a unary operator inplace on each element of a matrix.
 *
 * @param M  a reference to the matrix
 * @param op  the operator
 * @return  a reference to the matrix
 */
yarp::sig::Matrix& map(yarp::sig::Matrix& M, double (op)(double));

/**
 * Performs a unary operator on each element of a vector.
 *
 * @param v  a constant reference to the vector
 * @param op  the operator
 * @return  the vector
 */
yarp::sig::Vector map(const yarp::sig::Vector& v, double (op)(double));

/**
 * Performs a unary operator on each element of a matrix.
 *
 * @param M  a constant reference to the matrix
 * @param op  the operator
 * @return  the matrix
 */
yarp::sig::Matrix map(const yarp::sig::Matrix& M, double (op)(double));

/**
 * Computes the cosine of a matrix element-wise inplace. Renamed to avoid possible
 * ambiguity with the standard cos function.
 *
 * @param M  a reference to the matrix
 * @return  the matrix of cosines
 */
yarp::sig::Matrix& cosmat(yarp::sig::Matrix& M);

/**
 * Computes the sine of a matrix element-wise inplace. Renamed to avoid possible
 * ambiguity with the standard sin function.
 *
 * @param M  a reference to the matrix
 * @return  the matrix of sines
 */
yarp::sig::Matrix& sinmat(yarp::sig::Matrix& M);

/**
 * Computes the cosine of a vector element-wise inplace. Renamed to avoid possible
 * ambiguity with the standard cos function.
 *
 * @param v  a reference to the vector
 * @return  the vector of cosines
 */
yarp::sig::Vector& cosvec(yarp::sig::Vector& v);

/**
 * Computes the sine of a vector element-wise inplace. Renamed to avoid possible
 * ambiguity with the standard sin function.
 *
 * @param v  a reference to the vector
 * @return  the vector of sines
 */
yarp::sig::Vector& sinvec(yarp::sig::Vector& v);

/**
 * Computes the cosine of a matrix element-wise. Renamed to avoid possible
 * ambiguity with the standard cos function.
 *
 * @param M  a constant reference to the matrix
 * @return  the matrix of cosines
 */
yarp::sig::Matrix cosmat(const yarp::sig::Matrix& M);

/**
 * Computes the sine of a matrix element-wise. Renamed to avoid possible
 * ambiguity with the standard sin function.
 *
 * @param M  a constant reference to the matrix
 * @return  the matrix of sines
 */
yarp::sig::Matrix sinmat(const yarp::sig::Matrix& M);

/**
 * Computes the cosine of a vector element-wise. Renamed to avoid possible
 * ambiguity with the standard cos function.
 *
 * @param v  a constant reference to the vector
 * @return  the vector of cosines
 */
yarp::sig::Vector cosvec(const yarp::sig::Vector& v);

/**
 * Computes the sine of a vector element-wise. Renamed to avoid possible
 * ambiguity with the standard sin function.
 *
 * @param v  a constant reference to the vector
 * @return  the vector of sines
 */
yarp::sig::Vector sinvec(const yarp::sig::Vector& v);

} // math
} // learningmachine
} // iCub

#endif
