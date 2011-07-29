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

#include <cassert>
#include <stdexcept>
#include <cmath>

#include <gsl/gsl_blas.h>

#include "iCub/learningMachine/Math.h"

namespace iCub {
namespace learningmachine {
namespace math {

void dchud(double* r, int ldr, int p, double* x, double* z, int ldz, int nz,
           double* y, double* rho, double* c, double* s,
           unsigned char rtrans, unsigned char ztrans) {
    unsigned int i;
    double* work = (double*) 0x0;
    double* tbuff = (double*) 0x0;
    unsigned int stp;
    unsigned int stp2;
    double scale, workscale, rhoscale;

    // create working copy of x
    work = (double*) malloc(p * sizeof(double));
    cblas_dcopy(p, x, 1, work, 1);

    stp = (rtrans == 1) ? p : 1;

    // update r and fill c and s on the go
    for(i = 0, tbuff = r; (int)i < p; tbuff+=(p+1), i++) {
        // compute givens rotation
        cblas_drotg(tbuff, work+i, c+i, s+i);

        // force positive values on the diagonal (not strictly necessary)
        //if(*tbuff < 0) {
        //    *tbuff = -(*tbuff);
        //    c[i] = -c[i];
        //    s[i] = -s[i];
        //}

        // apply givens rotation
        if((int)i < p - 1) {
            cblas_drot(p-i-1, tbuff+stp, stp, work+i+1, 1, c[i], s[i]);
        }
    }
    free(work);

    // update z and rho if applicable
    if(nz > 0) {
        work = (double*) malloc(nz * sizeof(double));
        cblas_dcopy(nz, y, 1, work, 1);

        stp = (ztrans == 1) ? 1 : nz;
        stp2 = (ztrans == 1) ? ldz : 1;

        // update z
        for(i = 0; (int)i < p; i++) {
            cblas_drot(nz, z+i*stp, stp2, work, 1, c[i], s[i]);
        }

        // update rho
        for(i = 0; (int)i < nz; i++) {
            if(work[i] < 0) {
                work[i] = -work[i];
            }
            if(rho[i] > 0) {
                scale = work[i] + rho[i];
                workscale = work[i] / scale;
                rhoscale = rho[i] / scale;
                rho[i] = scale * sqrt((workscale)*(workscale) + (rhoscale)*(rhoscale));
                // above is equal to line below, but less sensitive to numerical instability
                //rho[i] = sqrt(work[i]*work[i] + rho[i]*rho[i]);
            }
        }
        free(work);
    }
}

void gsl_linalg_cholesky_update(gsl_matrix* R, gsl_vector* x, gsl_vector* c, gsl_vector* s,
                                gsl_matrix* Z, gsl_vector* y, gsl_vector* rho,
                                unsigned char rtrans, unsigned char ztrans) {
    int i, j;
    int ldr, p;
    int ldz = 0;
    int nz = 0;
    double* Zp = NULL;
    double* yp = NULL;
    double* rhop = NULL;

    ldr = rtrans ? R->size2 : R->size1;
    p = rtrans ? R->size1 : R->size2;

    // fill Z, y, and rho pointers and dimensions if not null
    if(Z != NULL && y != NULL && rho != NULL) {
        ldz = ztrans ? Z->size2 : Z->size1;
        nz = ztrans ? Z->size1 : Z->size2;
        Zp = Z->data;
        yp = y->data;
        rhop = rho->data;
    }

    dchud(R->data, ldr, p, x->data, Zp, ldz, nz, yp, rhop, c->data, s->data, rtrans, ztrans);

    // reflect, as GSL functions expects duplicate information (i.e., lower and upper triangles)
    for(i = 0; i < p; i++) {
        for(j = 0; j < i; j++) {
            if(rtrans) {
                gsl_matrix_set(R, j, i, gsl_matrix_get(R, i, j));
            } else {
                gsl_matrix_set(R, i, j, gsl_matrix_get(R, j, i));
            }
        }
    }
}

void cholupdate(yarp::sig::Matrix& R, const yarp::sig::Vector& x, yarp::sig::Vector& c, yarp::sig::Vector& s,
                yarp::sig::Matrix& Z, const yarp::sig::Vector& y, yarp::sig::Vector& rho, bool rtrans, bool ztrans) {
    gsl_matrix* Rgsl = (gsl_matrix*) R.getGslMatrix();
    gsl_vector* xgsl = (gsl_vector*) x.getGslVector();
    gsl_matrix* Zgsl = (gsl_matrix*) Z.getGslMatrix();
    gsl_vector* ygsl = (gsl_vector*) y.getGslVector();
    gsl_vector* rhogsl = (gsl_vector*) rho.getGslVector();
    gsl_vector* cgsl = (gsl_vector*) c.getGslVector();
    gsl_vector* sgsl = (gsl_vector*) s.getGslVector();

    gsl_linalg_cholesky_update(Rgsl, xgsl, cgsl, sgsl, Zgsl, ygsl, rhogsl, (unsigned char) rtrans, (unsigned char) ztrans);
}

void cholupdate(yarp::sig::Matrix& R, const yarp::sig::Vector& x, bool rtrans) {
    yarp::sig::Vector c(R.cols());
    yarp::sig::Vector s(R.cols());

    gsl_matrix* Rgsl = (gsl_matrix*) R.getGslMatrix();
    gsl_vector* xgsl = (gsl_vector*) x.getGslVector();
    gsl_vector* cgsl = (gsl_vector*) c.getGslVector();
    gsl_vector* sgsl = (gsl_vector*) s.getGslVector();

    gsl_linalg_cholesky_update(Rgsl, xgsl, cgsl, sgsl, NULL, NULL, NULL, (unsigned char) rtrans, 0);
}

void cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Matrix& B, yarp::sig::Matrix& X) {
    assert(B.rows() == X.rows());
    assert(B.cols() == X.cols());
    assert(R.rows() == R.cols());
    assert(R.cols() == B.cols());

    yarp::sig::Vector x;
    yarp::sig::Vector b;
    for(int r = 0; r < B.rows(); r++) {
        // we're forced to waste memory and computation here :(
        b = B.getRow(r);
        x = X.getRow(r);
        cholsolve(R, b, x);
        // copy back the solution
        X.setRow(r, x);
    }
}

yarp::sig::Matrix cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Matrix& B) {
    yarp::sig::Matrix X(B.rows(), B.cols());
    cholsolve(R, B, X);
    return X;
}

void cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Vector& b, yarp::sig::Vector& x) {
    int info;

    gsl_matrix* Rgsl = (gsl_matrix*) R.getGslMatrix();
    gsl_vector* bgsl = (gsl_vector*) b.getGslVector();
    gsl_vector* xgsl = (gsl_vector*) x.getGslVector();

    info = gsl_linalg_cholesky_solve(Rgsl, bgsl, xgsl);
    if(info) {
        throw std::runtime_error(gsl_strerror(info));
    }
}

yarp::sig::Vector cholsolve(const yarp::sig::Matrix& R, const yarp::sig::Vector& b) {
    yarp::sig::Vector x(b.size());
    cholsolve(R, b, x);
    return x;
}

yarp::sig::Matrix outerprod(const yarp::sig::Vector& v1, const yarp::sig::Vector& v2) {
    yarp::sig::Matrix out(v1.size(), v2.size());
    for(int r = 0; r < out.rows(); r++) {
        for(int c = 0; c < out.cols(); c++) {
            out(r, c) = v1(r) * v2(c);
        }
    }
    return out;
}

yarp::sig::Vector& addvec(yarp::sig::Vector& v, double val) {
    for(int i = 0; i < v.size(); i++) {
        v(i) += val;
    }
    return v;
}

void trsolve(const yarp::sig::Matrix& A, const yarp::sig::Vector& b, yarp::sig::Vector& x, bool transa) {
    gsl_matrix* Agsl = (gsl_matrix*) A.getGslMatrix();
    gsl_vector* bgsl = (gsl_vector*) b.getGslVector();
    gsl_vector* xgsl = (gsl_vector*) x.getGslVector();

    gsl_vector_memcpy(xgsl, bgsl);
    CBLAS_TRANSPOSE trans = transa ? CblasTrans : CblasNoTrans;
    gsl_blas_dtrsv(CblasUpper, trans, CblasNonUnit, Agsl, xgsl);
}

yarp::sig::Vector trsolve(const yarp::sig::Matrix& A, const yarp::sig::Vector& b, bool transa) {
    yarp::sig::Vector x(b.size());
    trsolve(A, b, x, transa);
    return x;
}

void fillrandom(yarp::sig::Vector& v, yarp::math::RandScalar& prng) {
    int i;
    for(i = 0; i < v.size(); i++) {
        v(i) = prng.get();
    }
}

void fillrandom(yarp::sig::Matrix& M, yarp::math::RandScalar& prng) {
    int r, c;
    for(r = 0; r < M.rows(); r++) {
        for(c = 0; c < M.cols(); c++) {
            M(r, c) = prng.get();
        }
    }
}

void fillrandom(yarp::sig::Vector& v, yarp::math::RandnScalar& prng) {
    int i;
    for(i = 0; i < v.size(); i++) {
        v(i) = prng.get();
    }
}

void fillrandom(yarp::sig::Matrix& M, yarp::math::RandnScalar& prng) {
    int r, c;
    for(r = 0; r < M.rows(); r++) {
        for(c = 0; c < M.cols(); c++) {
            M(r, c) = prng.get();
        }
    }
}

yarp::sig::Vector random(int length, yarp::math::RandScalar& prng) {
    yarp::sig::Vector v(length);
    fillrandom(v, prng);
    return v;
}

yarp::sig::Matrix random(int rows, int columns, yarp::math::RandScalar& prng) {
    yarp::sig::Matrix M(rows, columns);
    fillrandom(M, prng);
    return M;
}

yarp::sig::Vector random(int length, yarp::math::RandnScalar& prng) {
    yarp::sig::Vector v(length);
    fillrandom(v, prng);
    return v;
}

yarp::sig::Matrix random(int rows, int columns, yarp::math::RandnScalar& prng) {
    yarp::sig::Matrix M(rows, columns);
    fillrandom(M, prng);
    return M;
}

yarp::sig::Vector& map(yarp::sig::Vector& v, double (op)(double)) {
    for(int i = 0; i < v.size(); i++) {
        v(i) = op(v(i));
    }
    return v;
}

yarp::sig::Matrix& map(yarp::sig::Matrix& M, double (op)(double)) {
    for(int r = 0; r < M.rows(); r++) {
        for(int c = 0; c < M.cols(); c++) {
            M(r, c) = op(M(r, c));
        }
    }
    return M;
}

yarp::sig::Vector map(const yarp::sig::Vector& v, double (op)(double)) {
    yarp::sig::Vector out(v.size());
    for(int i = 0; i < out.size(); i++) {
        out(i) = op(v(i));
    }
    return out;
}

yarp::sig::Matrix map(const yarp::sig::Matrix& M, double (op)(double)) {
    yarp::sig::Matrix out(M.rows(), M.cols());
    for(int r = 0; r < out.rows(); r++) {
        for(int c = 0; c < out.cols(); c++) {
            out(r, c) = op(M(r, c));
        }
    }
    return out;
}

yarp::sig::Vector& cosvec(yarp::sig::Vector& v){
    return map(v, std::cos);
}

yarp::sig::Vector& sinvec(yarp::sig::Vector& v){
    return map(v, std::sin);
}

yarp::sig::Matrix& cosmat(yarp::sig::Matrix& M) {
    return map(M, std::cos);
}

yarp::sig::Matrix& sinmat(yarp::sig::Matrix& M) {
    return map(M, std::sin);
}

yarp::sig::Vector cosvec(const yarp::sig::Vector& v){
    return map(v, std::cos);
}

yarp::sig::Vector sinvec(const yarp::sig::Vector& v){
    return map(v, std::sin);
}

yarp::sig::Matrix cosmat(const yarp::sig::Matrix& M) {
    return map(M, std::cos);
}

yarp::sig::Matrix sinmat(const yarp::sig::Matrix& M) {
    return map(M, std::sin);
}

} // math
} // learningmachine
} // iCub

