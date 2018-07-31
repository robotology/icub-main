/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

#include <cmath>

#include <yarp/os/Log.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/optimalControl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Riccati::Riccati(const Matrix &_A, const Matrix &_B, const Matrix &_V,
                 const Matrix &_P, const Matrix &_VN, bool verb) 
{
    A = _A; At = A.transposed();
    B = _B; Bt = B.transposed();
    V = _V;
    P = _P;
    VN = _VN;

    Ti = new Matrix[1]; Ti[0].resize(1,1); Ti[0].zero();
    Li = new Matrix[1]; Li[0].resize(1,1); Li[0].zero();

    n=A.rows();
    m=B.rows();
    N=-1;

    verbose=verb;
    if (verbose)
        yWarning("Riccati: problem defined, unsolved.");
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::setVerbose(bool verb)
{
    verbose=verb;
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix Riccati::L(int step)
{
    if(N<0) 
    {
        if (verbose)
            yWarning("Riccati: DARE has not been solved yet.");
        return Li[0];
    }
    if(step>=0 && step>=N)
    {
        if (verbose)
            yWarning("Riccati: Index for gain matrix out of bound.");
        return Li[0];
    }
    return Li[step];
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Matrix Riccati::T(int step)
{
    if(N<0) 
    {
        if (verbose)
            yError("Riccati: DARE has not been solved yet.");
        return Ti[0];
    }
    if(step>=0 && step>N)
    {
        if (verbose)
            yError("Riccati: Index for DARE matrix out of bound.");
        return Ti[0];
    }
    return Ti[step];
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::setProblemData(const Matrix &_A, const Matrix &_B, const Matrix &_V,
                             const Matrix &_P, const Matrix &_VN)
{
    A = _A; At = A.transposed();
    B = _B; Bt = B.transposed();
    V = _V;
    P = _P;
    VN = _VN;

    n=A.rows();
    m=B.rows();
    N=-1;

    if (verbose)
        yWarning("Riccati: problem defined, unsolved.");
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::solveRiccati(int steps)
{
    int i;
    N = steps;
    delete [] Ti;
    delete [] Li;
    Ti = new Matrix[steps+1];
    Li = new Matrix[steps];
    for(i=0; i<=steps; i++)
        Ti[i].resize(VN.rows(),VN.cols());		
    //init TN=VN
    Ti[steps]=VN;
    //compute backward all Ti
    for(i=steps-1; i>=0; i--)
    {
        lastT=Ti[i+1]; 
        //Ti = V + A' * (Ti+1 - Ti+1 * B * (P + B' * Ti+1 * B)^-1 * B' * Ti+1 )* A;
        Ti[i] = V + At *(lastT - lastT * B* pinv(P+Bt*lastT*B)*Bt*lastT )* A; 
    }
    //compute forward all Li
    for(i=0;i<steps; i++)
    {
        //Li = (P + B' * Ti+1 * B)^-1 * B' * Ti+1 * A
        Li[i] = pinv(P + Bt*Ti[i+1]*B) *Bt * Ti[i+1] * A;	
    }

    if (verbose)
        yInfo("Riccati: DARE solved, matrices Li and Ti computed and stored.");
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Vector Riccati::doLQcontrol(int step, const Vector &x)
{
    if(N<0) 
    {
        if (verbose)
            yError("Riccati: DARE has not been solved yet.");
        Vector ret(1,0.0);
        return ret;
    }
    if(step>=0 && step>N)
    {
        if (verbose)
            yError("Riccati: Index for DARE matrix out of bound.");
        Vector ret(1,0.0);
        return ret;
    }
    return (Li[step] * (-1.0*x));
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void Riccati::doLQcontrol(int step, const Vector &x, Vector &ret)
{
    if(N<0) 
    {
        if (verbose)
            yError("Riccati: DARE has not been solved yet.");
        ret.zero();
    }
    else if(step>=0 && step>N)
    {
        if (verbose)
            yError("Riccati: Index for DARE matrix out of bound.");
        ret.zero();
    }
    else
    {
        ret.resize(m);
        ret = Li[step] * (-1.0*x);
    }
}


