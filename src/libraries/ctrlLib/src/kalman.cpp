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

#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/kalman.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/**********************************************************************/
void Kalman::initialize()
{
    n=A.rows();
    m=H.rows();

    At=A.transposed();
    Ht=H.transposed();
    I=eye(n,n);

    x.resize(n,0.0);
    P.resize(n,n); P.zero();
    K.resize(n,m); K.zero();
    S.resize(m,m); S.zero();
    validationGate=0.0;
}


/**********************************************************************/
Kalman::Kalman(const Matrix &_A, const Matrix &_H, const Matrix &_Q,
               const Matrix &_R) : A(_A), H(_H), Q(_Q), R(_R)
{
    initialize();
    B.resize(n,n); B.zero();
}


/**********************************************************************/
Kalman::Kalman(const Matrix &_A, const Matrix &_B, const Matrix &_H,
               const Matrix &_Q, const Matrix &_R) : A(_A), B(_B), H(_H), Q(_Q), R(_R)
{
    initialize();
}


/**********************************************************************/
bool Kalman::init(const Vector &_x0, const Matrix &_P0)
{
    if ((_x0.length()==x.length()) && (_P0.rows()==P.rows()) && (_P0.cols()==P.cols()))
    {
        x=_x0;
        P=_P0;
        return true;
    }
    else
        return false;
}


/**********************************************************************/
const Vector& Kalman::predict(const Vector &u)
{
    x=A*x+B*u;
    P=A*P*At+Q;
    S=H*P*Ht+R;
    validationGate=0.0;
    return x;
}


/**********************************************************************/
const Vector& Kalman::predict()
{
    return predict(Vector(n,0.0));
}


/**********************************************************************/
const Vector& Kalman::correct(const Vector &z)
{
    Matrix invS=pinv(S);
    K=P*Ht*invS;
    Vector e=z-get_y();
    x+=K*e;
    P=(I-K*H)*P;
    validationGate=yarp::math::dot(e,invS*e);
    return x;
}


/**********************************************************************/
const Vector& Kalman::filt(const Vector &u, const Vector &z)
{
    predict(u);
    correct(z);
    return x;
}


/**********************************************************************/
const Vector& Kalman::filt(const Vector &z)
{
    return filt(Vector(n,0.0),z);
}


/**********************************************************************/
Vector Kalman::get_y() const
{
    return H*x;
}


/**********************************************************************/
bool Kalman::set_A(const yarp::sig::Matrix &_A)
{
    if ((_A.cols()==A.cols()) && (_A.rows()==A.rows()))
    {
        A=_A;
        At=A.transposed();
        return true;
    }
    else
        return false;
}


/**********************************************************************/
bool Kalman::set_B(const yarp::sig::Matrix &_B)
{
    if ((_B.cols()==B.cols()) && (_B.rows()==B.rows()))
    {
        B=_B;
        return true;
    }
    else
        return false;
}


/**********************************************************************/
bool Kalman::set_H(const yarp::sig::Matrix &_H)
{
    if ((_H.cols()==H.cols()) && (_H.rows()==H.rows()))
    {
        H=_H;
        Ht=H.transposed();
        return true;
    }
    else
        return false;
}


/**********************************************************************/
bool Kalman::set_Q(const yarp::sig::Matrix &_Q)
{
    if ((_Q.cols()==Q.cols()) && (_Q.rows()==Q.rows()))
    {
        Q=_Q;
        return true;
    }
    else
        return false;
}


/**********************************************************************/
bool Kalman::set_R(const yarp::sig::Matrix &_R)
{
    if ((_R.cols()==R.cols()) && (_R.rows()==R.rows()))
    {
        R=_R;
        return true;
    }
    else
        return false;
}


