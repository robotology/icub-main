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
#include <string>

#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


/************************************************************************/
namespace iCub {
    namespace ctrl {
        const double CTRL_PI=M_PI;
        const double CTRL_RAD2DEG=180.0/M_PI;
        const double CTRL_DEG2RAD=M_PI/180.0;
    }
}


/************************************************************************/
double iCub::ctrl::dot(const Matrix &A, int colA, const Matrix &B, int colB)
{
    double ret=0.0;
    size_t nrowsA=A.rows();
    size_t nrowsB=B.rows();
    size_t nrows=nrowsA<nrowsB ? nrowsA : nrowsB;

    for (unsigned int i=0; i<nrows; i++)
        ret+=A(i,colA)*B(i,colB);

    return ret;
}


/************************************************************************/
double iCub::ctrl::norm(const Matrix &M, int col)
{
    return sqrt(norm2(M,col));
}


/************************************************************************/
Vector iCub::ctrl::cross(const Matrix &A, int colA, const Matrix &B, int colB)
{    
    yAssert((A.rows()>=3) && (B.rows()>=3));

    Vector v(3);
    v[0]=A(1,colA)*B(2,colB)-A(2,colA)*B(1,colB);
    v[1]=A(2,colA)*B(0,colB)-A(0,colA)*B(2,colB);
    v[2]=A(0,colA)*B(1,colB)-A(1,colA)*B(0,colB);

    return v;
}


/************************************************************************/
Vector iCub::ctrl::Dcross(const Vector &a, const Vector &Da, const Vector &b,
                          const Vector &Db)
{    
    yAssert((a.length()>=3) && (b.length()>=3) &&
            (Da.length()>=3) && (Db.length()>=3));

    Vector Dv(3);
    Dv[0]=Da[1]*b[2]+a[1]*Db[2]-Da[2]*b[1]-a[2]*Db[1];
    Dv[1]=Da[2]*b[0]+a[2]*Db[0]-Da[0]*b[2]-a[0]*Db[2];
    Dv[2]=Da[0]*b[1]+a[0]*Db[1]-Da[1]*b[0]-a[1]*Db[0];

    return Dv;
}


/************************************************************************/
Vector iCub::ctrl::Dcross(const Matrix &A, const Matrix &DA, int colA,
                          const Matrix &B, const Matrix &DB, int colB)
{    
    yAssert((A.rows()>=3) && (B.rows()>=3) &&
            (DA.rows()>=3) && (DB.rows()>=3));

    Vector Dv(3);
    Dv[0]=DA(1,colA)*B(2,colB)+A(1,colA)*DB(2,colB)-DA(2,colA)*B(1,colB)-A(2,colA)*DB(1,colB);
    Dv[1]=DA(2,colA)*B(0,colB)+A(2,colA)*DB(0,colB)-DA(0,colA)*B(2,colB)-A(0,colA)*DB(2,colB);
    Dv[2]=DA(0,colA)*B(1,colB)+A(0,colA)*DB(1,colB)-DA(1,colA)*B(0,colB)-A(1,colA)*DB(0,colB);

    return Dv;
}


