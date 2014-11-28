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

#include <string>

#include <gsl/gsl_math.h>

#include <yarp/os/Log.h>
#include <yarp/math/SVD.h>
#include <iCub/ctrl/math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


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
Vector iCub::ctrl::cross(const Matrix &A, int colA, const Matrix &B, int colB,
                         unsigned int verbose)
{
    Vector v(3);
    if ((A.rows()>=3) && (B.rows()>=3))
    {
        v[0]=A(1,colA)*B(2,colB)-A(2,colA)*B(1,colB);
        v[1]=A(2,colA)*B(0,colB)-A(0,colA)*B(2,colB);
        v[2]=A(0,colA)*B(1,colB)-A(1,colA)*B(0,colB);
    }
    else if (verbose)
        yError("cross() failed: not 3xn input matrixes");

    return v;
}


/************************************************************************/
Vector iCub::ctrl::Dcross(const Vector &a, const Vector &Da, const Vector &b,
                          const Vector &Db, unsigned int verbose)
{
    Vector Dv(3);
    if ((a.length()>=3) && (b.length()>=3) && (Da.length()>=3) && (Db.length()>=3))
    {
        Dv[0]=Da[1]*b[2]+a[1]*Db[2]-Da[2]*b[1]-a[2]*Db[1];
        Dv[1]=Da[2]*b[0]+a[2]*Db[0]-Da[0]*b[2]-a[0]*Db[2];
        Dv[2]=Da[0]*b[1]+a[0]*Db[1]-Da[1]*b[0]-a[1]*Db[0];
    }
    else if (verbose)
        yError("Dcross() failed: not 3x1 input vectors");

    return Dv;
}


/************************************************************************/
Vector iCub::ctrl::Dcross(const Matrix &A, const Matrix &DA, int colA,
                          const Matrix &B, const Matrix &DB, int colB,
                          unsigned int verbose)
{
    Vector Dv(3);
    if ((A.rows()>=3) && (B.rows()>=3) && (DA.rows()>=3) && (DB.rows()>=3))
    {
        Dv[0]=DA(1,colA)*B(2,colB)+A(1,colA)*DB(2,colB)-DA(2,colA)*B(1,colB)-A(2,colA)*DB(1,colB);
        Dv[1]=DA(2,colA)*B(0,colB)+A(2,colA)*DB(0,colB)-DA(0,colA)*B(2,colB)-A(0,colA)*DB(2,colB);
        Dv[2]=DA(0,colA)*B(1,colB)+A(0,colA)*DB(1,colB)-DA(1,colA)*B(0,colB)-A(1,colA)*DB(0,colB);
    }
    else if (verbose)
        yError("Dcross() failed: not 3xn input matrixes");

    return Dv;
}


/************************************************************************/
Vector iCub::ctrl::dcm2axis(const Matrix &R, unsigned int verbose)
{
    if ((R.rows()<3) || (R.cols()<3))
    {
        if (verbose)
            yError("dcm2axis() failed");

        return Vector(0);
    }

    Vector v(4);
    v[0]=R(2,1)-R(1,2);
    v[1]=R(0,2)-R(2,0);
    v[2]=R(1,0)-R(0,1);
    v[3]=0.0;
    double r=yarp::math::norm(v);
    double theta=atan2(0.5*r,0.5*(R(0,0)+R(1,1)+R(2,2)-1));

    if (r<1e-9)
    {
        // if we enter here, then 
        // R is symmetric; this can
        // happen only if the rotation
        // angle is 0 (R=I) or 180 degrees
        Matrix A=R.submatrix(0,2,0,2);
        Matrix U(3,3), V(3,3);
        Vector S(3);

        // A=I+sin(theta)*S+(1-cos(theta))*S^2
        // where S is the skew matrix.
        // Given a point x, A*x is the rotated one,
        // hence if Ax=x then x belongs to the rotation
        // axis. We have therefore to find the kernel of
        // the linear application (A-I).
        SVD(A-eye(3,3),U,S,V);

        v[0]=V(0,2);
        v[1]=V(1,2);
        v[2]=V(2,2);
        r=yarp::math::norm(v);
    }

    v=(1.0/r)*v;
    v[3]=theta;

    return v;
}


/************************************************************************/
Matrix iCub::ctrl::axis2dcm(const Vector &v, unsigned int verbose)
{
    if (v.length()<4)
    {
        if (verbose)
            yError("axis2dcm() failed");
    
        return Matrix(0,0);
    }

    Matrix R=eye(4,4);

    double theta=v[3];
    if (theta==0.0)
        return R;

    double c=cos(theta);
    double s=sin(theta);
    double C=1.0-c;

    double xs =v[0]*s;
    double ys =v[1]*s;
    double zs =v[2]*s;
    double xC =v[0]*C;
    double yC =v[1]*C;
    double zC =v[2]*C;
    double xyC=v[0]*yC;
    double yzC=v[1]*zC;
    double zxC=v[2]*xC;
    
    R(0,0)=v[0]*xC+c;
    R(0,1)=xyC-zs;
    R(0,2)=zxC+ys;
    R(1,0)=xyC+zs;
    R(1,1)=v[1]*yC+c;
    R(1,2)=yzC-xs;
    R(2,0)=zxC-ys;
    R(2,1)=yzC+xs;
    R(2,2)=v[2]*zC+c;

    return R;
}


/************************************************************************/
Vector iCub::ctrl::dcm2euler(const Matrix &R, unsigned int verbose)
{
    if ((R.rows()<3) || (R.cols()<3))
    {
        if (verbose)
            yError("dcm2euler() failed");

        return Vector(0);
    }

    Vector v(3);
    bool singularity=false;
    if (R(2,2)<1.0)
    {
        if (R(2,2)>-1.0)
        {
            v[0]=atan2(R(1,2),R(0,2));
            v[1]=acos(R(2,2));
            v[2]=atan2(R(2,1),-R(2,0));
        }
        else
        {
            // Not a unique solution: gamma-alpha=atan2(R10,R11)
            singularity=true;
            v[0]=-atan2(R(1,0),R(1,1));
            v[1]=M_PI;
            v[2]=0.0;
        }
    }
    else
    {
        // Not a unique solution: gamma+alpha=atan2(R10,R11)
        singularity=true;
        v[0]=atan2(R(1,0),R(1,1));
        v[1]=0.0;
        v[2]=0.0;
    }

    if (verbose && singularity)
        yWarning("dcm2euler() in singularity: choosing one solution among multiple");

    return v;
}


/************************************************************************/
Matrix iCub::ctrl::euler2dcm(const Vector &v, unsigned int verbose)
{
    if (v.length()<3)
    {
        if (verbose)
            yError("euler2dcm() failed");
    
        return Matrix(0,0);
    }

    Matrix Rza=eye(4,4); Matrix Ryb=eye(4,4);  Matrix Rzg=eye(4,4);
    double alpha=v[0];   double ca=cos(alpha); double sa=sin(alpha);
    double beta=v[1];    double cb=cos(beta);  double sb=sin(beta);
    double gamma=v[2];   double cg=cos(gamma); double sg=sin(gamma);
    
    Rza(0,0)=ca; Rza(1,1)=ca; Rza(1,0)= sa; Rza(0,1)=-sa;
    Rzg(0,0)=cg; Rzg(1,1)=cg; Rzg(1,0)= sg; Rzg(0,1)=-sg;
    Ryb(0,0)=cb; Ryb(2,2)=cb; Ryb(2,0)=-sb; Ryb(0,2)= sb;

    return Rza*Ryb*Rzg;
}


/************************************************************************/
Vector iCub::ctrl::dcm2rpy(const Matrix &R, unsigned int verbose)
{
    if ((R.rows()<3) || (R.cols()<3))
    {
        if (verbose)
            yError("dcm2rpy() failed");

        return Vector(0);
    }

    Vector v(3);
    bool singularity=false;
    if (R(2,0)<1.0)
    {
        if (R(2,0)>-1.0)
        {
            v[0]=atan2(R(2,1),R(2,2));
            v[1]=asin(-R(2,0));
            v[2]=atan2(R(1,0),R(0,0));
        }
        else
        {
            // Not a unique solution: psi-phi=atan2(-R12,R11)
            v[0]=0.0;
            v[1]=M_PI/2.0;
            v[2]=-atan2(-R(1,2),R(1,1));
        }
    }
    else
    {
        // Not a unique solution: psi+phi=atan2(-R12,R11)
        v[0]=0.0;
        v[1]=-M_PI/2.0;
        v[2]=atan2(-R(1,2),R(1,1));
    }

    if (verbose && singularity)
        yWarning("dcm2rpy() in singularity: choosing one solution among multiple");

    return v;
}


/************************************************************************/
Matrix iCub::ctrl::rpy2dcm(const Vector &v, unsigned int verbose)
{
    if (v.length()<3)
    {
        if (verbose)
            yError("rpy2dcm() failed");
    
        return Matrix(0,0);
    }

    Matrix Rz=eye(4,4); Matrix Ry=eye(4,4);   Matrix Rx=eye(4,4);
    double roll=v[0];   double cr=cos(roll);  double sr=sin(roll);
    double pitch=v[1];  double cp=cos(pitch); double sp=sin(pitch);
    double yaw=v[2];    double cy=cos(yaw);   double sy=sin(yaw);
    
    Rz(0,0)=cy; Rz(1,1)=cy; Rz(0,1)=-sy; Rz(1,0)= sy;   // z-rotation with yaw
    Ry(0,0)=cp; Ry(2,2)=cp; Ry(0,2)= sp; Ry(2,0)=-sp;   // y-rotation with pitch
    Rx(1,1)=cr; Rx(2,2)=cr; Rx(1,2)=-sr; Rx(2,1)= sr;   // x-rotation with roll

    return Rz*Ry*Rx;
}


/************************************************************************/
Matrix iCub::ctrl::SE3inv(const Matrix &H, unsigned int verbose)
{    
    if ((H.rows()!=4) || (H.cols()!=4))
    {
        if (verbose)
            yError("SE3inv() failed");

        return Matrix(0,0);
    }

    Vector p(4);
    p[0]=H(0,3);
    p[1]=H(1,3);
    p[2]=H(2,3);
    p[3]=1.0;

    Matrix invH=H.transposed();
    p=invH*p;
        
    invH(0,3)=-p[0];
    invH(1,3)=-p[1];
    invH(2,3)=-p[2];
    invH(3,0)=invH(3,1)=invH(3,2)=0.0;

    return invH;
}


/************************************************************************/
Vector iCub::ctrl::sign(const Vector &v)
{
    Vector ret(v.length());
    for (size_t i=0; i<v.length(); i++)
        ret[i]=sign(v[i]);

    return ret;
}


/************************************************************************/
Matrix iCub::ctrl::adjoint(const Matrix &H, unsigned int verbose)
{
    if ((H.rows()!=4) || (H.cols()!=4))
    {
        if (verbose)
            yError("adjoint() failed: roto-translational matrix sized %dx%d instead of 4x4",
                   H.rows(),H.cols());

        return Matrix(0,0);
    }

    // the skew matrix coming from the translational part of H: S(r)
    Matrix S(3,3);
    S(0,0)= 0.0;    S(0,1)=-H(2,3); S(0,2)= H(1,3);
    S(1,0)= H(2,3); S(1,1)= 0.0;    S(1,2)=-H(0,3);
    S(2,0)=-H(1,3); S(2,1)= H(0,3); S(2,2)= 0.0;

    S = S*H.submatrix(0,2,0,2);

    Matrix A(6,6); A.zero();
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            A(i,j)     = H(i,j);
            A(i+3,j+3) = H(i,j);
            A(i,j+3)   = S(i,j);
        }
    }

    return A;
}


/************************************************************************/
Matrix iCub::ctrl::adjointInv(const Matrix &H, unsigned int verbose)
{
    if ((H.rows()!=4) || (H.cols()!=4))
    {
        if (verbose)
            yError("adjointInv() failed: roto-translational matrix sized %dx%d instead of 4x4",
                   H.rows(),H.cols());

        return Matrix(0,0);
    }
    
    // R^T
    Matrix Rt = H.submatrix(0,2,0,2).transposed();
    // R^T * r
    Vector Rtp = Rt*H.getCol(3).subVector(0,2);

    Matrix S(3,3);
    S(0,0)= 0.0;    S(0,1)=-Rtp(2); S(0,2)= Rtp(1);
    S(1,0)= Rtp(2); S(1,1)= 0.0;    S(1,2)=-Rtp(0);
    S(2,0)=-Rtp(1); S(2,1)= Rtp(0); S(2,2)= 0.0;

    S = S*Rt;

    Matrix A(6,6); A.zero();
    for (int i=0; i<3; i++)
    {
        for (int j=0; j<3; j++)
        {
            A(i,j)     = Rt(i,j);
            A(i+3,j+3) = Rt(i,j);
            A(i,j+3)   = -S(i,j);
        }
    }

    return A;
}


