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

#include <yarp/math/SVD.h>
#include <iCub/ctrl/math.h>

#include <string>
#include <stdio.h>

using namespace std;
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
Vector iCub::ctrl::cross(const Matrix &A, int colA, const Matrix &B, int colB,
                         unsigned int verbose)
{
    Vector v(3);

    if (A.rows()>=3 && B.rows()>=3)
    {
        v[0]=A(1,colA)*B(2,colB)-A(2,colA)*B(1,colB);
        v[1]=A(2,colA)*B(0,colB)-A(0,colA)*B(2,colB);
        v[2]=A(0,colA)*B(1,colB)-A(1,colA)*B(0,colB);
    }
    else if (verbose)
        fprintf(stderr,"cross() failed: not 3xn input matrixes\n");

    return v;
}


/************************************************************************/
Vector iCub::ctrl::Dcross(const Vector &a, const Vector &Da, const Vector &b,
                          const Vector &Db, unsigned int verbose)
{
    Vector Dv(3);

    if (a.length()>=3 && b.length()>=3 && Da.length()>=3 && Db.length()>=3)
    {
        Dv[0]=Da[1]*b[2]+a[1]*Db[2]-Da[2]*b[1]-a[2]*Db[1];
        Dv[1]=Da[2]*b[0]+a[2]*Db[0]-Da[0]*b[2]-a[0]*Db[2];
        Dv[2]=Da[0]*b[1]+a[0]*Db[1]-Da[1]*b[0]-a[1]*Db[0];
    }
    else if (verbose)
        fprintf(stderr,"Dcross() failed: not 3x1 input vectors\n");

    return Dv;
}


/************************************************************************/
Vector iCub::ctrl::Dcross(const Matrix &A, const Matrix &DA, int colA,
                          const Matrix &B, const Matrix &DB, int colB,
                          unsigned int verbose)
{
    Vector Dv(3);

    if (A.rows()>=3 && B.rows()>=3 && DA.rows()>=3 && DB.rows()>=3)
    {
        Dv[0]=DA(1,colA)*B(2,colB)+A(1,colA)*DB(2,colB)-DA(2,colA)*B(1,colB)-A(2,colA)*DB(1,colB);
        Dv[1]=DA(2,colA)*B(0,colB)+A(2,colA)*DB(0,colB)-DA(0,colA)*B(2,colB)-A(0,colA)*DB(2,colB);
        Dv[2]=DA(0,colA)*B(1,colB)+A(0,colA)*DB(1,colB)-DA(1,colA)*B(0,colB)-A(1,colA)*DB(0,colB);
    }
    else if (verbose)
        fprintf(stderr,"Dcross() failed: not 3xn input matrixes\n");

    return Dv;
}


/************************************************************************/
Vector iCub::ctrl::dcm2axis(const Matrix &R, unsigned int verbose)
{
    if (R.rows()<3 || R.cols()<3)
    {
        if (verbose)
            fprintf(stderr,"dcm2axis() failed\n");

        return Vector(0);
    }

    Vector v(4); v=0.0;
    v[0]=R(2,1)-R(1,2);
    v[1]=R(0,2)-R(2,0);
    v[2]=R(1,0)-R(0,1);
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
            fprintf(stderr,"axis2dcm() failed\n");
    
        return Matrix(0,0);
    }

    Matrix R=eye(4,4);

    double theta=v[3];

    if (!theta)
        return R;

    double c=cos(theta);
    double s=sin(theta);
    double C=1-c;

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
    if (R.rows()<3 || R.cols()<3)
    {
        if (verbose)
            fprintf(stderr,"dcm2euler() failed\n");

        return Vector(0);
    }

    Vector v(3);
    double r2 = R(2,0)*R(2,0) + R(2,1)*R(2,1);
    if (r2 > 0)
    {
        v[1]=atan2(sqrt(r2), R(2,2));
        v[0]=atan2(R(1,2)/sin(v[1]), R(0,2)/sin(v[1]));
        v[2]=atan2(R(2,1)/sin(v[1]),-R(2,0)/sin(v[1]));
    }
    else
    {
        if (verbose)
            fprintf(stderr,"dcm2euler() in singularity: choosing one solution among multiple\n");

        v[1]=0;
        v[0]=atan2(R(1,0), R(0,0));
        v[2]=0;
    }

    return v;
}


/************************************************************************/
Matrix iCub::ctrl::euler2dcm(const Vector &v, unsigned int verbose)
{
    if (v.length()<3)
    {
        if (verbose)
            fprintf(stderr,"euler2dcm() failed\n");
    
        return Matrix(0,0);
    }

    Matrix Rza=eye(4,4);  Matrix Ryb=eye(4,4);   Matrix Rzg=eye(4,4);
    double alpha = v[0];  double ca=cos(alpha);  double sa=sin(alpha);
    double beta  = v[1];  double cb=cos(beta);   double sb=sin(beta);
    double gamma = v[2];  double cg=cos(gamma);  double sg=sin(gamma);
    
    Rza(0,0)=ca;   Rza(1,1)=ca;   Rza(1,0)=sa;   Rza(0,1)=-sa;
    Rzg(0,0)=cg;   Rzg(1,1)=cg;   Rzg(1,0)=sg;   Rzg(0,1)=-sg;
    Ryb(0,0)=cb;   Ryb(2,2)=cb;   Ryb(2,0)=-sb;  Ryb(0,2)= sb;

    return Rza*Ryb*Rzg;
}


/************************************************************************/
Matrix iCub::ctrl::SE3inv(const Matrix &H, unsigned int verbose)
{    
    if (H.rows()<4 || H.cols()<4)
    {
        if (verbose)
            fprintf(stderr,"SE3inv() failed\n");

        return Matrix(0,0);
    }

    Matrix invH(4,4);
    Vector p(3);

    Matrix Rt=H.submatrix(0,2,0,2).transposed();
    p[0]=H(0,3);
    p[1]=H(1,3);
    p[2]=H(2,3);

    p=Rt*p;

    for (unsigned int i=0; i<3; i++)
        for (unsigned int j=0; j<3; j++)
            invH(i,j)=Rt(i,j);

    invH(0,3)=-p[0];
    invH(1,3)=-p[1];
    invH(2,3)=-p[2];
    invH(3,3)=1.0;

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
    if((H.rows()!=4) || (H.cols()!=4))
    {
        if (verbose)
            fprintf(stderr,"CtrlLib: error, adjoint() failed due to wrong sized roto-translational matrix, sized %dx%d instead of 4x4\n",
                    H.rows(),H.cols());

        return Matrix(0,0);
    }

    // the skew matrix coming from the translational part of H: S(r)
    // note the commented zeros above to show how the matrix looks like..
    Matrix S(3,3); S.zero();
        /* 0 */     S(0,1)=-H(2,3); S(0,2)= H(1,3);
    S(1,0)= H(2,3);      /* 0 */    S(1,3)=-H(0,3);
    S(2,0)=-H(1,3); S(2,1)= H(0,3);     /* 0 */

    // this is S(r)*R
    S = S*H.submatrix(0,2,0,2);

    Matrix A(6,6); A.zero();
    unsigned int i,j;
    for(i=0; i<3; i++)
    {
        for(j=0;j<3;j++)
        {
            A(i,j)      = H(i,j);
            A(i+3,j+3)  = H(i,j);
            A(i,j+3)    = S(i,j);
        }
    }
    return A;
}


/************************************************************************/
Matrix iCub::ctrl::adjointInv(const Matrix &H, unsigned int verbose)
{
    if((H.rows()!=4) || (H.cols()!=4))
    {
        if (verbose)
            fprintf(stderr,"CtrlLib: error, adjointInv() failed due to wrong sized roto-translational matrix, sized %dx%d instead of 4x4\n",
                    H.rows(),H.cols());

        return Matrix(0,0);
    }
    
    // R^T
    Matrix Rt = H.submatrix(0,2,0,2).transposed();
    // R^T * r
    Vector Rtp = Rt*H.submatrix(0,2,0,3).getCol(3);

    // the skew matrix coming from the translational part of H: S(r)
    // note the commented zeros above to show how the matrix looks like..
    Matrix S(3,3); S.zero();
        /* 0 */       S(0,1)=-Rtp(2);     S(0,2)= Rtp(1);
    S(1,0)= Rtp(2);      /* 0 */          S(1,3)=-Rtp(0);
    S(2,0)=-Rtp(1);   S(2,1)= Rtp(0);     /* 0 */

    // this is S(r)*Rt
    S = S*Rt;

    Matrix A(6,6); A.zero();
    unsigned int i,j;
    for(i=0; i<3; i++)
    {
        for(j=0;j<3;j++)
        {
            A(i,j)      = Rt(i,j);
            A(i+3,j+3)  = Rt(i,j);
            A(i,j+3)    = -S(i,j);
        }
    }
    return A;
}


