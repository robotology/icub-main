/*
 * Copyright (C) 2006-2018 Istituto Italiano di Tecnologia (IIT)
 * Copyright (C) 2006-2010 RobotCub Consortium
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the BSD-3-Clause license. See the accompanying LICENSE file for
 * details.
*/

#include <cstdlib>
#include <sstream>
#include <cmath>
#include <algorithm>

#include <yarp/os/Log.h>

#include <iCub/iKin/iKinFwd.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
void iCub::iKin::notImplemented(const unsigned int verbose)
{
    if (verbose)
        yError("iKin: not implemented");
}


/************************************************************************/
iKinLink::iKinLink(double _A, double _D, double _Alpha, double _Offset,
                   double _Min, double _Max): zeros1x1(zeros(1,1)), zeros1(zeros(1))
{
    A     =_A;
    D     =_D;
    Alpha =_Alpha;
    Offset=_Offset;

    Min=_Min;
    Max=_Max;
    Ang=Min;

    c_alpha=cos(Alpha);
    s_alpha=sin(Alpha);

    blocked    =false;
    cumulative =false;
    constrained=true;
    verbose    =0;

    H.resize(4,4);
    H.zero();
    DnH =H;
    cumH=H;
    cumH.eye();

    H(2,1)=s_alpha;
    H(2,2)=c_alpha;
    H(2,3)=D;
    H(3,3)=1.0;
}


/************************************************************************/
void iKinLink::clone(const iKinLink &l)
{
    A     =l.A;
    D     =l.D;
    Alpha =l.Alpha;
    Offset=l.Offset;

    c_alpha=l.c_alpha;
    s_alpha=l.s_alpha;

    Ang=l.Ang;
    Min=l.Min;
    Max=l.Max;

    blocked    =l.blocked;
    cumulative =l.cumulative;
    constrained=l.constrained;
    verbose    =l.verbose;

    H   =l.H;
    cumH=l.cumH;
    DnH =l.DnH;
}


/************************************************************************/
iKinLink::iKinLink(const iKinLink &l)
{
    clone(l);
}


/************************************************************************/
iKinLink &iKinLink::operator=(const iKinLink &l)
{
    clone(l);

    return *this;
}


/************************************************************************/
void iKinLink::setMin(const double _Min)
{
    Min=_Min;

    if (Ang<Min)
        Ang=Min;
}


/************************************************************************/
void iKinLink::setMax(const double _Max)
{
    Max=_Max;

    if (Ang>Max)
        Ang=Max;
}


/************************************************************************/
void iKinLink::setD(const double _D)
{
    H(2,3)=D=_D;
}


/************************************************************************/
void iKinLink::setAlpha(const double _Alpha)
{
    Alpha=_Alpha;

    H(2,2)=c_alpha=cos(Alpha);
    H(2,1)=s_alpha=sin(Alpha);
}


/************************************************************************/
double iKinLink::setAng(double _Ang)
{
    if (!blocked)
    {
        if (constrained)
            Ang=(_Ang<Min) ? Min : ((_Ang>Max) ? Max : _Ang);
        else
            Ang=_Ang;
    }
    else if (verbose)
        yWarning("Attempt to set joint angle to %g while blocked",_Ang);

    return Ang;
}


/************************************************************************/
Matrix iKinLink::getH(bool c_override)
{
    double theta=Ang+Offset;
    double c_theta=cos(theta);
    double s_theta=sin(theta);

    H(0,0)=c_theta;
    H(0,1)=-s_theta*c_alpha;
    H(0,2)=s_theta*s_alpha;
    H(0,3)=c_theta*A;

    H(1,0)=s_theta;
    H(1,1)=c_theta*c_alpha;
    H(1,2)=-c_theta*s_alpha;
    H(1,3)=s_theta*A;

    if (cumulative && !c_override)
        return cumH*H;
    else
        return H;
}


/************************************************************************/
Matrix iKinLink::getH(double _Ang, bool c_override)
{
    setAng(_Ang);

    return getH(c_override);
}


/************************************************************************/
Matrix iKinLink::getDnH(unsigned int n, bool c_override)
{
    if (n==0)
        return getH(c_override);
    else
    {
        double theta=Ang+Offset;
        double c_theta=cos(theta);
        double s_theta=sin(theta);

        int    C=(n>>1)&1 ? -1 : 1;

        if (n&1)
        {
            DnH(0,0)=-C*s_theta;
            DnH(0,1)=-C*c_theta*c_alpha;
            DnH(0,2)=C*c_theta*s_alpha;
            DnH(0,3)=-C*s_theta*A;
    
            DnH(1,0)=C*c_theta;
            DnH(1,1)=-C*s_theta*c_alpha;
            DnH(1,2)=C*s_theta*s_alpha;
            DnH(1,3)=C*c_theta*A;
        }
        else
        {
            DnH(0,0)=C*c_theta;
            DnH(0,1)=-C*s_theta*c_alpha;
            DnH(0,2)=C*s_theta*s_alpha;
            DnH(0,3)=C*c_theta*A;

            DnH(1,0)=C*s_theta;
            DnH(1,1)=C*c_theta*c_alpha;
            DnH(1,2)=-C*c_theta*s_alpha;
            DnH(1,3)=C*s_theta*A;
        }

        if (cumulative && !c_override)
            return cumH*DnH;
        else
            return DnH;
    }
}


/************************************************************************/
void iKinLink::addCumH(const Matrix &_cumH)
{
    cumulative=true;
    cumH=_cumH;
}


/************************************************************************/
iKinChain::iKinChain()
{
    N=DOF=verbose=0;
    H0=HN=eye(4,4);
}


/************************************************************************/
void iKinChain::clone(const iKinChain &c)
{
    N        =c.N;
    DOF      =c.DOF;
    H0       =c.H0;
    HN       =c.HN;
    curr_q   =c.curr_q;    
    verbose  =c.verbose;
    hess_J   =c.hess_J;
    hess_Jlnk=c.hess_Jlnk;

    allList.assign(c.allList.begin(),c.allList.end());
    quickList.assign(c.quickList.begin(),c.quickList.end());
    hash.assign(c.hash.begin(),c.hash.end());
    hash_dof.assign(c.hash_dof.begin(),c.hash_dof.end());
}


/************************************************************************/
iKinChain::iKinChain(const iKinChain &c)
{
    clone(c);
}


/************************************************************************/
iKinChain &iKinChain::operator=(const iKinChain &c)
{
    clone(c);

    return *this;
}


/************************************************************************/
bool iKinChain::addLink(const unsigned int i, iKinLink &l)
{
    if (i<=N)
    {
        allList.insert(allList.begin()+i,&l);
        N=(unsigned int)allList.size();

        build();

        return true;
    }
    else
    {
        if (verbose)
            yError("addLink() failed due to out of range index: %d>%d",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChain::rmLink(const unsigned int i)
{
    if (i<N)
    {
        allList.erase(allList.begin()+i);
        N=(unsigned int)allList.size();

        build();

        return true;
    }
    else
    {
        if (verbose)
            yError("rmLink() failed due to out of range index: %d>=%d",i,N);

        return false;
    }
}


/************************************************************************/
void iKinChain::pushLink(iKinLink &l)
{
    allList.push_back(&l);
    N=(unsigned int)allList.size();

    build();
}


/************************************************************************/
void iKinChain::clear()
{
    allList.clear();
    quickList.clear();
    hash.clear();
    hash_dof.clear();

    N=DOF=0;
    H0=HN=eye(4,4);
}


/************************************************************************/
iKinChain &iKinChain::operator<<(iKinLink &l)
{
    pushLink(l);

    return *this;
}


/************************************************************************/
void iKinChain::popLink()
{
    allList.pop_back();
    N=(unsigned int)allList.size();

    build();
}


/************************************************************************/
iKinChain &iKinChain::operator--(int)
{
    popLink();

    return *this;
}


/************************************************************************/
bool iKinChain::blockLink(const unsigned int i, double Ang)
{
    if (i<N)
    {
        allList[i]->block(Ang);
        build();

        return true;
    }
    else
    {
        if (verbose)
            yError("blockLink() failed due to out of range index: %d>=%d",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChain::setBlockingValue(const unsigned int i, double Ang)
{
    if (i<N)
    {
        if (allList[i]->isBlocked() && (Ang!=allList[i]->getAng()))
        {
            allList[i]->blocked=false; // remove the block temporarly
            allList[i]->block(Ang);    // update the blocked link

            // update the cumulative link which follows in the chain
            if (i<N-1)
            {
                Matrix H=eye(4,4);
                int j;

                for (j=i-1; j>=0; j--)
                    if (!allList[j]->isBlocked())
                        break;
                
                for (++j; j<=(int)i; j++)
                    H*=allList[j]->getH(true);
    
                for (; j<(int)N && !allList[j]->isCumulative(); j++)
                    H*=allList[j]->getH(true);
    
                allList[j]->addCumH(H);
            } 

            return true;
        }
        else
        {
            if (verbose)
                yError("setBlockingValue() failed since the %dth link was not already blocked",i);

            return false;
        }
    }
    else
    {
        if (verbose)
            yError("setBlockingValue() failed due to out of range index: %d>=%d",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChain::releaseLink(const unsigned int i)
{
    if (i<N)
    {
        allList[i]->release();
        build();

        return true;
    }
    else
    {    
        if (verbose)
            yError("releaseLink() failed due to out of range index: %d>=%d",i,N);

        return false;
    }
}


/************************************************************************/
bool iKinChain::isLinkBlocked(const unsigned int i)
{
    if (i<N)
        return allList[i]->isBlocked();
    else
    {    
        if (verbose)
            yError("isLinkBlocked() failed due to out of range index: %d>=%d",i,N);

        return false;
    }
}


/************************************************************************/
void iKinChain::setAllConstraints(bool _constrained)
{
    for (unsigned int i=0; i<N; i++)
        allList[i]->setConstraint(_constrained);
}


/************************************************************************/
void iKinChain::setAllLinkVerbosity(unsigned int _verbose)
{
    for (unsigned int i=0; i<N; i++)
        allList[i]->setVerbosity(_verbose);
}


/************************************************************************/
void iKinChain::build()
{
    quickList.clear();
    hash.clear();
    hash_dof.clear();
    DOF=0;

    Matrix H=eye(4,4);
    bool cumulOn=false;

    for (unsigned int i=0; i<N; i++)
    {
        allList[i]->rmCumH();

        if (allList[i]->isBlocked())
        {
            if (i==N-1)
            {    
                allList[i]->addCumH(H);
                quickList.push_back(allList[i]);
            }
            else
            {
                H*=allList[i]->getH();
                cumulOn=true;
            }
        }
        else
        {
            if (cumulOn)
                allList[i]->addCumH(H);

            DOF++;
            quickList.push_back(allList[i]);
            hash_dof.push_back((unsigned int)quickList.size()-1);
            hash.push_back(i);

            H.eye();
            cumulOn=false;
        }
    }

    if (DOF>0)
        curr_q.resize(DOF,0);
}


/************************************************************************/
bool iKinChain::setH0(const Matrix &_H0)
{
    if ((_H0.rows()==4) && (_H0.cols()==4))
    {
        H0=_H0;
        return true;
    }
    else
    {
        if (verbose)
            yError("Attempt to reference a wrong matrix H0 (not 4x4)");

        return false;
    }
}


/************************************************************************/
bool iKinChain::setHN(const Matrix &_HN)
{
    if ((_HN.rows()==4) && (_HN.cols()==4))
    {
        HN=_HN;
        return true;
    }
    else
    {
        if (verbose)
            yError("Attempt to reference a wrong matrix HN (not 4x4)");

        return false;
    }
}


/************************************************************************/
Vector iKinChain::setAng(const Vector &q)
{
    yAssert(DOF>0);

    size_t sz=std::min(q.length(),(size_t)DOF);
    for (size_t i=0; i<sz; i++)
        curr_q[i]=quickList[hash_dof[i]]->setAng(q[i]);

    return curr_q;
}


/************************************************************************/
Vector iKinChain::getAng()
{
    yAssert(DOF>0);

    for (unsigned int i=0; i<DOF; i++)
        curr_q[i]=quickList[hash_dof[i]]->getAng();

    return curr_q;
}


/************************************************************************/
double iKinChain::setAng(const unsigned int i, double _Ang)
{
    double res=0.0;

    if (i<N)
    {
        if (allList[i]->isBlocked())
        {
            setBlockingValue(i,_Ang);
            res=allList[i]->getAng();
        }
        else
            res=allList[i]->setAng(_Ang);
    }
    else if (verbose)
        yError("setAng() failed due to out of range index: %d>=%d",i,N);

    return res;
}


/************************************************************************/
double iKinChain::getAng(const unsigned int i)
{
    double res=0.0;

    if (i<N)
        res=allList[i]->getAng();
    else if (verbose)
        yError("getAng() failed due to out of range index: %d>=%d",i,N);

    return res;
}


/************************************************************************/
Vector iKinChain::RotAng(const Matrix &R)
{
    Vector r(3);

    // Euler Angles as XYZ (see dcm2angle.m)
    r[0]=atan2(-R(2,1),R(2,2));
    r[1]=asin(R(2,0));
    r[2]=atan2(-R(1,0),R(0,0));

    return r;
}


/************************************************************************/
Vector iKinChain::dRotAng(const Matrix &R, const Matrix &dR)
{
    Vector dr(3);

    dr[0]=(R(2,1)*dR(2,2) - R(2,2)*dR(2,1)) / (R(2,1)*R(2,1) + R(2,2)*R(2,2));
    dr[1]=dR(2,0)/sqrt(fabs(1-R(2,0)*R(2,0)));
    dr[2]=(R(1,0)*dR(0,0) - R(0,0)*dR(1,0)) / (R(1,0)*R(1,0) + R(0,0)*R(0,0));

    return dr;
}


/************************************************************************/
Vector iKinChain::d2RotAng(const Matrix &R, const Matrix &dRi,
                           const Matrix &dRj, const Matrix &d2R)
{
    Vector d2r(3);

    double y,yi,yj,yij,x,xi,xj,xij;
    double tmp1,tmp2;

    y  =-R(2,1);
    yi =-dRi(2,1);
    yj =-dRj(2,1);
    yij=-d2R(2,1);
    x  = R(2,2);
    xi = dRi(2,2);
    xj = dRj(2,2);
    xij= d2R(2,2);

    tmp1  =x*x+y*y;
    d2r[0]=((xj*yi+x*yij-xij*y-xi*yj)*tmp1 - 2.0*(x*yi-xi*y)*(x*xj+y*yj)) / (tmp1*tmp1);

    x  =R(2,0);
    xi =dRi(2,0);
    xj =dRj(2,0);
    xij=d2R(2,0);

    tmp1  =1-x*x;
    tmp2  =sqrt(fabs(tmp1));
    d2r[1]=(xij-(x*xi*xj)/tmp1) / (tmp1*tmp2);

    y  =-R(1,0);
    yi =-dRi(1,0);
    yj =-dRj(1,0);
    yij=-d2R(1,0);
    x  = R(0,0);
    xi = dRi(0,0);
    xj = dRj(0,0);
    xij= d2R(0,0);

    tmp1  =x*x+y*y;
    d2r[2]=((xj*yi+x*yij-xij*y-xi*yj)*tmp1 - 2.0*(x*yi-xi*y)*(x*xj+y*yj)) / (tmp1*tmp1);

    return d2r;
}


/************************************************************************/
Matrix iKinChain::getH(const unsigned int i, const bool allLink)
{
    Matrix H=H0;
    unsigned int _i,n;
    deque<iKinLink*> *l;
    bool cumulHN=false;
    bool c_override;

    if (allLink)
    {
        n=N;
        l=&allList;
        c_override=true;

        _i=i;
        if (_i>=N-1)
            cumulHN=true;
    }
    else
    {
        n=DOF;
        l=&quickList;
        c_override=false;

        if (i==DOF)
            _i=(unsigned int)quickList.size();
        else
            _i=i;

        if (hash[_i]>=N-1)
            cumulHN=true;
    }

    yAssert(i<n);

    for (unsigned int j=0; j<=_i; j++)
        H*=((*l)[j]->getH(c_override));

    if (cumulHN)
        H*=HN;

    return H;
}


/************************************************************************/
Matrix iKinChain::getH()
{
    // may be different from DOF since one blocked link may lie
    // at the end of the chain.
    unsigned int n=(unsigned int)quickList.size();
    Matrix H=H0;

    for (unsigned int i=0; i<n; i++)
        H*=quickList[i]->getH();

    return H*HN;
}


/************************************************************************/
Matrix iKinChain::getH(const Vector &q)
{
    yAssert(DOF>0);

    setAng(q);
    return getH();
}


/************************************************************************/
Vector iKinChain::Pose(const unsigned int i, const bool axisRep)
{
    Matrix H=getH(i,true);
    Vector v;

    if (i<N)
    {
        if (axisRep)
        {
            v.resize(7);
            Vector r=dcm2axis(H);
            v[0]=H(0,3);
            v[1]=H(1,3);
            v[2]=H(2,3);
            v[3]=r[0];
            v[4]=r[1];
            v[5]=r[2];
            v[6]=r[3];
        }
        else
        {
            v.resize(6);
            Vector r=RotAng(H);
            v[0]=H(0,3);
            v[1]=H(1,3);
            v[2]=H(2,3);
            v[3]=r[0];
            v[4]=r[1];
            v[5]=r[2];
        }
    }
    else if (verbose)
        yError("Pose() failed due to out of range index: %d>=%d",i,N);

    return v;
}


/************************************************************************/
Vector iKinChain::Position(const unsigned int i)
{
    yAssert(i<N);
    return getH(i,true).subcol(0,3,3);
}


/************************************************************************/
Vector iKinChain::EndEffPose(const bool axisRep)
{
    Matrix H=getH();
    Vector v;

    if (axisRep)
    {
        v.resize(7);
        Vector r=dcm2axis(H);
        v[0]=H(0,3);
        v[1]=H(1,3);
        v[2]=H(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
        v[6]=r[3];
    }
    else
    {
        v.resize(6);
        Vector r=RotAng(H);
        v[0]=H(0,3);
        v[1]=H(1,3);
        v[2]=H(2,3);
        v[3]=r[0];
        v[4]=r[1];
        v[5]=r[2];
    }

    return v;
}


/************************************************************************/
Vector iKinChain::EndEffPose(const Vector &q, const bool axisRep)
{
    yAssert(DOF>0);

    setAng(q);
    return EndEffPose(axisRep);
}


/************************************************************************/
Vector iKinChain::EndEffPosition()
{
    return getH().subcol(0,3,3);
}


/************************************************************************/
Vector iKinChain::EndEffPosition(const Vector &q)
{
    yAssert(DOF>0);

    setAng(q);
    return EndEffPosition();
}


/************************************************************************/
Matrix iKinChain::AnaJacobian(const unsigned int i, unsigned int col)
{
    yAssert(i<N);

    col=col>3 ? 3 : col;

    Matrix J(6,i+1);
    Matrix H,dH,_H;
    Vector dr;

    for (unsigned int j=0; j<=i; j++)
    {
        H=dH=H0;

        for (unsigned int k=0; k<=i; k++)
        {
            _H=allList[k]->getH(true);
            H*=_H;

            if (j==k)
                dH*=allList[k]->getDnH(1,true);
            else
                dH*=_H;
        }

        if (i>=N-1)
        {
            H*=HN;
            dH*=HN;
        }

        dr=dRotAng(H,dH);

        J(0,j)=dH(0,col);
        J(1,j)=dH(1,col);
        J(2,j)=dH(2,col);
        J(3,j)=dr[0];
        J(4,j)=dr[1];
        J(5,j)=dr[2];
    }

    return J;
}


/************************************************************************/
Matrix iKinChain::AnaJacobian(unsigned int col)
{
    yAssert(DOF>0);

    col=col>3 ? 3 : col;

    // may be different from DOF since one blocked link may lie
    // at the end of the chain.
    unsigned int n=(unsigned int)quickList.size();
    Matrix J(6,DOF);
    Matrix H,dH,_H;
    Vector dr;

    for (unsigned int i=0; i<DOF; i++)
    {
        H=dH=H0;

        for (unsigned int j=0; j<n; j++)
        {
            _H=quickList[j]->getH();
            H*=_H;

            if (hash_dof[i]==j)
                dH*=quickList[j]->getDnH();
            else
                dH*=_H;
        }

        H*=HN;
        dH*=HN;
        dr=dRotAng(H,dH);

        J(0,i)=dH(0,col);
        J(1,i)=dH(1,col);
        J(2,i)=dH(2,col);
        J(3,i)=dr[0];
        J(4,i)=dr[1];
        J(5,i)=dr[2];
    }

    return J;
}


/************************************************************************/
Matrix iKinChain::AnaJacobian(const Vector &q, unsigned int col)
{
    yAssert(DOF>0);

    setAng(q);
    return AnaJacobian(col);
}


/************************************************************************/
Matrix iKinChain::GeoJacobian(const unsigned int i)
{
    yAssert(i<N);

    Matrix J(6,i+1);
    Matrix PN,Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);

    for (unsigned int j=0; j<=i; j++)
        intH.push_back(intH[j]*allList[j]->getH(true));

    PN=intH[i+1];
    if (i>=N-1)
        PN=PN*HN;

    for (unsigned int j=0; j<=i; j++)
    {
        Z=intH[j];
        w=cross(Z,2,PN-Z,3);

        J(0,j)=w[0];
        J(1,j)=w[1];
        J(2,j)=w[2];
        J(3,j)=Z(0,2);
        J(4,j)=Z(1,2);
        J(5,j)=Z(2,2);
    }

    return J;
}


/************************************************************************/
Matrix iKinChain::GeoJacobian()
{
    yAssert(DOF>0);

    Matrix J(6,DOF);
    Matrix PN,Z;
    Vector w;

    deque<Matrix> intH;
    intH.push_back(H0);

    for (unsigned int i=0; i<N; i++)
        intH.push_back(intH[i]*allList[i]->getH(true));

    PN=intH[N]*HN;

    for (unsigned int i=0; i<DOF; i++)
    {
        unsigned int j=hash[i];

        Z=intH[j];
        w=cross(Z,2,PN-Z,3);

        J(0,i)=w[0];
        J(1,i)=w[1];
        J(2,i)=w[2];
        J(3,i)=Z(0,2);
        J(4,i)=Z(1,2);
        J(5,i)=Z(2,2);
    }

    return J;
}


/************************************************************************/
Matrix iKinChain::GeoJacobian(const Vector &q)
{
    yAssert(DOF>0);

    setAng(q);
    return GeoJacobian();
}


/************************************************************************/
Vector iKinChain::Hessian_ij(const unsigned int i, const unsigned int j)
{
    prepareForHessian();
    return fastHessian_ij(i,j);
}


/************************************************************************/
void iKinChain::prepareForHessian()
{
    if (DOF==0)
    {
        if (verbose)
            yError("prepareForHessian() failed since DOF==0");

        return;
    }

    hess_J=GeoJacobian();
}


/************************************************************************/
Vector iKinChain::fastHessian_ij(const unsigned int i, const unsigned int j)
{
    yAssert((i<DOF) && (j<DOF));

    // ref. E.D. Pohl, H. Lipkin, "A New Method of Robotic Motion Control Near Singularities",
    // Advanced Robotics, 1991
    Vector h(6,0.0);
    if(i<j)
    {
        //h.setSubvector(0,cross(hess_Jo,i,hess_Jl,j));
        h[0] = hess_J(4,i)*hess_J(2,j) - hess_J(5,i)*hess_J(1,j);
        h[1] = hess_J(5,i)*hess_J(0,j) - hess_J(3,i)*hess_J(2,j);
        h[2] = hess_J(3,i)*hess_J(1,j) - hess_J(4,i)*hess_J(0,j);
        //h.setSubvector(3,cross(hess_Jo,i,hess_Jo,j));
        h(3) = hess_J(4,i)*hess_J(5,j)-hess_J(5,i)*hess_J(4,j);
        h(4) = hess_J(5,i)*hess_J(3,j)-hess_J(3,i)*hess_J(5,j);
        h(5) = hess_J(3,i)*hess_J(4,j)-hess_J(4,i)*hess_J(3,j);
    }
    else
    {
        //h.setSubvector(0, cross(Jo,j,Jl,i));
        h[0] = hess_J(4,j)*hess_J(2,i) - hess_J(5,j)*hess_J(1,i);
        h[1] = hess_J(5,j)*hess_J(0,i) - hess_J(3,j)*hess_J(2,i);
        h[2] = hess_J(3,j)*hess_J(1,i) - hess_J(4,j)*hess_J(0,i);
        h[3]=h[4]=h[5]=0.0;
    }

    return h;
}


/************************************************************************/
Vector iKinChain::Hessian_ij(const unsigned int lnk, const unsigned int i,
                             const unsigned int j)
{
    prepareForHessian(lnk);
    return fastHessian_ij(lnk,i,j);
}


/************************************************************************/
void iKinChain::prepareForHessian(const unsigned int lnk)
{
    if (lnk>=N)
    {
        if (verbose)
            yError("prepareForHessian() failed due to out of range index: %d>=%d",lnk,N);

        return;
    }

    hess_Jlnk=GeoJacobian(lnk);
}


/************************************************************************/
Vector iKinChain::fastHessian_ij(const unsigned int lnk, const unsigned int i,
                                 const unsigned int j)
{
    yAssert((i<lnk) && (j<lnk));

    Vector h(6,0.0);
    if (i<j)
    {
        //h.setSubvector(0,cross(hess_Jlnko,i,hess_Jlnkl,j));
        h[0] = hess_Jlnk(4,i)*hess_Jlnk(2,j) - hess_Jlnk(5,i)*hess_Jlnk(1,j);
        h[1] = hess_Jlnk(5,i)*hess_Jlnk(0,j) - hess_Jlnk(3,i)*hess_Jlnk(2,j);
        h[2] = hess_Jlnk(3,i)*hess_Jlnk(1,j) - hess_Jlnk(4,i)*hess_Jlnk(0,j);
        //h.setSubvector(3,cross(hess_Jlnko,i,hess_Jlnko,j));
        h[3] = hess_Jlnk(4,i)*hess_Jlnk(5,j) - hess_Jlnk(5,i)*hess_Jlnk(4,j);
        h[4] = hess_Jlnk(5,i)*hess_Jlnk(3,j) - hess_Jlnk(3,i)*hess_Jlnk(5,j);
        h[5] = hess_Jlnk(3,i)*hess_Jlnk(4,j) - hess_Jlnk(4,i)*hess_Jlnk(3,j);
    }
    else
    {
        //h.setSubvector(0,cross(hess_Jlnko,j,hess_Jlnkl,i));
        h[0] = hess_Jlnk(4,j)*hess_Jlnk(2,i) - hess_Jlnk(5,j)*hess_Jlnk(1,i);
        h[1] = hess_Jlnk(5,j)*hess_Jlnk(0,i) - hess_Jlnk(3,j)*hess_Jlnk(2,i);
        h[2] = hess_Jlnk(3,j)*hess_Jlnk(1,i) - hess_Jlnk(4,j)*hess_Jlnk(0,i);
        h[3]=h[4]=h[5]=0.0;
    }

    return h;
}


/************************************************************************/
Matrix iKinChain::DJacobian(const Vector &dq)
{
    Matrix J=GeoJacobian();
    Matrix dJ(6,DOF); dJ.zero();
    double dqj,dqi,a,b,c;
    for (unsigned int i=0; i<DOF; i++)  // i: col
    {
        for (unsigned int j=0; j<=i; j++)  // j: row
        {
            dqj=dq[j];
            
            a=J(4,j)*J(2,i)-J(5,j)*J(1,i);
            b=J(5,j)*J(0,i)-J(3,j)*J(2,i);
            c=J(3,j)*J(1,i)-J(4,j)*J(0,i);
            dJ(0,i)+=dqj*a;
            dJ(1,i)+=dqj*b;
            dJ(2,i)+=dqj*c;
            dJ(3,i)+=dqj*(J(4,j)*J(5,i)-J(5,j)*J(4,i));
            dJ(4,i)+=dqj*(J(5,j)*J(3,i)-J(3,j)*J(5,i));
            dJ(5,i)+=dqj*(J(3,j)*J(4,i)-J(4,j)*J(3,i));
            
            if (i!=j)
            {
                dqi     =dq[i];
                dJ(0,j)+=dqi*a;
                dJ(1,j)+=dqi*b;
                dJ(2,j)+=dqi*c;
            }
        }
    }

    return dJ;

    /* OLD IMPLEMENTATION (SLOWER, BUT CLEARER)
    prepareForHessian();
    Vector tmp(6,0.0);
    for (unsigned int i=0; i<DOF; i++)
    {
        for (unsigned int j=0; j<DOF; j++)
            tmp+=fastHessian_ij(j,i)*dq(j);

        dJ.setCol(i,tmp);
        tmp.zero();
    }*/
}


/************************************************************************/
Matrix iKinChain::DJacobian(const unsigned int lnk, const Vector &dq)
{
    Matrix J=GeoJacobian(lnk);
    Matrix dJ(6,lnk-1); dJ.zero();
    double dqj,dqi,a,b,c;
    for (unsigned int i=0; i<lnk; i++)  // i: col
    {
        for (unsigned int j=0; j<=i; j++)  // j: row
        {
            dqj=dq[j];
            
            a=J(4,j)*J(2,i)-J(5,j)*J(1,i);
            b=J(5,j)*J(0,i)-J(3,j)*J(2,i);
            c=J(3,j)*J(1,i)-J(4,j)*J(0,i);
            dJ(0,i)+=dqj*a;
            dJ(1,i)+=dqj*b;
            dJ(2,i)+=dqj*c;
            dJ(3,i)+=dqj*(J(4,j)*J(5,i)-J(5,j)*J(4,i));
            dJ(4,i)+=dqj*(J(5,j)*J(3,i)-J(3,j)*J(5,i));
            dJ(5,i)+=dqj*(J(3,j)*J(4,i)-J(4,j)*J(3,i));
            
            if (i!=j)
            {
                dqi     =dq[i];
                dJ(0,j)+=dqi*a;
                dJ(1,j)+=dqi*b;
                dJ(2,j)+=dqi*c;
            }
        }
    }

    return dJ;

    // OLD IMPLEMENTATION (SLOWER, BUT CLEARER)
    /*prepareForHessian(lnk);
    Vector tmp(6,0.0);
    for (unsigned int i=0; i<lnk; i++)
    {
        for (unsigned int j=0; j<lnk; j++)
            tmp+=fastHessian_ij(lnk,j,i)*dq(j);

        dJ.setCol(i,tmp);
        tmp.zero();
    }*/
}


/************************************************************************/
iKinChain::~iKinChain()
{
    dispose();
}


/************************************************************************/
void iKinChain::dispose()
{
    allList.clear();
    quickList.clear();
}


/************************************************************************/
iKinLimb::iKinLimb()
{
    allocate("right");
}


/************************************************************************/
iKinLimb::iKinLimb(const string &_type)
{
    allocate(_type);
}


/************************************************************************/
iKinLimb::iKinLimb(const iKinLimb &limb)
{
    clone(limb);
}


/************************************************************************/
iKinLimb::iKinLimb(const Property &options)
{
    fromLinksProperties(options);
}


/************************************************************************/
void iKinLimb::pushLink(iKinLink *pl)
{
    linkList.push_back(pl);
    pushLink(*pl);
}


/************************************************************************/
void iKinLimb::getMatrixFromProperties(const Property &options, const string &tag,
                                       Matrix &H)
{
    if (Bottle *bH=options.find(tag).asList())
    {
        int i=0;
        int j=0;

        H.zero();
        for (int cnt=0; (cnt<bH->size()) && (cnt<H.rows()*H.cols()); cnt++)
        {    
            H(i,j)=bH->get(cnt).asDouble();
            if (++j>=H.cols())
            {
                i++;
                j=0;
            }
        }
    }
}


/************************************************************************/
void iKinLimb::setMatrixToProperties(Property &options, const string &tag,
                                     Matrix &H)
{
    Bottle b; Bottle &l=b.addList();
    for (int r=0; r<H.rows(); r++)
        for (int c=0; c<H.cols(); c++)
            l.addDouble(H(r,c));

    options.put(tag,b.get(0));
}


/************************************************************************/
bool iKinLimb::fromLinksProperties(const Property &options)
{
    dispose();    

    type=options.check("type",Value("right")).asString();

    getMatrixFromProperties(options,"H0",H0);
    getMatrixFromProperties(options,"HN",HN);

    int numLinks=options.check("numLinks",Value(0)).asInt();
    if (numLinks==0)
    {
        yError("invalid number of links specified!");

        type="right";
        H0.eye();
        HN.eye();

        return false;
    }

    for (int i=0; i<numLinks; i++)
    {
        ostringstream link;
        link<<"link_"<<i;

        Bottle &bLink=options.findGroup(link.str());
        if (bLink.isNull())
        {
            yError("%s is missing!",link.str().c_str());

            type="right";
            H0.eye();
            dispose();

            return false;
        }

        double A=bLink.check("A",Value(0.0)).asDouble();
        double D=bLink.check("D",Value(0.0)).asDouble();
        double alpha=CTRL_DEG2RAD*bLink.check("alpha",Value(0.0)).asDouble();
        double offset=CTRL_DEG2RAD*bLink.check("offset",Value(0.0)).asDouble();
        double min=CTRL_DEG2RAD*bLink.check("min",Value(-180.0)).asDouble();
        double max=CTRL_DEG2RAD*bLink.check("max",Value(180.0)).asDouble();

        pushLink(new iKinLink(A,D,alpha,offset,min,max));

        if (bLink.check("blocked"))
            blockLink(i,CTRL_DEG2RAD*bLink.find("blocked").asDouble());
    }

    return true;
}


/************************************************************************/
bool iKinLimb::toLinksProperties(Property &options)
{
    Bottle links;
    for (unsigned int i=0; i<N; i++)
    {
        Bottle &link=links.addList();
        ostringstream tag;
        tag<<"link_"<<i;
        link.addString(tag.str());

        Bottle &A=link.addList();
        A.addString("A");
        A.addDouble((*this)[i].getA());

        Bottle &D=link.addList();
        D.addString("D");
        D.addDouble((*this)[i].getD());

        Bottle &alpha=link.addList();
        alpha.addString("alpha");
        alpha.addDouble(CTRL_RAD2DEG*(*this)[i].getAlpha());

        Bottle &offset=link.addList();
        offset.addString("offset");
        offset.addDouble(CTRL_RAD2DEG*(*this)[i].getOffset());

        Bottle &min=link.addList();
        min.addString("min");
        min.addDouble(CTRL_RAD2DEG*(*this)[i].getMin());

        Bottle &max=link.addList();
        max.addString("max");
        max.addDouble(CTRL_RAD2DEG*(*this)[i].getMax());

        if ((*this)[i].isBlocked())
        {
            Bottle &blocked=link.addList();
            blocked.addString("blocked");
            blocked.addDouble(CTRL_RAD2DEG*(*this)[i].getAng());
        }
    }

    links.write(options);

    options.put("type",getType());
    options.put("numLinks",(int)N);

    setMatrixToProperties(options,"H0",H0);
    setMatrixToProperties(options,"HN",HN);

    return true;
}


/************************************************************************/
iKinLimb &iKinLimb::operator=(const iKinLimb &limb)
{
    dispose();
    clone(limb);

    return *this;
}


/************************************************************************/
iKinLimb::~iKinLimb()
{
    dispose();
}


/************************************************************************/
void iKinLimb::allocate(const string &_type)
{
    type=_type;
}


/************************************************************************/
void iKinLimb::clone(const iKinLimb &limb)
{
    type=limb.type;
    H0=limb.H0;
    HN=limb.HN;

    for (unsigned int i=0; i<limb.getN(); i++)
        pushLink(new iKinLink(*(limb.linkList[i])));
}


/************************************************************************/
void iKinLimb::dispose()
{
    for (unsigned int i=0; i<linkList.size(); i++)
        if (linkList[i]!=NULL)
            delete linkList[i];

    linkList.clear();
    iKinChain::dispose();
}


/************************************************************************/
iCubTorso::iCubTorso() : iKinLimb()
{
    allocate("");
}


/************************************************************************/
iCubTorso::iCubTorso(const string &_type) : iKinLimb(string(""))
{
    allocate("");
}


/************************************************************************/
void iCubTorso::allocate(const string &_type)
{
    size_t underscore=getType().find('_');
    if (underscore!=string::npos)
        version=strtod(getType().substr(underscore+2).c_str(),NULL);
    else
        version=1.0;

    Matrix H0(4,4);
    H0.zero();
    H0(3,3)=1.0;

    if (version<3.0)
    {
        H0(0,1)=-1.0;
        H0(1,2)=-1.0;
        H0(2,0)=1.0;
        setH0(H0);

        pushLink(new iKinLink(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.00231, -0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD));
    }
    else
    {
        H0(0,2)=1.0;
        H0(1,1)=-1.0;
        H0(2,0)=1.0;
        setH0(H0);

        pushLink(new iKinLink(     0.0725,      0.0, -M_PI/2.0,       0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(        0.0,      0.0,  M_PI/2.0, -M_PI/2.0, -40.0*CTRL_DEG2RAD, 40.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.00486151, -0.26377, -M_PI/2.0,      M_PI, -20.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
    }
}


/************************************************************************/
bool iCubTorso::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()==0)
        return false;

    IControlLimits &limTorso=*lim[0];

    unsigned int iTorso;
    double min, max;

    for (iTorso=0; iTorso<3; iTorso++)
    {   
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        if (version<3.0)
        {
            (*this)[2-iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[2-iTorso].setMax(CTRL_DEG2RAD*max);
        }
        else
        {
            (*this)[iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[iTorso].setMax(CTRL_DEG2RAD*max);
        }
    }

    return true;
}


/************************************************************************/
iCubArm::iCubArm() : iKinLimb()
{
    allocate("right");
}


/************************************************************************/
iCubArm::iCubArm(const string &_type) : iKinLimb(_type)
{
    allocate(_type);
}


/************************************************************************/
void iCubArm::allocate(const string &_type)
{
    string arm;
    size_t underscore=getType().find('_');
    if (underscore!=string::npos)
    {
        arm=getType().substr(0,underscore);
        version=strtod(getType().substr(underscore+2).c_str(),NULL);
    }
    else
    {
        arm=getType();
        version=1.0;
    }

    Matrix H0(4,4);
    H0.zero();

    if (version<3.0)
    {
        H0(0,1)=-1.0;
        H0(1,2)=-1.0;
        H0(2,0)=1.0;
    }
    else
    {
        H0(0,2)=1.0;
        H0(1,1)=-1.0;
        H0(2,0)=1.0;
    }

    H0(3,3)=1.0;
    setH0(H0);

    if (arm=="right")
    {
        if (version<3.0)
        {
            pushLink(new iKinLink(     0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(-0.0233647,  -0.1433,  M_PI/2.0, -105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0, -0.10774,  M_PI/2.0,           -M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,      0.0, -M_PI/2.0,           -M_PI/2.0,   0.0*CTRL_DEG2RAD, 160.8*CTRL_DEG2RAD));
            pushLink(new iKinLink(    -0.015, -0.15228, -M_PI/2.0, -105.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(     0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
        if (version<1.7)
            pushLink(new iKinLink(       0.0,  -0.1373,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));
        else
            pushLink(new iKinLink(       0.0,  -0.1413,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -65.0*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD));
        if (version<2.0)
            pushLink(new iKinLink(    0.0625,    0.016,       0.0,                M_PI, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
        else
            pushLink(new iKinLink(    0.0625,  0.02598,       0.0,                M_PI, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
        }
        else
        {
            pushLink(new iKinLink(    0.0725,        0.0,              -M_PI/2.0,                 0.0, -20.0*CTRL_DEG2RAD,  20.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,        0.0,               M_PI/2.0,           -M_PI/2.0, -40.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(-0.0304112,  -0.161133, 75.489181*CTRL_DEG2RAD,  165.0*CTRL_DEG2RAD, -20.0*CTRL_DEG2RAD,  60.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,  -0.163978,               M_PI/2.0,           -M_PI/2.0, -90.0*CTRL_DEG2RAD,   2.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,        0.0,              -M_PI/2.0, -104.5*CTRL_DEG2RAD,  -5.0*CTRL_DEG2RAD, 140.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(  -0.01065,    -0.2005,              -M_PI/2.0,   -105*CTRL_DEG2RAD, -45.0*CTRL_DEG2RAD,  80.0*CTRL_DEG2RAD));
            pushLink(new iKinLink( 0.0146593, 0.00308819,               M_PI/2.0,                 0.0,  -5.0*CTRL_DEG2RAD, 110.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(-0.0020075,    -0.1628,               M_PI/2.0,           -M_PI/2.0, -60.0*CTRL_DEG2RAD,  60.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,        0.0,               M_PI/2.0,            M_PI/2.0, -70.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(    0.0625,      0.016,                    0.0,                M_PI, -15.0*CTRL_DEG2RAD,  35.0*CTRL_DEG2RAD));
        }
    }
    else
    {
        if (arm!="left")
            type.replace(0,underscore,"left");

        if (version<3.0)
        {
            pushLink(new iKinLink(     0.032,      0.0,  M_PI/2.0,                 0.0, -22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD)); 
            pushLink(new iKinLink(       0.0,  -0.0055,  M_PI/2.0,           -M_PI/2.0, -39.0*CTRL_DEG2RAD,  39.0*CTRL_DEG2RAD));
            pushLink(new iKinLink( 0.0233647,  -0.1433, -M_PI/2.0,  105.0*CTRL_DEG2RAD, -59.0*CTRL_DEG2RAD,  59.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,  0.10774, -M_PI/2.0,            M_PI/2.0, -95.5*CTRL_DEG2RAD,   5.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,           -M_PI/2.0,   0.0*CTRL_DEG2RAD, 160.8*CTRL_DEG2RAD));
            pushLink(new iKinLink(     0.015,  0.15228, -M_PI/2.0,   75.0*CTRL_DEG2RAD, -37.0*CTRL_DEG2RAD, 100.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(    -0.015,      0.0,  M_PI/2.0,                 0.0,   5.5*CTRL_DEG2RAD, 106.0*CTRL_DEG2RAD));
        if (version<1.7)
            pushLink(new iKinLink(       0.0,   0.1373,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));
        else
            pushLink(new iKinLink(       0.0,   0.1413,  M_PI/2.0,           -M_PI/2.0, -50.0*CTRL_DEG2RAD,  50.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,            M_PI/2.0, -65.0*CTRL_DEG2RAD,  10.0*CTRL_DEG2RAD));
        if (version<2.0)
            pushLink(new iKinLink(    0.0625,   -0.016,       0.0,                 0.0, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
        else
            pushLink(new iKinLink(    0.0625, -0.02598,       0.0,                 0.0, -25.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
        }
        else
        {
            pushLink(new iKinLink(    0.0725,         0.0,                -M_PI/2.0,               0.0, -20.0*CTRL_DEG2RAD,  20.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,         0.0,                 M_PI/2.0,         -M_PI/2.0, -40.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
            pushLink(new iKinLink( 0.0304112,   -0.161133, -104.510819*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD, -20.0*CTRL_DEG2RAD,  60.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,    0.163978,                 M_PI/2.0,         -M_PI/2.0, -90.0*CTRL_DEG2RAD,   2.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,         0.0,                 M_PI/2.0, 75.5*CTRL_DEG2RAD,  -5.0*CTRL_DEG2RAD, 140.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(   0.01065,      0.2005,                -M_PI/2.0, 75.0*CTRL_DEG2RAD, -45.0*CTRL_DEG2RAD,  80.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(-0.0146593, -0.00308819,                 M_PI/2.0,               0.0,  -5.0*CTRL_DEG2RAD, 110.0*CTRL_DEG2RAD));
            pushLink(new iKinLink( 0.0020075,      0.1628,                 M_PI/2.0,         -M_PI/2.0, -60.0*CTRL_DEG2RAD,  60.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(       0.0,         0.0,                 M_PI/2.0,          M_PI/2.0, -70.0*CTRL_DEG2RAD,  25.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(    0.0625,      -0.016,                      0.0,               0.0, -15.0*CTRL_DEG2RAD,  35.0*CTRL_DEG2RAD));
        }
    }

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);
}


/************************************************************************/
bool iCubArm::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<2)
        return false;

    IControlLimits &limTorso=*lim[0];
    IControlLimits &limArm  =*lim[1];

    unsigned int iTorso;
    unsigned int iArm;
    double min, max;

    for (iTorso=0; iTorso<3; iTorso++)
    {   
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        if (version<3.0)
        {
            (*this)[2-iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[2-iTorso].setMax(CTRL_DEG2RAD*max);
        }
        else
        {
            (*this)[iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[iTorso].setMax(CTRL_DEG2RAD*max);
        }
    }

    for (iArm=0; iArm<getN()-iTorso; iArm++)
    {   
        if (!limArm.getLimits(iArm,&min,&max))
            return false;

        (*this)[iTorso+iArm].setMin(CTRL_DEG2RAD*min);
        (*this)[iTorso+iArm].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}


/************************************************************************/
iCubFinger::iCubFinger() : iKinLimb()
{
    allocate("right_index");
}


/************************************************************************/
iCubFinger::iCubFinger(const string &_type) : iKinLimb(_type)
{
    allocate(_type);
}


/************************************************************************/
iCubFinger::iCubFinger(const iCubFinger &finger) : iKinLimb(finger)
{
    clone(finger);
}


/************************************************************************/
void iCubFinger::clone(const iCubFinger &finger)
{
    this->hand=finger.hand;
    this->finger=finger.finger;
    this->version=finger.version;
}


/************************************************************************/
iCubFinger &iCubFinger::operator=(const iCubFinger &finger)
{
    dispose();
    iKinLimb::clone(finger);
    clone(finger);

    return *this;
}


/************************************************************************/
void iCubFinger::allocate(const string &_type)
{
    size_t underscore=getType().find('_');
    if (underscore!=string::npos)
    {
        hand=getType().substr(0,underscore);
        finger=getType().substr(underscore+1,getType().length()-underscore-1);

        underscore=finger.find('_');
        if (underscore!=string::npos)
        {
            version=finger.substr(underscore+1,finger.length()-underscore-1);
            finger=finger.substr(0,underscore);
        }
        else
            version="na";
    }
    else
    {        
        hand="right";
        finger="index";
        version="na";
    }

    type=hand+"_"+finger+"_"+version;

    // reinforce hand info
    if (hand!="left")
        hand="right";

    Matrix H0(4,4);
    if (finger=="thumb")
    {
        if (version=="a")
        {
            H0(0,0)=0.121132;  H0(0,1)=0.043736; H0(0,2)=-0.991672; H0(0,3)=-0.025391770;
            H0(1,0)=-0.958978; H0(1,1)=0.263104; H0(1,2)=-0.105535; H0(1,3)=-0.011783901;
            H0(2,0)=0.256297;  H0(2,1)=0.963776; H0(2,2)=0.073812;  H0(2,3)=-0.0017018;
            H0(3,0)=0.0;       H0(3,1)=0.0;      H0(3,2)=0.0;       H0(3,3)=1.0;
        }
        else
        {
            version="b";
            H0(0,0)=0.478469;  H0(0,1)=0.063689; H0(0,2)=-0.875792; H0(0,3)=-0.024029759;
            H0(1,0)=-0.878095; H0(1,1)=0.039246; H0(1,2)=-0.476873; H0(1,3)=-0.01193433;
            H0(2,0)=0.004;     H0(2,1)=0.997198; H0(2,2)=0.074703;  H0(2,3)=-0.00168926;
            H0(3,0)=0.0;       H0(3,1)=0.0;      H0(3,2)=0.0;       H0(3,3)=1.0;
        }

        if (hand=="left")
        {
            H0(2,1)=-H0(2,1);
            H0(0,2)=-H0(0,2);
            H0(1,2)=-H0(1,2);            
            H0(2,3)=-H0(2,3);

            pushLink(new iKinLink(   0.0,     0.0,  M_PI/2.0, 0.0, 10.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(0.0210, -0.0056,       0.0, 0.0,  0.0*CTRL_DEG2RAD,  0.0*CTRL_DEG2RAD));
        }
        else
        {
            pushLink(new iKinLink(   0.0,     0.0, -M_PI/2.0, 0.0, 10.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(0.0210,  0.0056,       0.0, 0.0,  0.0*CTRL_DEG2RAD,  0.0*CTRL_DEG2RAD));
        }

        pushLink(new iKinLink(0.0260, 0.0,       0.0, 0.0, 0.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0220, 0.0,       0.0, 0.0, 0.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0168, 0.0, -M_PI/2.0, 0.0, 0.0*CTRL_DEG2RAD, 90.0*CTRL_DEG2RAD));

        // this is a dummy link
        blockLink(1,0.0);
    }
    else if (finger=="index")
    {
        H0(0,0)=0.898138; H0(0,1)=0.439714;  H0(0,2)=0.0;      H0(0,3)=0.00245549;
        H0(1,0)=-0.43804; H0(1,1)=0.89472;   H0(1,2)=0.087156; H0(1,3)=-0.025320433;
        H0(2,0)=0.038324; H0(2,1)=-0.078278; H0(2,2)=0.996195; H0(2,3)=-0.010973325;
        H0(3,0)=0.0;      H0(3,1)=0.0;       H0(3,2)=0.0;      H0(3,3)=1.0;

        if (hand=="left")
        {
            H0(2,0)=-H0(2,0);
            H0(2,1)=-H0(2,1);
            H0(1,2)=-H0(1,2);
            H0(2,3)=-H0(2,3);

            pushLink(new iKinLink(0.0148, 0.0, -M_PI/2.0, 0.0, 0.0, 20.0*CTRL_DEG2RAD));
        }
        else
            pushLink(new iKinLink(0.0148, 0.0, M_PI/2.0, 0.0, 0.0, 20.0*CTRL_DEG2RAD));

        pushLink(new iKinLink(0.0259, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0220, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0168, 0.0, -M_PI/2.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
    }
    else if (finger=="middle")
    {
        H0(0,0)=1.0; H0(0,1)=0.0; H0(0,2)=0.0;  H0(0,3)=0.0178;
        H0(1,0)=0.0; H0(1,1)=0.0; H0(1,2)=-1.0; H0(1,3)=-0.00830233;
        H0(2,0)=0.0; H0(2,1)=1.0; H0(2,2)=0.0;  H0(2,3)=-0.0118;
        H0(3,0)=0.0; H0(3,1)=0.0; H0(3,2)=0.0;  H0(3,3)=1.0;

        if (hand=="left")
        {
            H0(2,1)=-H0(2,1);
            H0(1,2)=-H0(1,2);            
            H0(2,3)=-H0(2,3);
        }

        pushLink(new iKinLink(0.0285, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0240, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0168, 0.0, -M_PI/2.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
    }
    else if (finger=="ring")
    {
        H0(0,0)=0.9998; H0(0,1)=0.0192;  H0(0,2)=0.0;     H0(0,3)=0.001569230;
        H0(1,0)=0.0191; H0(1,1)=-0.9960; H0(1,2)=0.0872;  H0(1,3)=0.007158757;
        H0(2,0)=0.0017; H0(2,1)=-0.0871; H0(2,2)=-0.9962; H0(2,3)=-0.011458935;
        H0(3,0)=0.0;    H0(3,1)=0.0;     H0(3,2)=0.0;     H0(3,3)=1.0;

        if (hand=="left")
        {
            H0(2,0)=-H0(2,0);
            H0(2,1)=-H0(2,1);
            H0(1,2)=-H0(1,2);
            H0(2,3)=-H0(2,3);

            pushLink(new iKinLink(0.0148, 0.0, M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));
        }
        else
            pushLink(new iKinLink(0.0148, 0.0, -M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));

        pushLink(new iKinLink(0.0259, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0220, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0168, 0.0, -M_PI/2.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
    }
    else if (finger=="little")
    {
        H0(0,0)=0.9998; H0(0,1)=0.0192;  H0(0,2)=0.0;     H0(0,3)=-0.00042147;
        H0(1,0)=0.0191; H0(1,1)=-0.9960; H0(1,2)=0.0872;  H0(1,3)=0.0232755;
        H0(2,0)=0.0017; H0(2,1)=-0.0871; H0(2,2)=-0.9962; H0(2,3)=-0.00956329;
        H0(3,0)=0.0;    H0(3,1)=0.0;     H0(3,2)=0.0;     H0(3,3)=1.0;

        if (hand=="left")
        {
            H0(2,0)=-H0(2,0);
            H0(2,1)=-H0(2,1);
            H0(1,2)=-H0(1,2);
            H0(2,3)=-H0(2,3);

            pushLink(new iKinLink(0.0148, 0.0, M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));
        }
        else
            pushLink(new iKinLink(0.0148, 0.0, -M_PI/2.0, -20.0*CTRL_DEG2RAD, 0.0, 20.0*CTRL_DEG2RAD));

        pushLink(new iKinLink(0.0219, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0190, 0.0,       0.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.0168, 0.0, -M_PI/2.0, 0.0, 0.0, 90.0*CTRL_DEG2RAD));
    }

    setH0(H0);
}


/************************************************************************/
bool iCubFinger::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limFinger=*lim[0];
    double min, max;

    if (finger=="thumb")
    {
        if (!limFinger.getLimits(8,&min,&max))
            return false;

        (*this)[0].setMin(CTRL_DEG2RAD*min);
        (*this)[0].setMax(CTRL_DEG2RAD*max);

        if (!limFinger.getLimits(9,&min,&max))
            return false;

        (*this)[2].setMin(CTRL_DEG2RAD*min);
        (*this)[2].setMax(CTRL_DEG2RAD*max);

        if (!limFinger.getLimits(10,&min,&max))
            return false;

        (*this)[3].setMin(CTRL_DEG2RAD*min);
        (*this)[3].setMax(CTRL_DEG2RAD*max/2.0);
        (*this)[4].setMin(CTRL_DEG2RAD*min);
        (*this)[4].setMax(CTRL_DEG2RAD*max/2.0);
    }
    else if (finger=="index")
    {
        if (!limFinger.getLimits(7,&min,&max))
            return false;

        (*this)[0].setMin(CTRL_DEG2RAD*min);
        (*this)[0].setMax(CTRL_DEG2RAD*max/3.0);

        if (!limFinger.getLimits(11,&min,&max))
            return false;

        (*this)[1].setMin(CTRL_DEG2RAD*min);
        (*this)[1].setMax(CTRL_DEG2RAD*max);

        if (!limFinger.getLimits(12,&min,&max))
            return false;

        (*this)[2].setMin(CTRL_DEG2RAD*min);
        (*this)[2].setMax(CTRL_DEG2RAD*max/2.0);
        (*this)[3].setMin(CTRL_DEG2RAD*min);
        (*this)[3].setMax(CTRL_DEG2RAD*max/2.0);
    }
    else if (finger=="middle")
    {
        if (!limFinger.getLimits(13,&min,&max))
            return false;

        (*this)[0].setMin(CTRL_DEG2RAD*min);
        (*this)[0].setMax(CTRL_DEG2RAD*max);

        if (!limFinger.getLimits(14,&min,&max))
            return false;

        (*this)[1].setMin(CTRL_DEG2RAD*min);
        (*this)[1].setMax(CTRL_DEG2RAD*max/2.0);
        (*this)[2].setMin(CTRL_DEG2RAD*min);
        (*this)[2].setMax(CTRL_DEG2RAD*max/2.0);
    }
    else if ((finger=="ring") || (finger=="little"))
    {
        if (!limFinger.getLimits(7,&min,&max))
            return false;

        (*this)[0].setMin(CTRL_DEG2RAD*min);
        (*this)[0].setMax(CTRL_DEG2RAD*max/3.0);

        if (!limFinger.getLimits(15,&min,&max))
            return false;

        (*this)[1].setMin(CTRL_DEG2RAD*min);
        (*this)[1].setMax(CTRL_DEG2RAD*max/3.0);
        (*this)[2].setMin(CTRL_DEG2RAD*min);
        (*this)[2].setMax(CTRL_DEG2RAD*max/3.0);
        (*this)[3].setMin(CTRL_DEG2RAD*min);
        (*this)[3].setMax(CTRL_DEG2RAD*max/3.0);
    }

    return true;
}


/************************************************************************/
bool iCubFinger::getChainJoints(const Vector &motorEncoders,
                                Vector &chainJoints)
{
    if ((motorEncoders.length()!=9) && (motorEncoders.length()!=16))
        return false;

    int offs=(motorEncoders.length()==16?7:0);
    
    if (finger=="thumb")
    {
        chainJoints.resize(4);
        chainJoints[0]=motorEncoders[offs+1];
        chainJoints[1]=motorEncoders[offs+2];
        chainJoints[2]=motorEncoders[offs+3]/2.0;
        chainJoints[3]=chainJoints[2];
    }
    else if (finger=="index")
    {
        chainJoints.resize(4);
        chainJoints[0]=motorEncoders[offs+0]/3.0;
        chainJoints[1]=motorEncoders[offs+4];
        chainJoints[2]=motorEncoders[offs+5]/2.0;
        chainJoints[3]=chainJoints[2];
    }
    else if (finger=="middle")
    {
        chainJoints.resize(3);
        chainJoints[0]=motorEncoders[offs+6];
        chainJoints[1]=motorEncoders[offs+7]/2.0;
        chainJoints[2]=chainJoints[1];
    }
    else if ((finger=="ring") || (finger=="little"))
    {
        chainJoints.resize(4);
        chainJoints[0]=motorEncoders[offs+0]/3.0;
        chainJoints[1]=motorEncoders[offs+8]/3.0;
        chainJoints[3]=chainJoints[2]=chainJoints[1];
    }
    else
        return false;

    return true;
}


/************************************************************************/
bool iCubFinger::getChainJoints(const Vector &motorEncoders,
                                const Vector &jointEncoders,
                                Vector &chainJoints,
                                const Matrix &jointEncodersBounds)
{
    if (((motorEncoders.length()!=9) && (motorEncoders.length()!=16)) ||
        (jointEncoders.length()<15) || (jointEncodersBounds.cols()<2))
        return false;

    int offs=(motorEncoders.length()==16?7:0);

    Matrix bounds=jointEncodersBounds;
    if (bounds.rows()!=jointEncoders.length())
    {
        bounds=zeros((unsigned int)jointEncoders.length(),2);
        for (size_t r=0; r<jointEncoders.length(); r++)
            bounds(r,0)=255.0;
    }
    
    if (finger=="thumb")
    {
        chainJoints.resize(4);
        chainJoints[0]=motorEncoders[offs+1];
        for (unsigned int i=1; i<chainJoints.length(); i++)
        {
            double c=0.0;
            double span=bounds(i-1,1)-bounds(i-1,0);
            if (span>0.0)
                c=std::min(1.0,std::max(0.0,(jointEncoders[i-1]-bounds(i-1,0))/span));
            else if (span<0.0)
                c=1.0-std::min(1.0,std::max(0.0,(bounds(i-1,1)-jointEncoders[i-1])/span));
            chainJoints[i]=CTRL_RAD2DEG*(c*((*this)(i).getMax()-(*this)(i).getMin())+(*this)(i).getMin());
        }
    }
    else if (finger=="index")
    {
        chainJoints.resize(4);
        chainJoints[0]=motorEncoders[offs+0]/3.0;
        for (unsigned int i=1; i<chainJoints.length(); i++)
        {
            double c=0.0;
            double span=bounds(i+2,1)-bounds(i+2,0);
            if (span>0.0)
                c=std::min(1.0,std::max(0.0,(jointEncoders[i+2]-bounds(i+2,0))/span));
            else if (span<0.0)
                c=1.0-std::min(1.0,std::max(0.0,(bounds(i+2,1)-jointEncoders[i+2])/span));
            chainJoints[i]=CTRL_RAD2DEG*(c*((*this)(i).getMax()-(*this)(i).getMin())+(*this)(i).getMin());
        }
    }
    else if (finger=="middle")
    {
        chainJoints.resize(3);
        for (unsigned int i=0; i<chainJoints.length(); i++)
        {
            double c=0.0;
            double span=bounds(i+6,1)-bounds(i+6,0);
            if (span>0.0)
                c=std::min(1.0,std::max(0.0,(jointEncoders[i+6]-bounds(i+6,0))/span));
            else if (span<0.0)
                c=1.0-std::min(1.0,std::max(0.0,(bounds(i+6,1)-jointEncoders[i+6])/span));
            chainJoints[i]=CTRL_RAD2DEG*(c*((*this)(i).getMax()-(*this)(i).getMin())+(*this)(i).getMin());
        }
    }
    else if (finger=="ring")
    {
        chainJoints.resize(4);
        chainJoints[0]=motorEncoders[offs+0]/3.0;
        for (unsigned int i=1; i<chainJoints.length(); i++)
        {
            double c=0.0;
            double span=bounds(i+8,1)-bounds(i+8,0);
            if (span>0.0)
                c=std::min(1.0,std::max(0.0,(jointEncoders[i+8]-bounds(i+8,0))/span));
            else if (span<0.0)
                c=1.0-std::min(1.0,std::max(0.0,(bounds(i+8,1)-jointEncoders[i+8])/span));
            chainJoints[i]=CTRL_RAD2DEG*(c*((*this)(i).getMax()-(*this)(i).getMin())+(*this)(i).getMin());
        }
    }
    else if (finger=="little")
    {
        chainJoints.resize(4);
        chainJoints[0]=motorEncoders[offs+0]/3.0;
        for (unsigned int i=1; i<chainJoints.length(); i++)
        {
            double c=0.0;
            double span=bounds(i+11,1)-bounds(i+11,0);
            if (span>0.0)
                c=std::min(1.0,std::max(0.0,(jointEncoders[i+11]-bounds(i+11,0))/span));
            else if (span<0.0)
                c=1.0-std::min(1.0,std::max(0.0,(bounds(i+11,1)-jointEncoders[i+11])/span));
            chainJoints[i]=CTRL_RAD2DEG*(c*((*this)(i).getMax()-(*this)(i).getMin())+(*this)(i).getMin());
        }
    }
    else
        return false;

    return true;
}


/************************************************************************/
iCubLeg::iCubLeg() : iKinLimb()
{
    allocate("right");
}


/************************************************************************/
iCubLeg::iCubLeg(const string &_type) : iKinLimb(_type)
{
    allocate(_type);
}


/************************************************************************/
void iCubLeg::allocate(const string &_type)
{
    Matrix H0(4,4);
    H0.zero();
    H0(0,0)=1.0;
    H0(1,2)=1.0;
    H0(2,1)=-1.0;
    H0(2,3)=-0.1199;
    H0(3,3)=1.0;

    if ((getType()=="right") || (getType()=="right_v1"))
    {
        H0(1,3)=0.0681;

        pushLink(new iKinLink(   0.0,     0.0,  M_PI/2.0,  M_PI/2.0,  -44.0*CTRL_DEG2RAD, 132.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,     0.0,  M_PI/2.0,  M_PI/2.0, -119.0*CTRL_DEG2RAD,  17.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,  0.2236, -M_PI/2.0, -M_PI/2.0,  -79.0*CTRL_DEG2RAD,  79.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.213,     0.0,      M_PI,  M_PI/2.0, -125.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,     0.0,  M_PI/2.0,       0.0,  -42.0*CTRL_DEG2RAD,  21.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.041,     0.0,      M_PI,       0.0,  -24.0*CTRL_DEG2RAD,  24.0*CTRL_DEG2RAD));
    }
    else if ((getType()=="left") || (getType()=="left_v1"))
    {
        H0(1,3)=-0.0681;

        pushLink(new iKinLink(   0.0,     0.0, -M_PI/2.0,  M_PI/2.0,  -44.0*CTRL_DEG2RAD, 132.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,     0.0, -M_PI/2.0,  M_PI/2.0, -119.0*CTRL_DEG2RAD,  17.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0, -0.2236,  M_PI/2.0, -M_PI/2.0,  -79.0*CTRL_DEG2RAD,  79.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.213,     0.0,      M_PI,  M_PI/2.0, -125.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   0.0,     0.0, -M_PI/2.0,       0.0,  -42.0*CTRL_DEG2RAD,  21.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.041,     0.0,       0.0,       0.0,  -24.0*CTRL_DEG2RAD,  24.0*CTRL_DEG2RAD));
    }
    else if (getType()=="right_v2.5")
    {
        H0(1,3)=0.0681;

        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,  M_PI/2.0,  -44.0*CTRL_DEG2RAD, 132.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,  M_PI/2.0, -119.0*CTRL_DEG2RAD,  17.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.0009175, 0.234545, -M_PI/2.0, -M_PI/2.0,  -79.0*CTRL_DEG2RAD,  79.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   -0.2005,      0.0,      M_PI,  M_PI/2.0, -125.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,      0.0,  M_PI/2.0,       0.0,  -42.0*CTRL_DEG2RAD,  21.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(  -0.06805,   0.0035,      M_PI,       0.0,  -24.0*CTRL_DEG2RAD,  24.0*CTRL_DEG2RAD));
    }
    else if (getType()=="right_v3")
    {
        H0.zero();
        H0(0,0)=1.0;
        H0(1,2)=1.0;
        H0(2,1)=-1.0;
        H0(0,3)=-0.0216;
        H0(1,3)=0.07365;
        H0(2,3)=-0.0605;
        H0(3,3)=1.0;

        pushLink(new iKinLink(    0.009,    0.0,  M_PI/2.0,  M_PI/2.0,  -40.0*CTRL_DEG2RAD, 125.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   -0.015, 0.0106,  M_PI/2.0,  M_PI/2.0,    5.0*CTRL_DEG2RAD, 111.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.015, 0.2668, -M_PI/2.0, -M_PI/2.0,  -62.5*CTRL_DEG2RAD,  62.5*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.255988, 0.0062,      M_PI,  M_PI/2.0, -100.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   -0.035,    0.0,  M_PI/2.0,       0.0,  -40.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(  -0.0532,    0.0,      M_PI,       0.0,  -20.0*CTRL_DEG2RAD,  20.0*CTRL_DEG2RAD));
    }
    else if (getType()=="left_v3")
    {
        H0.zero();
        H0(0,0)=1.0;
        H0(1,2)=1.0;
        H0(2,1)=-1.0;
        H0(0,3)=-0.0216;
        H0(1,3)=-0.07365;
        H0(2,3)=-0.0605;
        H0(3,3)=1.0;

        pushLink(new iKinLink(    0.009,     0.0, -M_PI/2.0,  M_PI/2.0,  -40.0*CTRL_DEG2RAD, 125.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   -0.015, -0.0106, -M_PI/2.0,  M_PI/2.0,    5.0*CTRL_DEG2RAD, 111.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.015, -0.2668,  M_PI/2.0, -M_PI/2.0,  -62.5*CTRL_DEG2RAD,  62.5*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.255988,  0.0062,      M_PI,  M_PI/2.0, -100.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   -0.035,     0.0, -M_PI/2.0,       0.0,  -40.0*CTRL_DEG2RAD,  40.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(  -0.0532,     0.0,       0.0,       0.0,  -20.0*CTRL_DEG2RAD,  20.0*CTRL_DEG2RAD));
    }
    else
    {
        type="left_v2.5";
        H0(1,3)=-0.0681;

        pushLink(new iKinLink(       0.0,       0.0, -M_PI/2.0,  M_PI/2.0,  -44.0*CTRL_DEG2RAD, 132.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,       0.0, -M_PI/2.0,  M_PI/2.0, -119.0*CTRL_DEG2RAD,  17.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.0009175, -0.234545,  M_PI/2.0, -M_PI/2.0,  -79.0*CTRL_DEG2RAD,  79.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(   -0.2005,       0.0,      M_PI,  M_PI/2.0, -125.0*CTRL_DEG2RAD,   0.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(       0.0,       0.0, -M_PI/2.0,       0.0,  -42.0*CTRL_DEG2RAD,  21.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(  -0.06805,   -0.0035,       0.0,       0.0,  -24.0*CTRL_DEG2RAD,  24.0*CTRL_DEG2RAD));
    }

    setH0(H0);

    size_t underscore=getType().find('_');
    if (underscore!=string::npos)
        version=strtod(getType().substr(underscore+2).c_str(),NULL);
    else
        version=1.0;
}


/************************************************************************/
bool iCubLeg::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<1)
        return false;

    IControlLimits &limLeg=*lim[0];

    unsigned int iLeg;
    double min, max;

    for (iLeg=0; iLeg<getN(); iLeg++)
    {   
        if (!limLeg.getLimits(iLeg,&min,&max))
            return false;

        (*this)[iLeg].setMin(CTRL_DEG2RAD*min);
        (*this)[iLeg].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}


/************************************************************************/
iCubEye::iCubEye() : iKinLimb()
{
    allocate("right");
}


/************************************************************************/
iCubEye::iCubEye(const string &_type) : iKinLimb(_type)
{
    allocate(_type);
}


/************************************************************************/
void iCubEye::allocate(const string &_type)
{
    Matrix H0(4,4);
    H0.zero();
    H0(0,1)=-1.0;
    H0(1,2)=-1.0;
    H0(2,0)=1.0;
    H0(3,3)=1.0;
    setH0(H0);

    if ((getType()=="right") || (getType()=="right_v1"))
    {
        pushLink(new iKinLink(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.00231, -0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(  0.033,     0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,   0.001, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( -0.054,  0.0825, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,   0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,     0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD));
    }
    else if ((getType()=="left") || (getType()=="left_v1"))
    {
        pushLink(new iKinLink(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.00231, -0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(  0.033,     0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,   0.001, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( -0.054,  0.0825, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,  -0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,     0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD));
    }
    else if ((getType()=="right_v2") || (getType()=="right_v2.5"))
    {
        pushLink(new iKinLink(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.2233, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( 0.0095,     0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,     0.0, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.0509, 0.08205, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,   0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,     0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD));
    }
    else if ((getType()=="left_v3") || (getType()=="right_v3"))
    {
        H0(0,2)=1.0;
        H0(1,1)= -1.0;
        H0(2,0)=1.0;
        setH0(H0);

        pushLink(new iKinLink(     0.0725,      0.0, -M_PI/2.0,       0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(        0.0,      0.0,  M_PI/2.0, -M_PI/2.0, -40.0*CTRL_DEG2RAD, 40.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.00486151, -0.26377, -M_PI/2.0,      M_PI, -20.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(     0.0095,      0.0,  M_PI/2.0,  M_PI/2.0, -30.0*CTRL_DEG2RAD, 22.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(        0.0,      0.0, -M_PI/2.0, -M_PI/2.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    -0.0509,  0.08205, -M_PI/2.0,  M_PI/2.0, -45.0*CTRL_DEG2RAD, 45.0*CTRL_DEG2RAD));

        if (getType()=="left_v3")
        {
            pushLink(new iKinLink(0.0,  0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD));
        }
        else
        {
            pushLink(new iKinLink(0.0, -0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD));
            pushLink(new iKinLink(0.0,    0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD));
        }
    }
    else
    {
        if ((type!="left_v2") && (type!="left_v2.5"))
            type="left_v2";

        pushLink(new iKinLink(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.2233, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( 0.0095,     0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,     0.0, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.0509, 0.08205, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,  -0.034, -M_PI/2.0,       0.0, -35.0*CTRL_DEG2RAD, 15.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,     0.0,  M_PI/2.0, -M_PI/2.0, -50.0*CTRL_DEG2RAD, 50.0*CTRL_DEG2RAD));
    }

    blockLink(0,0.0);
    blockLink(1,0.0);
    blockLink(2,0.0);

    size_t underscore=getType().find('_');
    if (underscore!=string::npos)
        version=strtod(getType().substr(underscore+2).c_str(),NULL);
    else
        version=1.0;
}


/************************************************************************/
bool iCubEye::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<2)
        return false;

    IControlLimits &limTorso=*lim[0];
    IControlLimits &limHead =*lim[1];

    unsigned int iTorso;
    unsigned int iHead;
    double min, max;

    for (iTorso=0; iTorso<3; iTorso++)
    {   
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        if (version<3.0)
        {
            (*this)[2-iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[2-iTorso].setMax(CTRL_DEG2RAD*max);
        }
        else
        {
            (*this)[iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[iTorso].setMax(CTRL_DEG2RAD*max);
        }
    }

    for (iHead=0; iHead<getN()-iTorso; iHead++)
    {   
        if (!limHead.getLimits(iHead,&min,&max))
            return false;

        (*this)[iTorso+iHead].setMin(CTRL_DEG2RAD*min);
        (*this)[iTorso+iHead].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}


/************************************************************************/
iCubEyeNeckRef::iCubEyeNeckRef() : iCubEye()
{
    allocate("right");
}


/************************************************************************/
iCubEyeNeckRef::iCubEyeNeckRef(const string &_type) : iCubEye(_type)
{
    allocate(_type);
}


/************************************************************************/
void iCubEyeNeckRef::allocate(const string &_type)
{
    rmLink(0);
    rmLink(0);
    rmLink(0);

    delete linkList[0];
    delete linkList[1];
    delete linkList[2];

    linkList.erase(linkList.begin(),linkList.begin()+2);
}


/************************************************************************/
iCubHeadCenter::iCubHeadCenter() : iCubEye()
{
    allocate("right");
}


/************************************************************************/
iCubHeadCenter::iCubHeadCenter(const string &_type) : iCubEye(_type)
{
    allocate(_type);
}


/************************************************************************/
void iCubHeadCenter::allocate(const string &_type)
{
    // change DH parameters
    (*this)[getN()-2].setD(0.0);

    // block last two links
    blockLink(getN()-2,0.0);
    blockLink(getN()-1,0.0);
}


/************************************************************************/
iCubInertialSensor::iCubInertialSensor() : iKinLimb()
{
    allocate("v1");
}


/************************************************************************/
iCubInertialSensor::iCubInertialSensor(const string &_type) : iKinLimb(_type)
{
    allocate(_type);
}


/************************************************************************/
void iCubInertialSensor::allocate(const string &_type)
{
    Matrix H0(4,4);
    H0.zero();
    H0(3,3)=1.0;

    if ((getType()=="v2") || (getType()=="v2.5"))
    {
        H0(0,1)=-1.0;
        H0(1,2)=-1.0;
        H0(2,0)=1.0;

        pushLink(new iKinLink(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.2233, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( 0.0095,     0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,     0.0, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( 0.0185,  0.1108, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));    
    }
    else if (getType()=="v3")
    {
        H0(0,2)=1.0;
        H0(1,1)=-1.0;
        H0(2,0)=1.0;

        pushLink(new iKinLink(     0.0725,      0.0, -M_PI/2.0,       0.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(        0.0,      0.0,  M_PI/2.0, -M_PI/2.0, -40.0*CTRL_DEG2RAD, 40.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(-0.00486151, -0.26377, -M_PI/2.0,      M_PI, -20.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(     0.0095,      0.0,  M_PI/2.0,  M_PI/2.0, -30.0*CTRL_DEG2RAD, 22.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(        0.0,      0.0, -M_PI/2.0, -M_PI/2.0, -20.0*CTRL_DEG2RAD, 20.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(     0.0185,   0.1108, -M_PI/2.0,  M_PI/2.0, -45.0*CTRL_DEG2RAD, 45.0*CTRL_DEG2RAD));
    }
    else
    {
        H0(0,1)=-1.0;
        H0(1,2)=-1.0;
        H0(2,0)=1.0;

        type="v1";
        pushLink(new iKinLink(  0.032,     0.0,  M_PI/2.0,       0.0, -22.0*CTRL_DEG2RAD, 84.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0, -0.0055,  M_PI/2.0, -M_PI/2.0, -39.0*CTRL_DEG2RAD, 39.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(0.00231, -0.1933, -M_PI/2.0, -M_PI/2.0, -59.0*CTRL_DEG2RAD, 59.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(  0.033,     0.0,  M_PI/2.0,  M_PI/2.0, -40.0*CTRL_DEG2RAD, 30.0*CTRL_DEG2RAD));
        pushLink(new iKinLink(    0.0,   0.001, -M_PI/2.0, -M_PI/2.0, -70.0*CTRL_DEG2RAD, 60.0*CTRL_DEG2RAD));
        pushLink(new iKinLink( 0.0225,  0.1005, -M_PI/2.0,  M_PI/2.0, -55.0*CTRL_DEG2RAD, 55.0*CTRL_DEG2RAD));
    }

    setH0(H0);

    // T_nls (see http://wiki.icub.org/wiki/ICubInertiaSensorKinematics )
    Matrix HN(4,4);
    HN.zero();
    HN(0,0)=1.0;
    HN(2,1)=1.0;
    HN(1,2)=-1.0;
    HN(2,3)=0.0066;
    HN(3,3)=1.0;
    setHN(HN);

    if ((getType()=="v2.5") || (getType()=="v3"))
    {
        Matrix HN=eye(4,4);
        HN(0,3)=0.0087;
        HN(1,3)=0.01795;
        HN(2,3)=-0.0105;
        setHN(getHN()*HN);
    }

    size_t underscore=getType().find('_');
    if (underscore!=string::npos)
        version=strtod(getType().substr(underscore+2).c_str(),NULL);
    else
        version=1.0;
}


/************************************************************************/
bool iCubInertialSensor::alignJointsBounds(const deque<IControlLimits*> &lim)
{
    if (lim.size()<2)
        return false;

    IControlLimits &limTorso=*lim[0];
    IControlLimits &limHead =*lim[1];

    unsigned int iTorso;
    unsigned int iHead;
    double min, max;

    for (iTorso=0; iTorso<3; iTorso++)
    {   
        if (!limTorso.getLimits(iTorso,&min,&max))
            return false;

        if (version<3.0)
        {
            (*this)[2-iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[2-iTorso].setMax(CTRL_DEG2RAD*max);
        }
        else
        {
            (*this)[iTorso].setMin(CTRL_DEG2RAD*min);
            (*this)[iTorso].setMax(CTRL_DEG2RAD*max);
        }
    }

    // only the neck
    for (iHead=0; iHead<3; iHead++)
    {   
        if (!limHead.getLimits(iHead,&min,&max))
            return false;

        (*this)[iTorso+iHead].setMin(CTRL_DEG2RAD*min);
        (*this)[iTorso+iHead].setMax(CTRL_DEG2RAD*max);
    }

    return true;
}


