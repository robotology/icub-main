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

#include <typeinfo>
#include <stdio.h>
#include <algorithm>

#include <gsl/gsl_math.h>

#include <yarp/os/Time.h>
#include <yarp/math/SVD.h>
#include <iCub/iKin/iKinInv.h>

#define IKINCTRL_INTARGET_TOL       5e-3
#define IKINCTRL_WATCHDOG_TOL       1e-4
#define IKINCTRL_WATCHDOG_MAXITER   200

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
iKinCtrl::iKinCtrl(iKinChain &c, unsigned int _ctrlPose) : chain(c)
{
    dim=chain.getDOF();

    grad.resize(dim,0.0);

    q_old=q=chain.getAng();

    inTargetTol    =IKINCTRL_INTARGET_TOL;
    watchDogTol    =IKINCTRL_WATCHDOG_TOL;
    watchDogMaxIter=IKINCTRL_WATCHDOG_MAXITER;

    iter=0;

    state=IKINCTRL_STATE_RUNNING;

    set_ctrlPose(_ctrlPose);

    watchDogOn =false;
    watchDogCnt=0;

    e.resize(6);
    x_set.resize(7,0.0);
    x=chain.EndEffPose(q);

    calc_e();
}


/************************************************************************/
void iKinCtrl::set_ctrlPose(unsigned int _ctrlPose)
{
    ctrlPose=_ctrlPose;

    if (ctrlPose>IKINCTRL_POSE_ANG)
        ctrlPose=IKINCTRL_POSE_ANG;
}


/************************************************************************/
void iKinCtrl::set_q(const Vector &q0)
{
    unsigned int n=q0.length();
    n=n>dim ? dim : n;

    for (unsigned int i=0; i<n; i++)
        q[i]=q0[i];

    q=chain.setAng(q);
    x=chain.EndEffPose(q);
}


/************************************************************************/
Vector iKinCtrl::calc_e()
{
    // x must be previously set
    if (x.length()>6)
    {
        if (ctrlPose==IKINCTRL_POSE_XYZ)
        {
           e[0]=x_set[0]-x[0];
           e[1]=x_set[1]-x[1];
           e[2]=x_set[2]-x[2];
           e[3]=e[4]=e[5]=0.0;
        }
        else
        {
            Vector v=x_set.subVector(3,6);
            Matrix Des=axis2dcm(v);
            Des(0,3)=x_set[0];
            Des(1,3)=x_set[1];
            Des(2,3)=x_set[2];

            Matrix H=chain.getH();
            Matrix E=Des*SE3inv(H);
            v=dcm2axis(E);
            
            e[0]=x_set[0]-H(0,3);
            e[1]=x_set[1]-H(1,3);
            e[2]=x_set[2]-H(2,3);
            e[3]=v[3]*v[0];
            e[4]=v[3]*v[1];
            e[5]=v[3]*v[2];
    
            if (ctrlPose==IKINCTRL_POSE_ANG)
               e[0]=e[1]=e[2]=0.0;
        }
    }
    else
    {
        e=x_set-x;
        if (ctrlPose==IKINCTRL_POSE_XYZ)
            e[3]=e[4]=e[5]=0.0;
        else if (ctrlPose==IKINCTRL_POSE_ANG)
            e[0]=e[1]=e[2]=0.0;
    }

    return e;
}


/************************************************************************/
void iKinCtrl::update_state()
{
    if (state==IKINCTRL_STATE_RUNNING)
    {
        if (isInTarget())
        {
            state=IKINCTRL_STATE_INTARGET;
            watchDogCnt=0;
        }
        else if (watchDogOn)
            watchDog();
    }
    else if (state==IKINCTRL_STATE_INTARGET)
    {
        if (!isInTarget())
            state=IKINCTRL_STATE_RUNNING;

        watchDogCnt=0;
    }
    else if (state==IKINCTRL_STATE_DEADLOCK)
    {
        if (isInTarget())
        {
            state=IKINCTRL_STATE_INTARGET;
            watchDogCnt=0;
        }
        else
            watchDog();
    }
}


/************************************************************************/
unsigned int iKinCtrl::printHandling(const unsigned int verbose)
{
    unsigned int hi=(verbose>>16) & 0xffff;
    unsigned int lo=verbose & 0xffff;

    if (iter % (hi+1))
        return 0;
    else
        return lo;
}


/************************************************************************/
void iKinCtrl::watchDog()
{
    if (norm(q-q_old)<watchDogTol)
        watchDogCnt++;
    else
    {
        state=IKINCTRL_STATE_RUNNING;
        watchDogCnt=0;
    }

    if (watchDogCnt>=watchDogMaxIter)
    {
        state=IKINCTRL_STATE_DEADLOCK;
        watchDogCnt=0;
    }    
}


/************************************************************************/
void iKinCtrl::restart(const Vector &q0)
{
    state=IKINCTRL_STATE_RUNNING;
    iter=0;
    set_q(q0);
}


/************************************************************************/
Vector iKinCtrl::checkVelocity(const Vector &_qdot, double _Ts)
{
    Vector checked_qdot=_qdot;
    Vector newq        =q+checked_qdot*_Ts;

    for (unsigned int i=0; i<dim; i++)
        if ((newq[i]<chain(i).getMin()) || (newq[i]>chain(i).getMax()))
            checked_qdot[i]=0.0;

    return checked_qdot;
}


/************************************************************************/
Vector iKinCtrl::solve(Vector &xd, const double tol_size, const int max_iter,
                       const unsigned int verbose, int *exit_code, bool *exhalt)
{
    while (true)
    {
        iterate(xd,verbose);
        
        if (isInTarget())
        {
            if (exit_code)
                *exit_code=IKINCTRL_RET_TOLX;

            break;
        }

        if (test_convergence(tol_size))
        {
            if (exit_code)
                *exit_code=IKINCTRL_RET_TOLSIZE;

            break;
        }

        if (state==IKINCTRL_STATE_DEADLOCK)
        {
            if (exit_code)
                *exit_code=IKINCTRL_RET_TOLQ;   

            break;
        }

        if (exhalt!=NULL)
        {
            if (*exhalt)
            {
                if (exit_code)
                    *exit_code=IKINCTRL_RET_EXHALT;
            
                break;
            }
        }

        if (max_iter>0 && (int)iter>=max_iter)
        {
            if (exit_code)
                *exit_code=IKINCTRL_RET_MAXITER;

            break;
        }
    }

    return q;
}


/************************************************************************/
SteepCtrl::SteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose,
                     double _Ts, double _Kp) : iKinCtrl(c,_ctrlPose)
{
    type=_type;
    constrained=true;

    qdot.resize(dim,0.0);

    Matrix lim(dim,2);
    for (unsigned int i=0; i<dim; i++)
    {
        lim(i,0)=chain(i).getMin();
        lim(i,1)=chain(i).getMax();
    }

    Ts=_Ts;
    I=new Integrator(Ts,chain.getAng(),lim);

    Kp=_Kp;
}


/************************************************************************/
void SteepCtrl::setChainConstraints(bool _constrained)
{
    constrained=_constrained;

    iKinCtrl::setChainConstraints(constrained);
    I->setSaturation(constrained);
}


/************************************************************************/
Vector SteepCtrl::update_qdot()
{
    qdot=-Kp*grad;

    return qdot;
}


/************************************************************************/
Vector SteepCtrl::iterate(Vector &xd, const unsigned int verbose)
{
    x_set=xd;

    if (state!=IKINCTRL_STATE_DEADLOCK)
    {
        iter++;
        q_old=q;
        
        calc_e();

        J =chain.GeoJacobian();
        Jt=J.transposed();

        if (J.rows()>=J.cols())
            pinvJ=pinv(J);
        else
            pinvJ=pinv(J.transposed()).transposed();

        if (type==IKINCTRL_STEEP_JT)
            grad=-1.0*(Jt*e);
        else
            grad=-1.0*(pinvJ*e);

        gpm=computeGPM();

        Vector _qdot=update_qdot()+gpm;

        if (constrained)
            qdot=checkVelocity(_qdot,Ts);
        else
            qdot=_qdot;

        q=chain.setAng(I->integrate(qdot));
        x=chain.EndEffPose();
    }

    update_state();

    if (state==IKINCTRL_STATE_INTARGET)
        inTargetFcn();
    else if (state==IKINCTRL_STATE_DEADLOCK)
        deadLockRecoveryFcn();

    printIter(verbose);

    return q;
}


/************************************************************************/
void SteepCtrl::restart(const Vector &q0)
{
    iKinCtrl::restart(q0);
    I->reset(q0);
    qdot=0.0;
}


/************************************************************************/
void SteepCtrl::printIter(const unsigned int verbose)
{
    // This should be the first line of any printIter method
    unsigned int _verbose=printHandling(verbose);

    if (_verbose)
    {
        string strState[3];

        strState[IKINCTRL_STATE_RUNNING] ="running";
        strState[IKINCTRL_STATE_INTARGET]="inTarget";
        strState[IKINCTRL_STATE_DEADLOCK]="deadLock";

        fprintf(stdout,"iter #%d\n",iter);
        fprintf(stdout,"state   = %s\n",strState[state].c_str());
        fprintf(stdout,"norm(e) = %g\n",dist());
        fprintf(stdout,"q       = %s\n",(CTRL_RAD2DEG*q).toString().c_str());
        fprintf(stdout,"x       = %s\n",x.toString().c_str());

        if (_verbose>1)
        {

            fprintf(stdout,"grad    = %s\n",grad.toString().c_str());
            fprintf(stdout,"qdot    = %s\n",(CTRL_RAD2DEG*qdot).toString().c_str());
        }

        if (_verbose>2)
            fprintf(stdout,"Kp      = %g\n",Kp);

        fprintf(stdout,"\n\n");
    }
}


/************************************************************************/
SteepCtrl::~SteepCtrl()
{
    delete I;
}


/************************************************************************/
VarKpSteepCtrl::VarKpSteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose, double _Ts,
                               double _Kp0, double _Kp_inc, double _Kp_dec, double _Kp_max,
                               double _max_perf_inc) : SteepCtrl(c,_ctrlPose,_type,_Ts,_Kp0)
{
    Kp_inc      =_Kp_inc;
    Kp_dec      =_Kp_dec;
    Kp_max      =_Kp_max;
    max_perf_inc=_max_perf_inc;

    dist_old=1.0;

    Kp0=Kp;
}


/************************************************************************/
void VarKpSteepCtrl::reset_Kp()
{
    Kp=Kp0;
    dist_old=1.0;
}


/************************************************************************/
Vector VarKpSteepCtrl::update_qdot()
{
    Vector qdot_old=qdot;
    qdot=SteepCtrl::update_qdot();

    double d=dist();
    double ratio=d/dist_old;
    double Kp_c=1.0;

    if (ratio>max_perf_inc && !norm(qdot_old))
    {
        qdot=qdot_old;
        Kp_c=Kp_dec;
    }
    else if (ratio<1.0)
        Kp_c=Kp_inc;

    Kp=Kp_c*Kp;

    if (Kp>Kp_max)
        Kp=Kp_max;

    dist_old=d;

    return qdot;
}


/************************************************************************/
LMCtrl::LMCtrl(iKinChain &c, unsigned int _ctrlPose, double _Ts, double _mu0,
               double _mu_inc, double _mu_dec, double _mu_min, double _mu_max,
               double _sv_thres) : iKinCtrl(c,_ctrlPose)
{
    constrained=true;

    qdot.resize(dim,0.0);

    Matrix lim(dim,2);
    for (unsigned int i=0; i<dim; i++)
    {
        lim(i,0)=chain(i).getMin();
        lim(i,1)=chain(i).getMax();
    }

    Ts=_Ts;
    I=new Integrator(Ts,chain.getAng(),lim);

    mu    =_mu0;
    mu0   =_mu0;
    mu_inc=_mu_inc;
    mu_dec=_mu_dec;
    mu_min=_mu_min;
    mu_max=_mu_max;
    svThres=_sv_thres;

    dist_old=1.0;
}


/************************************************************************/
void LMCtrl::setChainConstraints(bool _constrained)
{
    constrained=_constrained;

    iKinCtrl::setChainConstraints(constrained);
    I->setSaturation(constrained);
}


/************************************************************************/
Matrix LMCtrl::pinv(const Matrix &A, const double tol)
{
    int m=A.rows();
    int n=A.cols();
    Matrix U(m,n);
    Vector Sdiag(n);
    Matrix V(n,n);

    SVD(A,U,Sdiag,V);

    Matrix Spinv=zeros(n,n);
    for (int c=0; c<n; c++)
    {    
        for (int r=0; r<n; r++)
        {    
            if (r==c && Sdiag[c]>tol)
                Spinv(r,c)=1.0/Sdiag[c];
            else
                Spinv(r,c)=0.0;
        }
    }

    svMin=Sdiag[n-1];
    return V*Spinv*U.transposed();
}


/************************************************************************/
void LMCtrl::reset_mu()
{
    mu=mu0;
    dist_old=1.0;
}


/************************************************************************/
double LMCtrl::update_mu()
{
    if (svMin>svThres)
    {
        double d=dist();
        double ratio=d/dist_old;
    
        if (ratio>1.0)
            mu*=mu_inc;
        else
            mu*=mu_dec;
    
        mu=mu>mu_max ? mu_max : (mu<mu_min ? mu_min : mu);
    
        dist_old=d;
    }
    else
    {
        mu=mu_max;
        dist_old=1.0;
    }

    return mu;
}


/************************************************************************/
Vector LMCtrl::iterate(Vector &xd, const unsigned int verbose)
{
    x_set=xd;

    if (state!=IKINCTRL_STATE_DEADLOCK)
    {
        iter++;
        q_old=q;

        calc_e();

        J=chain.GeoJacobian();
        Jt=J.transposed();
        grad=-1.0*(Jt*e);

        Matrix LM=J*Jt;
        for (int i=0; i<6; i++)
            LM(i,i)+=mu*LM(i,i);

        if (LM.rows()>=LM.cols())
            pinvLM=Jt*pinv(LM);
        else
            pinvLM=Jt*pinv(LM.transposed()).transposed();

        if (J.rows()>=J.cols())
            pinvJ=LMCtrl::pinv(J);
        else
            pinvJ=LMCtrl::pinv(J.transposed()).transposed();

        gpm=computeGPM();

        Vector _qdot=pinvLM*e+gpm;

        if (constrained)
            qdot=checkVelocity(_qdot,Ts);
        else
            qdot=_qdot;

        q=chain.setAng(I->integrate(qdot));
        x=chain.EndEffPose();

        mu=update_mu();
    }

    update_state();

    if (state==IKINCTRL_STATE_INTARGET)
        inTargetFcn();
    else if (state==IKINCTRL_STATE_DEADLOCK)
        deadLockRecoveryFcn();

    printIter(verbose);

    return q;
}


/************************************************************************/
void LMCtrl::restart(const Vector &q0)
{
    iKinCtrl::restart(q0);
    I->reset(q0);
    qdot=0.0;
    reset_mu();
}


/************************************************************************/
void LMCtrl::printIter(const unsigned int verbose)
{
    // This should be the first line of any printIter method
    unsigned int _verbose=printHandling(verbose);

    if (_verbose)
    {
        string strState[3];

        strState[IKINCTRL_STATE_RUNNING] ="running";
        strState[IKINCTRL_STATE_INTARGET]="inTarget";
        strState[IKINCTRL_STATE_DEADLOCK]="deadLock";

        fprintf(stdout,"iter #%d\n",iter);
        fprintf(stdout,"state   = %s\n",strState[state].c_str());
        fprintf(stdout,"norm(e) = %g\n",dist());
        fprintf(stdout,"q       = %s\n",(CTRL_RAD2DEG*q).toString().c_str());
        fprintf(stdout,"x       = %s\n",x.toString().c_str());

        if (_verbose>1)
            fprintf(stdout,"grad    = %s\n",grad.toString().c_str());

        if (_verbose>2)
            fprintf(stdout,"mu      = %g\n",mu);

        fprintf(stdout,"\n\n");
    }
}


/************************************************************************/
LMCtrl::~LMCtrl()
{
    delete I;
}


/************************************************************************/
LMCtrl_GPM::LMCtrl_GPM(iKinChain &c, unsigned int _ctrlPose, double _Ts,
                       double _mu0, double _mu_inc, double _mu_dec,
                       double _mu_min, double _mu_max, double _sv_thres) : 
                       LMCtrl(c,_ctrlPose,_Ts,_mu0,_mu_inc,_mu_dec,_mu_min,_mu_max,_sv_thres)
{
    span.resize(dim);
    alpha_min.resize(dim);
    alpha_max.resize(dim);
    Eye=eye(dim,dim);

    set_safeAreaRatio(0.9);
    K=1.0;
}


/************************************************************************/
void LMCtrl_GPM::set_safeAreaRatio(const double _safeAreaRatio)
{
    safeAreaRatio=_safeAreaRatio<0.0 ? 0.0 : (_safeAreaRatio>1.0 ? 1.0 : _safeAreaRatio);

    for (unsigned int i=0; i<dim; i++)
    {
        span[i]=chain(i).getMax()-chain(i).getMin();

        double alpha=0.5*(1.0-safeAreaRatio)*span[i];
        alpha_min[i]=chain(i).getMin()+alpha;
        alpha_max[i]=chain(i).getMax()-alpha;
    }
}


/************************************************************************/
Vector LMCtrl_GPM::computeGPM()
{
    Vector w(dim);
    Vector d_min=q-alpha_min;
    Vector d_max=q-alpha_max;

    for (unsigned int i=0; i<dim; i++)
    {
        w[i] =d_min[i]>0.0 ? 0.0 : 2.0*d_min[i]/(span[i]*span[i]);
        w[i]+=d_max[i]<0.0 ? 0.0 : 2.0*d_max[i]/(span[i]*span[i]);
    }

    return (Eye-pinvLM*J)*((-K)*w);
}


/************************************************************************/
MultiRefMinJerkCtrl::MultiRefMinJerkCtrl(iKinChain &c, unsigned int _ctrlPose, double _Ts,
                                         bool nonIdealPlant) : 
                                         iKinCtrl(c,_ctrlPose), Ts(_Ts)
{
    q_set.resize(dim,0.0);
    qdot.resize(dim,0.0);
    xdot.resize(6,0.0);
    compensation.resize(dim,0.0);

    W=eye(dim,dim);
    Eye6=eye(6,6);

    execTime=1.0;

    Matrix lim(dim,2);
    for (unsigned int i=0; i<dim; i++)
    {
        lim(i,0)=chain(i).getMin();
        lim(i,1)=chain(i).getMax();
    }

    if (nonIdealPlant)
        mjCtrlJoint=new minJerkVelCtrlForNonIdealPlant(Ts,dim);
    else
        mjCtrlJoint=new minJerkVelCtrlForIdealPlant(Ts,dim);

    mjCtrlTask=new minJerkVelCtrlForIdealPlant(Ts,e.length());
    I=new Integrator(Ts,q,lim);

    gamma=0.05;
    guardRatio=0.1;

    qGuard.resize(dim);
    qGuardMinInt.resize(dim);
    qGuardMinExt.resize(dim);
    qGuardMaxInt.resize(dim);
    qGuardMaxExt.resize(dim);
    qGuardMinCOG.resize(dim);
    qGuardMaxCOG.resize(dim);

    computeGuard();
}


/************************************************************************/
void MultiRefMinJerkCtrl::set_guardRatio(double _guardRatio)
{
    guardRatio=_guardRatio>1.0 ? 1.0 : (_guardRatio<0.0 ? 0.0 : _guardRatio);

    computeGuard();
}


/************************************************************************/
void MultiRefMinJerkCtrl::computeGuard()
{
    for (unsigned int i=0; i<dim; i++)
    {
        qGuard[i]=0.25*guardRatio*(chain(i).getMax()-chain(i).getMin());

        qGuardMinExt[i]=chain(i).getMin()+qGuard[i];
        qGuardMinInt[i]=qGuardMinExt[i]  +qGuard[i];
        qGuardMinCOG[i]=0.5*(qGuardMinExt[i]+qGuardMinInt[i]);

        qGuardMaxExt[i]=chain(i).getMax()-qGuard[i];
        qGuardMaxInt[i]=qGuardMaxExt[i]  -qGuard[i];
        qGuardMaxCOG[i]=0.5*(qGuardMaxExt[i]+qGuardMaxInt[i]);
    }
}


/************************************************************************/
void MultiRefMinJerkCtrl::computeWeight()
{
    for (unsigned int i=0; i<dim; i++)
    {
        if ((q[i]>=qGuardMinInt[i]) && (q[i]<=qGuardMaxInt[i]))
            W(i,i)=gamma;
        else if ((q[i]<=qGuardMinExt[i]) || (q[i]>=qGuardMaxExt[i]))
            W(i,i)=0.0;
        else if (q[i]<qGuardMinInt[i])
            W(i,i)=0.5*gamma*(1.0+tanh(+10.0*(q[i]-qGuardMinCOG[i])/qGuard[i]));
        else
            W(i,i)=0.5*gamma*(1.0+tanh(-10.0*(q[i]-qGuardMaxCOG[i])/qGuard[i]));
    }
}


/************************************************************************/
Vector MultiRefMinJerkCtrl::iterate(Vector &xd, Vector &qd, Vector *xdot_set,
                                    const unsigned int verbose)
{
    x_set=xd;
    q_set=qd;

    if (state!=IKINCTRL_STATE_DEADLOCK)
    {
        iter++;
        q_old=q;

        calc_e();

        Vector _qdot=mjCtrlJoint->computeCmd(execTime,q_set-q+compensation);

        Vector _xdot;
        if (xdot_set!=NULL)
        {
            _xdot.resize(6);
            _xdot[0]=(*xdot_set)[0];
            _xdot[1]=(*xdot_set)[1];
            _xdot[2]=(*xdot_set)[2];
            _xdot[3]=(*xdot_set)[3]*(*xdot_set)[6];
            _xdot[4]=(*xdot_set)[4]*(*xdot_set)[6];
            _xdot[5]=(*xdot_set)[5]*(*xdot_set)[6];
        }
        else
            _xdot=mjCtrlTask->computeCmd(execTime,e);
   
        J =chain.GeoJacobian();
        Jt=J.transposed();

        computeWeight();

        qdot=_qdot+W*(Jt*(pinv(Eye6+J*W*Jt)*(_xdot-J*_qdot)));        
        xdot=J*qdot;
        q=chain.setAng(I->integrate(qdot));
        x=chain.EndEffPose();
    }

    update_state();

    if (state==IKINCTRL_STATE_INTARGET)
        inTargetFcn();
    else if (state==IKINCTRL_STATE_DEADLOCK)
        deadLockRecoveryFcn();

    printIter(verbose);

    compensation=0.0;

    return q;
}


/************************************************************************/
Vector MultiRefMinJerkCtrl::iterate(Vector &xd, Vector &qd, const unsigned int verbose)
{
    return iterate(xd,qd,NULL,verbose);
}


/************************************************************************/
Vector MultiRefMinJerkCtrl::iterate(Vector &xd, Vector &qd, Vector &xdot_set,
                                    const unsigned int verbose)
{
    return iterate(xd,qd,&xdot_set,verbose);
}


/************************************************************************/
void MultiRefMinJerkCtrl::restart(const Vector &q0)
{
    iKinCtrl::restart(q0);

    qdot=0.0;
    xdot=0.0;

    mjCtrlJoint->reset(qdot);
    mjCtrlTask->reset(zeros(e.length()));
}


/************************************************************************/
void MultiRefMinJerkCtrl::printIter(const unsigned int verbose)
{
    // This should be the first line of any printIter method
    unsigned int _verbose=printHandling(verbose);

    if (_verbose)
    {
        string strState[3];

        strState[IKINCTRL_STATE_RUNNING] ="running";
        strState[IKINCTRL_STATE_INTARGET]="inTarget";
        strState[IKINCTRL_STATE_DEADLOCK]="deadLock";

        fprintf(stdout,"state   = %s\n",strState[state].c_str());
        fprintf(stdout,"norm(e) = %g\n",dist());
        fprintf(stdout,"xd      = %s\n",x_set.toString().c_str());
        fprintf(stdout,"x       = %s\n",x.toString().c_str());
        fprintf(stdout,"qd      = %s\n",(CTRL_RAD2DEG*q_set).toString().c_str());
        fprintf(stdout,"q       = %s\n",(CTRL_RAD2DEG*q).toString().c_str());

        if (_verbose>1)
        {
            fprintf(stdout,"qdot    = %s\n",(CTRL_RAD2DEG*qdot).toString().c_str());
            fprintf(stdout,"xdot    = %s\n",xdot.toString().c_str());
        }

        if (_verbose>2)
            fprintf(stdout,"comp    = %s\n",compensation.toString().c_str());

        fprintf(stdout,"\n\n");
    }
}


/************************************************************************/
void MultiRefMinJerkCtrl::set_q(const Vector &q0)
{
    iKinCtrl::set_q(q0);
    I->reset(q);
}


/************************************************************************/
double MultiRefMinJerkCtrl::set_execTime(const double _execTime, const bool warn)
{
    double lowerThres=10.0*Ts;

    execTime=_execTime>lowerThres ? _execTime : lowerThres;

    if (warn && (execTime!=_execTime))
        fprintf(stderr,"Warning: task execution time limited to the lower bound %g\n",lowerThres);

    return execTime;
}


/************************************************************************/
void MultiRefMinJerkCtrl::add_compensation(const Vector &comp)
{
    size_t len=std::min(comp.length(),q.length());
    for (size_t i=0; i<len; i++)
        compensation[i]=comp[i];
}


/************************************************************************/
void MultiRefMinJerkCtrl::setPlantParameters(const Property &parameters,
                                             const string &entryTag)
{
    if (typeid(*mjCtrlJoint)!=typeid(minJerkVelCtrlForNonIdealPlant))
    {
        delete mjCtrlJoint;
        mjCtrlJoint=new minJerkVelCtrlForNonIdealPlant(Ts,dim);
    }

    Bottle ordering;
    for (unsigned int i=0; i<chain.getN(); i++)
        if (!chain[i].isBlocked())
            ordering.addInt(i);

    dynamic_cast<minJerkVelCtrlForNonIdealPlant*>(mjCtrlJoint)->setPlantParameters(parameters,entryTag,ordering);    
}


/************************************************************************/
MultiRefMinJerkCtrl::~MultiRefMinJerkCtrl()
{
    delete mjCtrlJoint;
    delete mjCtrlTask;
    delete I;
}



