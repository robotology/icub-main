
#include <yarp/os/Time.h>
#include <iCub/iKinInv.h>

#include <iostream>
#include <iomanip>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iKin;
using namespace ctrl;


/************************************************************************/
iKinCtrl::iKinCtrl(iKinChain &c, unsigned int _ctrlPose, const Vector &q0) : chain(c)
{
    dim=chain.getDOF();

    Vector _q0(dim);  _q0 =0.0;
    grad.resize(dim,0.0);

    size_t n=q0.length();
    n=n>dim ? dim : n;

    for (unsigned int i=0; i<n; i++)
        _q0[i]=q0[i];

    q    =_q0;
    q_old=q;

    inTargetTol    =5e-3;
    watchDogTol    =1e-4;
    watchDogMaxIter=200;

    iter=0;

    State=IKINCTRL_STATE_RUNNING;

    set_ctrlPose(_ctrlPose);

    watchDogOn =false;
    watchDogCnt=0;

    e.resize(6);
    x_set.resize(7);

    // randomly initialize x_set in order to avoid a precalculated grad equal
    // to zero (which is bad in some special cases), because of the call to _fdf()
    // made by gsl_multimin_fdfminimizer_set()
    for (unsigned int i=0; i<7; i++)
        x_set[i]=(rand() % 1000) / 1000.0;

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
    size_t n=q0.length();
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
            Vector v(4);
            v[0]=x_set[3];
            v[1]=x_set[4];
            v[2]=x_set[5];
            v[3]=x_set[6];
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
void iKinCtrl::updateState()
{
    if (State==IKINCTRL_STATE_RUNNING)
    {
        if (isInTarget())
        {
            State=IKINCTRL_STATE_INTARGET;
            watchDogCnt=0;
        }
        else if (watchDogOn)
            watchDog();
    }
    else if (State==IKINCTRL_STATE_INTARGET)
    {
        if (!isInTarget())
            State=IKINCTRL_STATE_RUNNING;

        watchDogCnt=0;
    }
    else if (State==IKINCTRL_STATE_DEADLOCK)
        if (isInTarget())
        {
            State=IKINCTRL_STATE_INTARGET;
            watchDogCnt=0;
        }
        else
            watchDog();
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
        State=IKINCTRL_STATE_RUNNING;
        watchDogCnt=0;
    }

    if (watchDogCnt>=watchDogMaxIter)
    {
        State=IKINCTRL_STATE_DEADLOCK;
        watchDogCnt=0;
    }    
}


/************************************************************************/
Vector iKinCtrl::checkVelocity(const Vector &_qdot, double _Ts)
{
    Vector checked_qdot=_qdot;
    Vector newq        =q+checked_qdot*_Ts;

    for (unsigned int i=0; i<dim; i++)
        if (newq[i]<chain(i).getMin() || newq[i]>chain(i).getMax())
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

        if (State==IKINCTRL_STATE_DEADLOCK)
        {
            if (exit_code)
                *exit_code=IKINCTRL_RET_TOLQ;   

            break;
        }

        if (exhalt)
            if (*exhalt)
            {
                if (exit_code)
                    *exit_code=IKINCTRL_RET_EXHALT;

                break;
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
SteepCtrl::SteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose, const Vector &q0,
                     double _Ts, double _Kp) : iKinCtrl(c,_ctrlPose,q0)
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
    I=new Integrator(Ts,q0,lim);

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

    if (State!=IKINCTRL_STATE_DEADLOCK)
    {
        iter++;
    
        q_old=q;

        x=chain.EndEffPose();
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

        q=I->integrate(qdot);
        q=chain.setAng(q);
    }

    updateState();

    if (State==IKINCTRL_STATE_INTARGET)
        inTargetFcn();
    else if (State==IKINCTRL_STATE_DEADLOCK)
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

        cout << "iter #"     << iter                        << endl;
        cout << "State   = " << strState[State]             << endl;
        cout << "norm(e) = " << dist()                      << endl;
        cout << "q       = " << (CTRL_RAD2DEG*q).toString() << endl;
        cout << "x       = " << x.toString()                << endl;

        if (_verbose>1)
        {
            cout << "grad    = " << grad.toString()                << endl;
            cout << "qdot    = " << (CTRL_RAD2DEG*qdot).toString() << endl;
        }

        if (_verbose>2)
            cout << "Kp      = " << Kp << endl;

        cout << endl << endl;
    }
}


/************************************************************************/
VarKpSteepCtrl::VarKpSteepCtrl(iKinChain &c, unsigned int _type, unsigned int _ctrlPose, const Vector &q0,
                               double _Ts, double _Kp0, double _Kp_inc, double _Kp_dec, double _Kp_max,
                               double _max_perf_inc) : SteepCtrl(c,_ctrlPose,_type,q0,_Ts,_Kp0)
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
LMCtrl::LMCtrl(iKinChain &c, unsigned int _ctrlPose, const Vector &q0, double _Ts, double _mu0,
               double _mu_inc, double _mu_dec, double _mu_min, double _mu_max) : iKinCtrl(c,_ctrlPose,q0)
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
    I=new Integrator(Ts,q0,lim);

    mu    =_mu0;
    mu0   =_mu0;
    mu_inc=_mu_inc;
    mu_dec=_mu_dec;
    mu_min=_mu_min;
    mu_max=_mu_max;

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
void LMCtrl::reset_mu()
{
    mu=mu0;
    dist_old=1.0;
}


/************************************************************************/
double LMCtrl::update_mu()
{
    double d=dist();
    double ratio=d/dist_old;
    double mu_c=1.0;

    if (ratio>1.0)
        mu_c=mu_inc;
    else
        mu_c=mu_dec;

    mu*=mu_c;

    mu=mu>mu_max ? mu_max : (mu<mu_min ? mu_min : mu);

    dist_old=d;

    return mu;
}


/************************************************************************/
Vector LMCtrl::iterate(Vector &xd, const unsigned int verbose)
{
    x_set=xd;

    if (State!=IKINCTRL_STATE_DEADLOCK)
    {
        iter++;

        q_old=q;

        x=chain.EndEffPose();
        calc_e();

        J=chain.GeoJacobian();
        Jt=J.transposed();
        grad=-1.0*(Jt*e);

        Matrix H=Jt*J;

        Matrix M=H;
        for (unsigned int i=0; i<dim; i++)
            M(i,i)+=mu*M(i,i);

        Matrix pinvM;
        if (M.rows()>=M.cols())
            pinvM=pinv(M);
        else
            pinvM=pinv(M.transposed()).transposed();

        if (J.rows()>=J.cols())
            pinvJ=pinv(J);
        else
            pinvJ=pinv(J.transposed()).transposed();

        gpm=computeGPM();

        Vector _qdot=-1.0*pinvM*grad+gpm;

        if (constrained)
            qdot=checkVelocity(_qdot,Ts);
        else
            qdot=_qdot;

        q=I->integrate(qdot);
        q=chain.setAng(q);

        mu=update_mu();
    }

    updateState();

    if (State==IKINCTRL_STATE_INTARGET)
        inTargetFcn();
    else if (State==IKINCTRL_STATE_DEADLOCK)
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

        cout << "iter #"     << iter                        << endl;
        cout << "State   = " << strState[State]             << endl;
        cout << "norm(e) = " << dist()                      << endl;
        cout << "q       = " << (CTRL_RAD2DEG*q).toString() << endl;
        cout << "x       = " << x.toString()                << endl;

        if (_verbose>1)
            cout << "grad    = " << grad.toString() << endl;

        if (_verbose>2)
            cout << "mu      = " << mu << endl;

        cout << endl << endl;
    }
}


/************************************************************************/
LMCtrl_GPM::LMCtrl_GPM(iKinChain &c, unsigned int _ctrlPose, const Vector &q0,
                       double _Ts, double _mu0, double _mu_inc, double _mu_dec,
                       double _mu_min, double _mu_max) : 
                       LMCtrl(c,_ctrlPose,q0,_Ts,_mu0,_mu_inc,_mu_dec,_mu_min,_mu_max)
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
        w[i] =d_min[i]>0 ? 0.0 : 2.0*d_min[i]/(span[i]*span[i]);
        w[i]+=d_max[i]<0 ? 0.0 : 2.0*d_max[i]/(span[i]*span[i]);
    }

    return (Eye-pinvJ*J)*((-K)*w);
}


namespace iKin
{
    /************************************************************************/
    double _f(const gsl_vector *v, void *params)
    {
        GSLMinCtrl &ctrl=*(GSLMinCtrl*)(params);
        unsigned int i;
    
        for (i=0; i<ctrl.dim; i++)
            ctrl.q[i]=gsl_vector_get(v,i);

        ctrl.q=ctrl.chain.setAng(ctrl.q);
    	ctrl.x=ctrl.chain.EndEffPose();
        ctrl.e=ctrl.calc_e();
    
        double e_x =0.5*norm2(ctrl.e);
        double e_qi=0.0;
        for (i=0; (int)i<ctrl.q_set_len; i++)
        {
            unsigned int j=ctrl.hash_qf[i];
            double d=ctrl.q[j]-ctrl.q_set[j];
            e_qi+=0.5*(d*d);
        }
    
        return e_x+e_qi;
    }
    
    
    /************************************************************************/
    void _df(const gsl_vector *v, void *params, gsl_vector *g)
    {
        GSLMinCtrl &ctrl=*(GSLMinCtrl*)(params);
        unsigned int i;
    
        for (i=0; i<ctrl.dim; i++)
            ctrl.q[i]=gsl_vector_get(v,i);

        ctrl.q =ctrl.chain.setAng(ctrl.q);
        ctrl.x =ctrl.chain.EndEffPose();
        ctrl.J =ctrl.chain.GeoJacobian();
        ctrl.Jt=ctrl.J.transposed();
        ctrl.e =ctrl.calc_e();
    
        ctrl.grad=-1.0*(ctrl.Jt*ctrl.e);

        for (i=0; (int)i<ctrl.q_set_len; i++)
        {
            unsigned int j=ctrl.hash_qf[i];
            ctrl.grad[j]+=ctrl.q[j]-ctrl.q_set[j];
        }
    
    	for (i=0; i<ctrl.dim; i++)
            gsl_vector_set(g,i,ctrl.grad[i]);
    }
    
    
    /************************************************************************/
    void _fdf(const gsl_vector *v, void *params, double *f, gsl_vector *g)
    {
        *f=_f(v,params);
        _df(v,params,g);
    }
}


/************************************************************************/
GSLMinCtrl::GSLMinCtrl(iKinChain &c, unsigned int _ctrlPose, const Vector &q0, const unsigned int _algo_type,
                       double _step_size, double _tol) : algo_type(_algo_type), step_size(_step_size),
                       tol(_tol), iKinCtrl(c,_ctrlPose,q0)
{  
    q_set.resize(dim,0.0);
    q_set_len=0;

    hash_qf.clear();
    hash_qf.resize(dim,-1);

    T1=NULL;
    T2=NULL;
    s1=NULL;
    s2=NULL;

    switch (algo_type)
    {
    case GSLALGOTYPE_STEEPEST:
        T1=gsl_multimin_fdfminimizer_steepest_descent;
        fdfOn=true;
        break;

    case GSLALGOTYPE_CONJ_FR:
        T1=gsl_multimin_fdfminimizer_conjugate_fr;
        fdfOn=true;
        break;

    case GSLALGOTYPE_CONJ_PR:
        T1=gsl_multimin_fdfminimizer_conjugate_pr;
        fdfOn=true;
        break;

    case GSLALGOTYPE_BFGS:
        T1=gsl_multimin_fdfminimizer_vector_bfgs;
        fdfOn=true;
        break;

    case GSLALGOTYPE_NMSIMPLEX:
    default:
        T2=gsl_multimin_fminimizer_nmsimplex;
        fdfOn=false;
    }

    if (fdfOn)
    {
        s1=gsl_multimin_fdfminimizer_alloc(T1,dim);

        des1.n     =dim;
        des1.f     =&_f;
        des1.df    =&_df;
        des1.fdf   =&_fdf;
        des1.params=(void*)this;

        gsl_multimin_fdfminimizer_set(s1,&des1,(const gsl_vector*)q0.getGslVector(),step_size,tol);
    }
    else
    {
        s2=gsl_multimin_fminimizer_alloc(T2,dim);

        des2.n     =dim;
        des2.f     =&_f;
        des2.params=(void*)this;

        Vector size(dim);
        size=step_size;

        gsl_multimin_fminimizer_set(s2,&des2,(const gsl_vector*)q0.getGslVector(),(const gsl_vector*)size.getGslVector());
    }
}


/************************************************************************/
void GSLMinCtrl::switch_qf(const Vector &sw)
{
    q_set_len=-1;
    hash_qf.clear();
    hash_qf.resize(dim,-1);

    for (int i=0; i<(int)dim && i<(int)sw.size(); i++)
        if (sw[i])
            hash_qf[++q_set_len]=i;
}


/************************************************************************/
void GSLMinCtrl::set_qf(const Vector &qf)
{
    int len=qf.length();
    len=q_set_len<len ? q_set_len : len;

    for (int i=0; i<len; i++)
        q_set[hash_qf[i]]=qf[i];
}


/************************************************************************/
Vector GSLMinCtrl::iterate(Vector &xd, const unsigned int verbose)
{
    unsigned int i;

    x_set=xd;

    if (State!=IKINCTRL_STATE_DEADLOCK)
    {    
        iter++;

        q_old=q;        
    
        if (fdfOn)
        {
            gsl_multimin_fdfminimizer_iterate(s1);

            for (i=0; i<dim; i++)
            {
                q[i]   =gsl_vector_get(s1->x,i);
                grad[i]=gsl_vector_get(s1->gradient,i);
            }
        }
        else
        {
            gsl_multimin_fminimizer_iterate(s2);
    
            for (i=0; i<dim; i++)
                q[i]=gsl_vector_get(s2->x,i);
        }


        q=chain.setAng(q);
    }

    updateState();

    if (State==IKINCTRL_STATE_INTARGET)
        inTargetFcn();
    else if (State==IKINCTRL_STATE_DEADLOCK)
        deadLockRecoveryFcn();

    printIter(verbose);

    return q;
}


/************************************************************************/
void GSLMinCtrl::reset(const Vector &q0)
{
    if (fdfOn)
        gsl_multimin_fdfminimizer_set(s1,&des1,(const gsl_vector*)q0.getGslVector(),step_size,tol);
    else
    {
        Vector size(dim);
        size=step_size;

        gsl_multimin_fminimizer_set(s2,&des2,(const gsl_vector*)q0.getGslVector(),(const gsl_vector*)size.getGslVector());
    }
}


/************************************************************************/
bool GSLMinCtrl::test_convergence(const double tol_size)
{
    if (fdfOn)
    {
        if (gsl_multimin_test_gradient(s1->gradient,tol_size)==GSL_SUCCESS)
            return true;
        else
            return false;
    }
    else
    {
        double size=gsl_multimin_fminimizer_size(s2);

        if (gsl_multimin_test_size(size,tol_size)==GSL_SUCCESS)
            return true;
        else
            return false;
    } 
}


/************************************************************************/
void GSLMinCtrl::printIter(const unsigned int verbose)
{
    // This should be the first line of any printIter method
    unsigned int _verbose=printHandling(verbose);

    if (_verbose) 
    {
        string strState[3];

        strState[IKINCTRL_STATE_RUNNING] ="running";
        strState[IKINCTRL_STATE_INTARGET]="inTarget";
        strState[IKINCTRL_STATE_DEADLOCK]="deadLock";

        cout << "iter #"     << iter                        << endl;
        cout << "state   = " << strState[State]             << endl;
        cout << "norm(e) = " << dist()                      << endl;
        cout << "q       = " << (CTRL_RAD2DEG*q).toString() << endl;
        cout << "x       = " << x.toString()                << endl;

        if (_verbose>1)
            if (fdfOn)
                cout << "grad    = " << grad.toString() << endl;
            else
                cout << "size    = " << gsl_multimin_fminimizer_size(s2) << endl;

        cout << endl << endl;
    }
}


/************************************************************************/
Vector GSLMinCtrl::get_x()
{
    x=chain.EndEffPose(q);

    return x;
}


/************************************************************************/
Vector GSLMinCtrl::get_e()
{
    x=chain.EndEffPose(q);
    e=calc_e();

    return e;
}


/************************************************************************/
Vector GSLMinCtrl::get_grad()
{
    e   =get_e();
    J   =get_J();
    Jt  =J.transposed();
    grad=-1.0*(Jt*e);

    return grad;
}


/************************************************************************/
Matrix GSLMinCtrl::get_J()
{
    J=chain.GeoJacobian(q);

    return J;
}


/************************************************************************/
double GSLMinCtrl::dist()
{
    e=get_e();

    return norm(e);
}


/************************************************************************/
void GSLMinCtrl::set_size(double _size)
{
    step_size=_size;
}


/************************************************************************/
double GSLMinCtrl::get_size()
{
    if (fdfOn)
        return -1.0;
    else
        return gsl_multimin_fminimizer_size(s2);
}


/************************************************************************/
string GSLMinCtrl::getAlgoName()
{
    if (fdfOn)
        return gsl_multimin_fdfminimizer_name(s1);
    else
        return gsl_multimin_fminimizer_name(s2);
}


/************************************************************************/
GSLMinCtrl::~GSLMinCtrl()
{
    if (fdfOn)
        gsl_multimin_fdfminimizer_free(s1);
    else
        gsl_multimin_fminimizer_free(s2);
}


/************************************************************************/
MultiRefMinJerkCtrl::MultiRefMinJerkCtrl(iKinChain &c, unsigned int _ctrlPose,
                                         const Vector &q0,  double _Ts) : 
                                         iKinCtrl(c,_ctrlPose,q0), Ts(_Ts)
{
    q_set.resize(dim,0.0);
    qdot.resize(dim,0.0);
    xdot.resize(6,0.0);

    W=eye(dim,dim);
    Eye6=eye(6,6);

    execTime=1.0;

    Matrix lim(dim,2);
    for (unsigned int i=0; i<dim; i++)
    {
        lim(i,0)=chain(i).getMin();
        lim(i,1)=chain(i).getMax();
    }

    genTrajJoint=new minJerkTrajGen(Ts,q);
    genTrajTask =new minJerkTrajGen(Ts,x);
    Int=new Integrator(Ts,q,lim);

    gamma=0.02;
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
        if (q[i]>=qGuardMinInt[i] && q[i]<=qGuardMaxInt[i])
            W(i,i)=gamma;
        else if (q[i]<=qGuardMinExt[i] || q[i]>=qGuardMaxExt[i])
            W(i,i)=0.0;
        else if (q[i]<qGuardMinInt[i])
            W(i,i)=0.5*gamma*(1.0+tanh(+10.0*(q[i]-qGuardMinCOG[i])/qGuard[i]));
        else
            W(i,i)=0.5*gamma*(1.0+tanh(-10.0*(q[i]-qGuardMaxCOG[i])/qGuard[i]));
}


/************************************************************************/
Vector MultiRefMinJerkCtrl::calc_e()
{
    // x must be previously set
    Vector __e=x_set-x;

    if (__e.length()>6)
    {
       e[0]=__e[0];
       e[1]=__e[1];
       e[2]=__e[2];
       e[3]=__e[3]*__e[6];
       e[4]=__e[4]*__e[6];
       e[5]=__e[5]*__e[6];
    }
    else
        e=__e;

    if (ctrlPose==IKINCTRL_POSE_XYZ)
        e[3]=e[4]=e[5]=0.0;
    else if (ctrlPose==IKINCTRL_POSE_ANG)
        e[0]=e[1]=e[2]=0.0;

    return e;
}


/************************************************************************/
Vector MultiRefMinJerkCtrl::iterate(Vector &xd, Vector &qd, const unsigned int verbose)
{
    x_set=xd;
    q_set=qd;

    if (State!=IKINCTRL_STATE_DEADLOCK)
    {
        iter++;
    
        q_old=q;
        calc_e();

        genTrajJoint->compute(execTime,q_set,q);
        genTrajTask->compute(execTime,x_set,x);

        qdot=genTrajJoint->get_v();
        xdot7=genTrajTask->get_v();

        xdot[0]=xdot7[0];
        xdot[1]=xdot7[1];
        xdot[2]=xdot7[2];
        xdot[3]=xdot7[3]*xdot7[6];
        xdot[4]=xdot7[4]*xdot7[6];
        xdot[5]=xdot7[5]*xdot7[6];
    
        J =chain.GeoJacobian();
        Jt=J.transposed();

        computeWeight();

        qdot=qdot+W*(Jt*(pinv(Eye6+J*W*Jt)*(xdot-J*qdot)));
        qdot=checkVelocity(qdot,Ts);
        xdot=J*qdot;
        q=chain.setAng(Int->integrate(qdot));
        x=chain.EndEffPose();
    }

    updateState();

    if (State==IKINCTRL_STATE_INTARGET)
        inTargetFcn();
    else if (State==IKINCTRL_STATE_DEADLOCK)
        deadLockRecoveryFcn();

    printIter(verbose);    

    return q;
}


/************************************************************************/
void MultiRefMinJerkCtrl::restart(const Vector &q0)
{
    iKinCtrl::restart(q0);

    genTrajJoint->reset(q);
    genTrajTask->reset(x);

    qdot=0.0;
    xdot=0.0;
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

        cout << "State   = " << strState[State]                 << endl;
        cout << "norm(e) = " << dist()                          << endl;
        cout << "xd      = " << x_set.toString()                << endl;
        cout << "x       = " << x.toString()                    << endl;
        cout << "qd      = " << (CTRL_RAD2DEG*q_set).toString() << endl;
        cout << "q       = " << (CTRL_RAD2DEG*q).toString()     << endl;

        if (_verbose>1)
        {
            cout << "qdot    = " << (CTRL_RAD2DEG*qdot).toString() << endl;
            cout << "xdot    = " << xdot.toString()                << endl;
        }

        cout << endl << endl;
    }
}


/************************************************************************/
void MultiRefMinJerkCtrl::set_q(const Vector &q0)
{
    iKinCtrl::set_q(q0);
    Int->reset(q);
}


/************************************************************************/
double MultiRefMinJerkCtrl::set_execTime(const double _execTime, const bool warn)
{
    double lowerThres=10.0*Ts;

    execTime=_execTime>lowerThres ? _execTime : lowerThres;

    if (warn && execTime!=_execTime)
        cerr << "Warning: task execution time limited to the lower bound "
             << lowerThres << endl;

    return execTime;
}


/************************************************************************/
MultiRefMinJerkCtrl::~MultiRefMinJerkCtrl()
{
    delete genTrajJoint;
    delete genTrajTask;
    delete Int;
}



