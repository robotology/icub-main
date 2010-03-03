
#include <yarp/math/Math.h>
#include <iCub/trajectoryGenerator.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/************************************************************************/
minJerkTrajGen::minJerkTrajGen(const double _Ts, const Vector &x0) :
                               Ts(_Ts), x(x0), TOld(0.0)
{
    dim=x.length();

    v.resize(dim);
    a.resize(dim);

    A.resize(3,3);
    A(0,0)=0.0; A(0,1)=1.0; A(0,2)=0.0;
    A(1,0)=0.0; A(1,1)=0.0; A(1,2)=1.0;
    b.resize(3,0.0);

    for (unsigned int i=0; i<dim; i++)
    {
        Vector X(3);

        X[0]=x[i];
        X[1]=v[i]=0.0;
        X[2]=a[i]=0.0;

        Int.push_back(new Integrator(Ts,X));
    }

    mutex=new Semaphore(1);
}


/************************************************************************/
void minJerkTrajGen::reset(const Vector &fb)
{
    for (unsigned int i=0; i<dim; i++)
    {
        Vector X(3);

        X[0]=fb[i];
        X[1]=v[i]=0.0;
        X[2]=a[i]=0.0;

        Int[i]->reset(X);
    }
}


/************************************************************************/
void minJerkTrajGen::compute(const double T, const Vector &xd, const Vector &fbPos)
{
    compute(T,xd,fbPos,v,a);
}


/************************************************************************/
void minJerkTrajGen::compute(const double T, const Vector &xd, const Vector &fbPos,
                             const Vector &fbVel)
{
    compute(T,xd,fbPos,fbVel,a);
}


/************************************************************************/
void minJerkTrajGen::compute(const double T, const Vector &xd, const Vector &fbPos,
                             const Vector &fbVel, const Vector &fbAcc)
{
    if (T!=TOld)
    {    
        double T2=T*T;
        double T3=T2*T;

        // 90% of steady-state value in t=T
        // transient extinguished for t>=1.5*T
        A(2,0)=-150.765868956161/T3;
        A(2,1)=-84.9812819469538/T2;
        A(2,2)=-15.9669610709384/T;
        b[2]=-A(2,0);

        TOld=T;
    }

    mutex->wait();
    for (unsigned int i=0; i<dim; i++)
    {
        Vector X(3);

        X[0]=fbPos[i];
        X[1]=fbVel[i];
        X[2]=fbAcc[i];

        X=Int[i]->integrate(A*X+xd[i]*b);

        x[i]=X[0];
        v[i]=X[1];
        a[i]=X[2];
    }
    mutex->post();
}


/************************************************************************/
Vector minJerkTrajGen::get_pos()
{
    mutex->wait();
    Vector latch_x=x;
    mutex->post();

    return latch_x;
}


/************************************************************************/
Vector minJerkTrajGen::get_vel()
{
    mutex->wait();
    Vector latch_v=v;
    mutex->post();

    return latch_v;
}


/************************************************************************/
Vector minJerkTrajGen::get_acc()
{
    mutex->wait();
    Vector latch_a=a;
    mutex->post();

    return latch_a;
}


/************************************************************************/
minJerkTrajGen::~minJerkTrajGen()
{
    for (unsigned int i=0; i<dim; i++)
        delete Int[i];

    delete mutex;
}




