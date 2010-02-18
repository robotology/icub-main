
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
                               Ts(_Ts), x(x0)
{
    dim=x.length();

    v.resize(dim,0.0);
    a.resize(dim,0.0);

    A.resize(3,3);
    A(0,0)=0.0; A(0,1)=1.0; A(0,2)=0.0;
    A(1,0)=0.0; A(1,1)=0.0; A(1,2)=1.0;
    b.resize(3,0.0);

    TOld=0.0;

    for (unsigned int i=0; i<dim; i++)
    {
        Vector X(3);

        X[0]=x[i]; X[1]=X[2]=0.0;

        Integrator *tmpInt=new Integrator(Ts,X);
        Int.push_back(tmpInt);
    }

    mutex=new Semaphore(1);

    state=MINJERK_STATE_REACHED;
}


/************************************************************************/
void minJerkTrajGen::compute(const double T, const Vector &xd, const Vector &fb,
                             const double tol)
{
    if (T!=TOld)
    {    
        double T1=0.8*T;
        double T2=T1*T1;
        double T3=T2*T1;

        A(2,0)=-60.0/T3; A(2,1)=36.0/T2; A(3,1)=-9.0/T1;
        b[2]=-A(2,0);

        TOld=T;
    }

    if (norm(xd-fb)<tol)
        state=MINJERK_STATE_REACHED;
    else
    {
        state=MINJERK_STATE_RUNNING;

        mutex->wait();
        for (unsigned int i=0; i<dim; i++)
        {
            Vector X(3);
    
            X[0]=x[i]; X[1]=v[i]; X[2]=a[i];
    
            X=Int[i]->integrate(A*X+xd[i]*b);
    
            x[i]=X[0]; v[i]=X[1]; a[i]=X[2];
        }
        mutex->post();
    }
}


/************************************************************************/
Vector minJerkTrajGen::get_x()
{
    mutex->wait();
    Vector latch_x=x;
    mutex->post();

    return latch_x;
}


/************************************************************************/
Vector minJerkTrajGen::get_v()
{
    mutex->wait();
    Vector latch_v=v;
    mutex->post();

    return latch_v;
}


/************************************************************************/
Vector minJerkTrajGen::get_a()
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




