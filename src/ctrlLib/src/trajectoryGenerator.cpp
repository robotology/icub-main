
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
    j.resize(dim);

    A.resize(4,4);
    A(0,0)=0.0; A(0,1)=1.0; A(0,2)=0.0; A(0,3)=0.0;
    A(1,0)=0.0; A(1,1)=0.0; A(1,2)=1.0; A(1,3)=0.0;
    A(2,0)=0.0; A(2,1)=0.0; A(2,2)=0.0; A(2,3)=1.0;
    b.resize(4,0.0);

    for (unsigned int i=0; i<dim; i++)
    {
        Vector X(4);

        X[0]=x[i];
        X[1]=v[i]=0.0;
        X[2]=a[i]=0.0;
        X[3]=j[i]=0.0;

        Int.push_back(new Integrator(Ts,X));
    }

    mutex=new Semaphore(1);
}


/************************************************************************/
void minJerkTrajGen::reset(const Vector &fb)
{
    for (unsigned int i=0; i<dim; i++)
    {
        Vector X(4);

        X[0]=fb[i];
        X[1]=v[i]=0.0;
        X[2]=a[i]=0.0;
        X[3]=j[i]=0.0;

        Int[i]->reset(X);
    }
}


/************************************************************************/
void minJerkTrajGen::compute(const double T, const Vector &xd, const Vector &fb)
{
    if (T!=TOld)
    {    
        double T2=T*T;
        double T3=T2*T;
        double T4=T3*T;

        // 90% in t=T
        A(3,0)=-2007.41717893511/T4;
        A(3,1)=-1200.17598071716/T3;
        A(3,2)=-268.997525253224/T2;
        A(3,3)=-26.7873693158509/T;
        b[3]=-A(3,0);

        TOld=T;
    }

    mutex->wait();
    for (unsigned int i=0; i<dim; i++)
    {
        Vector X(4);

        X[0]=fb[i];
        X[1]=v[i];
        X[2]=a[i];
        X[3]=j[i];

        X=Int[i]->integrate(A*X+xd[i]*b);

        x[i]=X[0];
        v[i]=X[1];
        a[i]=X[2];
        j[i]=X[3];
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
Vector minJerkTrajGen::get_jerk()
{
    mutex->wait();
    Vector latch_j=j;
    mutex->post();

    return latch_j;
}


/************************************************************************/
minJerkTrajGen::~minJerkTrajGen()
{
    for (unsigned int i=0; i<dim; i++)
        delete Int[i];

    delete mutex;
}




