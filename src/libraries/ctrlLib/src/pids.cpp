
#include <yarp/math/Math.h>
#include <iCub/pids.h>
#include <iCub/ctrlMath.h>

#include <iostream>
#include <iomanip>

#define SAT(x,L,H)  ((x)>(H)?(H):((x)<(L)?(L):(x)))

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/************************************************************************/
Integrator::Integrator(const double _Ts, const Vector &y0, const Matrix &_lim)
{
    dim=y0.length();
    x_old.resize(dim,0.0);
    applySat=true;

    Ts =_Ts;
    lim=_lim;
    y  =saturate(y0);
}


/************************************************************************/
Integrator::Integrator(const double _Ts, const Vector &y0)
{
    dim=y0.length();
    x_old.resize(dim,0.0);
    applySat=false;

    lim.resize(1,2);

    Ts=_Ts;
    y =y0;
}


/************************************************************************/
void Integrator::_allocate(const Integrator &I)
{
    y       =I.y;
    x_old   =I.x_old;
    Ts      =I.Ts;
    dim     =I.dim;
    applySat=I.applySat;
}


/************************************************************************/
Vector Integrator::saturate(const Vector &v)
{
    if (applySat)
    {
        Vector res=v;
    
        for (unsigned int i=0; i<dim; i++)
            if (res[i]<lim(i,0))
                res[i]=lim(i,0);
            else if (res[i]>lim(i,1))
                res[i]=lim(i,1);
    
        return res;
    }
    else
        return v;
}


/************************************************************************/
void Integrator::setSaturation(bool _applySat)
{
    if (applySat=_applySat)
        y=saturate(y);
}


/************************************************************************/
void Integrator::setLim(const yarp::sig::Matrix &_lim)
{
    lim=_lim;

    if (applySat)
        y=saturate(y);
}


/************************************************************************/
Vector Integrator::integrate(const Vector &x)
{
    // implements the Tustin formula
    y=saturate(y+(x+x_old)*(Ts/2));
    x_old=x;

    return y;
}


/************************************************************************/
void Integrator::reset(const Vector &y0)
{
    y=saturate(y0);
    x_old=0.0;
}


/************************************************************************/
parallelPID::parallelPID(const double _Ts,
                         const Vector &_Kp, const Vector &_Ki, const Vector &_Kd,
                         const Vector &_Wp, const Vector &_Wi, const Vector &_Wd,
                         const Vector &_N,  const Vector &_Tt, const Matrix &_satLim) : 
                         Ts(_Ts), Kp(_Kp), Ki(_Ki), Kd(_Kd), Wp(_Wp), Wi(_Wi), Wd(_Wd),
                         N(_N), Tt(_Tt), satLim(_satLim)
{
    dim=N.length();
    u.resize(dim,0.0);

    uSat.resize(dim);
    for (unsigned int i=0; i<dim; i++)
        uSat[i]=SAT(u[i],satLim(i,0),satLim(i,1));

    Int=new Integrator(Ts,uSat);

    for (unsigned int i=0; i<dim; i++)
    {
        Vector num(2),den(2),u0i(1);
        double tau=Kd[i]/(Kp[i]*N[i]);

        num[0]=2.0;        num[1]=-2.0;
        den[0]=Ts+2.0*tau; den[1]=Ts-2.0*tau;
        u0i[0]=uSat[i];

        Der.push_back(new Filter(num,den,u0i));
    }

    P.resize(dim);
    I.resize(dim);
    D.resize(dim);
}


/************************************************************************/
Vector parallelPID::compute(const Vector &ref, const Vector &fb)
{
    // proportional part
    P=Kp*(Wp*ref-fb);

    // integral part
    I=Int->integrate(Ki*(Wi*ref-fb)+(uSat-u)/Tt);

    // derivative part
    Vector inputD=Kd*(Wd*ref-fb);
    for (unsigned int i=0; i<dim; i++)
    {
        Vector inputDi(1);
        Vector outputDi(1);
        
        inputDi[0]=inputD[i];
        outputDi=Der[i]->filt(inputDi);

        D[i]=outputDi[0];
    }

    // cumul output
    u=P+I+D;

    // saturation stage
    for (unsigned int i=0; i<dim; i++)
        uSat[i]=SAT(u[i],satLim(i,0),satLim(i,1));

    return uSat;
}


/************************************************************************/
void parallelPID::reset()
{
    Vector z(dim); z=0.0;
    Vector z1(1);  z1=0.0;

    Int->reset(z);

    for (unsigned int i=0; i<dim; i++)
        Der[i]->init(z1);
}


/************************************************************************/
parallelPID::~parallelPID()
{
    delete Int;
    
    for (unsigned int i=0; i<dim; i++)
        delete Der[i];

    Der.clear();
}


/************************************************************************/
seriesPID::seriesPID(const double _Ts,
                     const Vector &_Kp, const Vector &_Ti, const Vector &_Kd,
                     const Vector &_N,  const Matrix &_satLim) :
                     Ts(_Ts), Kp(_Kp), Ti(_Ti), Kd(_Kd), N(_N), satLim(_satLim)
{
    dim=N.length();
    u.resize(dim,0.0);

    uSat.resize(dim);
    for (unsigned int i=0; i<dim; i++)
        uSat[i]=SAT(u[i],satLim(i,0),satLim(i,1));

    for (unsigned int i=0; i<dim; i++)
    {
        Vector num(2),den(2),u0i(1);
        u0i[0]=uSat[i];

        num[0]=Ts;           num[1]=Ts;
        den[0]=Ts+2.0*Ti[i]; den[1]=Ts-2.0*Ti[i];

        Int.push_back(new Filter(num,den,u0i));

        double tau=Kd[i]/(Kp[i]*N[i]);
        num[0]=2.0;        num[1]=-2.0;
        den[0]=Ts+2.0*tau; den[1]=Ts-2.0*tau;

        Der.push_back(new Filter(num,den,u0i));
    }

    e.resize(dim);
    P.resize(dim);
    I.resize(dim);
    D.resize(dim);
}


/************************************************************************/
Vector seriesPID::compute(const Vector &ref, const Vector &fb)
{
    // compute error
    e=ref-fb;

    // derivative part
    for (unsigned int i=0; i<dim; i++)
    {
        Vector inputDi(1);
        Vector outputDi(1);

        inputDi[0]=Kd[i]*e[i];
        outputDi=Der[i]->filt(inputDi);

        D[i]=outputDi[0];
    }

    // proportional part
    P=Kp*(e+D);

    // integral part
    for (unsigned int i=0; i<dim; i++)
    {
        Vector inputIi(1);
        Vector outputIi(1);
        
        inputIi[0]=uSat[i];
        outputIi=Int[i]->filt(inputIi);

        I[i]=outputIi[0];
    }

    // cumul output
    u=P+I;

    // saturation stage
    for (unsigned int i=0; i<dim; i++)
        uSat[i]=SAT(u[i],satLim(i,0),satLim(i,1));

    return uSat;
}


/************************************************************************/
void seriesPID::reset()
{
    Vector z1(1); z1=0.0;

    for (unsigned int i=0; i<dim; i++)
    {
        Int[i]->init(z1);    
        Der[i]->init(z1);
    }
}


/************************************************************************/
seriesPID::~seriesPID()
{
    for (unsigned int i=0; i<dim; i++)
    {   
        delete Int[i]; 
        delete Der[i];
    }

    Int.clear();
    Der.clear();
}


