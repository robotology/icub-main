
#include <yarp/math/Math.h>
#include <iCub/minJerkCtrl.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/************************************************************************/
minJerkVelCtrl::minJerkVelCtrl(const double _Ts, const Vector &x0) :
                               Ts(_Ts), T(1.0)
{
    computeCoeffs();
    F=new Filter(num,den,x0);
}


/************************************************************************/
void minJerkVelCtrl::computeCoeffs()
{
    double T2=T*T;
    double T3=T2*T;
    double twoOnTs=2.0/Ts;

    // 90% of steady-state value in t=T
    // transient extinguished for t>=1.5*T
    double a=-150.765868956161/T3;
    double b=-84.9812819469538/T2;
    double c=-15.9669610709384/T;

    // implementing F(s)=-a/(s^2-c*s-b)
    // with Tustin: F(z)=(-a*z^2-2*a*z-a)/((Ts/2*(Ts/2-c)-b)*z^2-2*((Ts/2)^2+b)*z+Ts/2*(Ts/2+c)-b)    
    num.resize(3);
    den.resize(3);

    double c1=twoOnTs*(twoOnTs-c)-b;
    double c2=-a/c1;

    num[0]=c2;
    num[1]=2.0*c2;
    num[2]=c2;

    den[0]=1.0;
    den[1]=-2.0*(twoOnTs*twoOnTs+b)/c1;
    den[2]=(twoOnTs*(twoOnTs+c)-b)/c1;
}


/************************************************************************/
void minJerkVelCtrl::reset(const Vector &fb)
{
    F->init(fb);
}


/************************************************************************/
Vector minJerkVelCtrl::computeCmd(const double _T, const Vector &e)
{
    if (T!=_T)
    {    
        T=_T;
        computeCoeffs();
        F->setCoeffs(num,den);
    }

    return F->filt(e);
}


/************************************************************************/
minJerkVelCtrl::~minJerkVelCtrl()
{
    delete F;
}




