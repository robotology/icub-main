
#include <yarp/math/Math.h>
#include <iCub/trajectoryGenerator.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/************************************************************************/
minJerkTrajGen::minJerkTrajGen(const double _Ts, const Vector &x0) :
                               Ts(_Ts), x(x0)
{
    dim=x.length();
    xdOld=x;

    v.resize(dim,0.0);
    a.resize(dim,0.0);
    vtau.resize(6);
    vData.resize(5);
    aData.resize(4);

    for (unsigned int i=0; i<dim; i++)
    {
        Vector c(6); c=0.0;
        coeff.push_back(c);
    }

    TOld=t0=t=tau=0.0;

    vtau[0]=1.0;

    vData[0]=1.0;
    vData[1]=2.0;
    vData[2]=3.0;
    vData[3]=4.0;
    vData[4]=5.0;

    aData[0]=2.0;
    aData[1]=6.0;
    aData[2]=12.0;
    aData[3]=20.0;

    state=MINJERK_STATE_REACHED;    
}


/************************************************************************/
void minJerkTrajGen::calcCoeff(const double T, const Vector &xd, const Vector &fb)
{
    for (unsigned int i=0; i<dim; i++)
    {
        double ei=xd[i]-fb[i];
        double tmp1=T*v[i];
        double tmp2=T*T*a[i]/2.0;
    
        coeff[i][0]=fb[i];
        coeff[i][1]=tmp1;
        coeff[i][2]=tmp2;
        coeff[i][3]=-3.0*tmp2-6.0*tmp1+10.0*ei;
        coeff[i][4]=3.0*tmp2+8.0*tmp1-15.0*ei;
        coeff[i][5]=-tmp2-3.0*tmp1+6.0*ei;
    
        xdOld[i]=xd[i];
    }

    TOld=T;
}


/************************************************************************/
void minJerkTrajGen::compute(const double T, const Vector &xd, const Vector &fb,
                             const double tol, const double dt)
{
    if (fabs(T-TOld)>1e-6 || norm(xd-xdOld)>1e-6)
    {    
        calcCoeff(T,xd,fb);
        t0=t;
        state=MINJERK_STATE_STARTING;
    }

    if (dt<0.0)
        t+=Ts;
    else
        t+=dt;        

    tau=(t-t0)/T;

    if (tau>=1.0)
    {
        if (norm(xd-fb)<tol)
        {
            x=xd;
            v=a=0.0;
            state=MINJERK_STATE_REACHED;
        }
        else
        {
            calcCoeff(T,xd,fb);
            t0=t;
            state=MINJERK_STATE_FEEDBACK;
        }
    }

    if (state!=MINJERK_STATE_REACHED)
    {
        for (unsigned int j=1; j<6; j++)
            vtau[j]=tau*vtau[j-1];

        for (unsigned int i=0; i<dim; i++)
        {    
            x[i]=yarp::math::dot(coeff[i],vtau);
    
            v[i]=a[i]=0.0;
            for (unsigned int j=0; j<5; j++)
            {    
                v[i]+=coeff[i][j+1]*vData[j]/T*vtau[j];
    
                if (j<4)
                    a[i]+=coeff[i][j+2]*aData[j]/(T*T)*vtau[j];
            }            
        }
    }
}


/************************************************************************/
minJerkTrajGen::~minJerkTrajGen()
{
    coeff.clear();
}




