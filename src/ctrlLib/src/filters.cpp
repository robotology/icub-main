
#include <yarp/math/Math.h>
#include <iCub/filters.h>

using namespace std;
using namespace yarp;
using namespace yarp::sig;
using namespace yarp::math;
using namespace ctrl;


/***************************************************************************/
Filter::Filter(const Vector &num, const Vector &den, const Vector &y0)
{
    b=num;
    a=den;
    
    m=b.size(); uold.resize(m-1);
    n=a.size(); yold.resize(n-1);
    
    init(y0);
}


/***************************************************************************/
void Filter::init(const Vector &y0)
{
    y=y0;
    
    for (int i=0; i<m-1; i++)
        uold[i]=y;
    
    for (int i=0; i<n-1; i++)
        yold[i]=y;
}


/***************************************************************************/
Vector Filter::filt(const Vector &u)
{
    y=b[0]*u;
    
    for (int i=1; i<m; i++)
        y=y+b[i]*uold[i-1];
    
    for (int i=1; i<n; i++)
        y=y-a[i]*yold[i-1];
    
    y=(1.0/a[0])*y;
    
    uold.push_front(u);
    uold.pop_back();
    
    yold.push_front(y);
    yold.pop_back();
    
    return y;
}


/**********************************************************************/
RateLimiter::RateLimiter(const Vector &_rL, const Vector &_rU) :
                         rateLowerLim(_rL), rateUpperLim(_rU)
{
    size_t nL=rateLowerLim.length();
    size_t nU=rateUpperLim.length();

    n=nU>nL ? nL : nU;
}


/**********************************************************************/
void RateLimiter::init(const Vector &u0)
{ 
    uLim=u0;
}


/**********************************************************************/
Vector RateLimiter::filt(const Vector &u)
{
    uD=u-uLim;
    for (size_t i=0; i<n; i++)
        if (uD[i]>rateUpperLim[i])
            uD[i]=rateUpperLim[i];
        else if (uD[i]<rateLowerLim[i])
            uD[i]=rateLowerLim[i];

    uLim=uLim+uD;

    return uLim;
}



