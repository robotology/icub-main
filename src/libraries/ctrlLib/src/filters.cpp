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

#include <algorithm>

#include <gsl/gsl_math.h>

#include <yarp/os/Log.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/filters.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/***************************************************************************/
Filter::Filter(const Vector &num, const Vector &den, const Vector &y0)
{
    b=num;
    a=den;
    
    m=b.length(); uold.resize(m-1);
    n=a.length(); yold.resize(n-1);
    
    init(y0);
}


/***************************************************************************/
void Filter::init(const Vector &y0)
{
    // if there is a last input then take it as guess for the next input
    if (uold[0].size()>0)
        init(y0,uold[0]);
    else    // otherwise use zero
        init(y0,zeros(y0.length()));
}


/***************************************************************************/
void Filter::init(const Vector &y0, const Vector &u0)
{
    Vector u_init(y0.length(),0.0);
    Vector y_init=y0;
    y=y0;

    double sum_b=0.0;
    for (size_t i=0; i<b.length(); i++)
        sum_b+=b[i];

    double sum_a=0.0;
    for (size_t i=0; i<a.length(); i++)
        sum_a+=a[i];
    
    if (fabs(sum_b)>1e-9)   // if filter DC gain is not zero
        u_init=(sum_a/sum_b)*y0;
    else
    {
        // if filter gain is zero then you need to know in advance what
        // the next input is going to be for initializing (that is u0)
        // Note that, unless y0=0, the filter output is not going to be stable
        u_init=u0;
        if (fabs(sum_a-a[0])>1e-9)
            y_init=a[0]/(a[0]-sum_a)*y;
        // if sum_a==a[0] then the filter can only be initialized to zero
    }
    
    for (size_t i=0; i<yold.size(); i++)
        yold[i]=y_init;
    
    for (size_t i=0; i<uold.size(); i++)
        uold[i]=u_init;
}


/***************************************************************************/
void Filter::getCoeffs(Vector &num, Vector &den)
{
    num=b;
    den=a;
}


/***************************************************************************/
void Filter::setCoeffs(const Vector &num, const Vector &den)
{
    b=num;
    a=den;

    uold.clear();
    yold.clear();

    m=b.length(); uold.resize(m-1);
    n=a.length(); yold.resize(n-1);

    init(y);
}


/***************************************************************************/
bool Filter::adjustCoeffs(const Vector &num, const Vector &den)
{
    if ((num.length()==b.length()) && (den.length()==a.length()))
    {
        b=num;
        a=den;
        return true;
    }
    else
        return false;
}


/***************************************************************************/
const Vector& Filter::filt(const Vector &u)
{
    yAssert(y.length()==u.length());
    for (size_t j=0; j<y.length(); j++)
        y[j]=b[0]*u[j];
    
    for (size_t i=1; i<m; i++)
        for (size_t j=0; j<y.length(); j++)
            y[j]+=b[i]*uold[i-1][j];
    
    for (size_t i=1; i<n; i++)
        for (size_t j=0; j<y.length(); j++)
            y[j]-=a[i]*yold[i-1][j];
    
    for (size_t j=0; j<y.length(); j++)
        y[j]/=a[0];
    
    uold.push_front(u);
    uold.pop_back();
    
    yold.push_front(y);
    yold.pop_back();
    
    return y;
}


/**********************************************************************/
RateLimiter::RateLimiter(const Vector &rL, const Vector &rU) :
                         rateLowerLim(rL), rateUpperLim(rU)
{
    size_t nL=rateLowerLim.length();
    size_t nU=rateUpperLim.length();
    n=std::min(nL,nU);
}


/**********************************************************************/
void RateLimiter::init(const Vector &u0)
{ 
    uLim=u0;
}


/**********************************************************************/
void RateLimiter::getLimits(Vector &rL, Vector &rU)
{
    rL=rateLowerLim;
    rU=rateUpperLim;
}


/**********************************************************************/
void RateLimiter::setLimits(const Vector &rL, const Vector &rU)
{
    rateLowerLim=rL;
    rateUpperLim=rU;
}


/**********************************************************************/
const Vector& RateLimiter::filt(const Vector &u)
{
    uD=u-uLim;
    for (size_t i=0; i<n; i++)
    {
        if (uD[i]>rateUpperLim[i])
            uD[i]=rateUpperLim[i];
        else if (uD[i]<rateLowerLim[i])
            uD[i]=rateLowerLim[i];
    }

    uLim=uLim+uD;
    return uLim;
}


/**********************************************************************/
FirstOrderLowPassFilter::FirstOrderLowPassFilter(const double cutFrequency,
                                                 const double sampleTime,
                                                 const Vector &y0)
{
    fc=cutFrequency;
    Ts=sampleTime;
    y=y0;
    filter=NULL;
    computeCoeff();
}


/**********************************************************************/
FirstOrderLowPassFilter::~FirstOrderLowPassFilter()
{
    delete filter;
}


/***************************************************************************/
void FirstOrderLowPassFilter::init(const Vector &y0)
{
    if (filter!=NULL)
        filter->init(y0);
}


/**********************************************************************/
bool FirstOrderLowPassFilter::setCutFrequency(const double cutFrequency)
{
    if (cutFrequency<=0.0)
        return false;

    fc=cutFrequency;
    computeCoeff();

    return true;
}


/**********************************************************************/
bool FirstOrderLowPassFilter::setSampleTime(const double sampleTime)
{
    if (sampleTime<=0.0)
        return false;

    Ts=sampleTime;
    computeCoeff();

    return true;
}


/**********************************************************************/
const Vector& FirstOrderLowPassFilter::filt(const Vector &u)
{
    if (filter!=NULL)
        y=filter->filt(u);

    return y;
}


/**********************************************************************/
void FirstOrderLowPassFilter::computeCoeff()
{
    double tau=1.0/(2.0*M_PI*fc);
    Vector num=cat(Ts,Ts);
    Vector den=cat(2.0*tau+Ts,Ts-2.0*tau);

    if (filter!=NULL)
        filter->adjustCoeffs(num,den);
    else
        filter=new Filter(num,den,y);
}


/***************************************************************************/
MedianFilter::MedianFilter(const size_t n, const Vector &y0)
{
    this->n=n;
    init(y0);
}


/***************************************************************************/
void MedianFilter::init(const Vector &y0)
{
    yAssert(y0.length()>0);
    y=y0;
    m=y.length();
    uold.assign(m,deque<double>());
}



/***************************************************************************/
void MedianFilter::setOrder(const size_t n)
{
    this->n=n;
    init(y);
}


/***************************************************************************/
double MedianFilter::median(deque<double>& v)
{
    size_t L=v.size()>>1;
    nth_element(v.begin(),v.begin()+L,v.end());
    if (v.size()&0x01)
        return v[L];
    else
    {
        nth_element(v.begin(),v.begin()+L-1,v.end());
        return 0.5*(v[L]+v[L-1]);
    }
}


/***************************************************************************/
const Vector& MedianFilter::filt(const Vector &u)
{
    yAssert(y.length()==u.length());
    for (size_t i=0; i<m; i++)
        uold[i].push_front(u[i]);

    if (uold[0].size()>n)
    {
        for (size_t i=0; i<m; i++)
        {
            deque<double> tmp=uold[i];
            y[i]=median(tmp);
            uold[i].pop_back();
        }
    }

    return y;
}


