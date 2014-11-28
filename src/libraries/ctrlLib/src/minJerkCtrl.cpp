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

#include <sstream>

#include <gsl/gsl_math.h>

#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/minJerkCtrl.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


/*******************************************************************************************/
minJerkVelCtrlForIdealPlant::minJerkVelCtrlForIdealPlant(const double _Ts, const int _dim) :
                                                         Ts(_Ts), dim(_dim), T(1.0), F(NULL)
{
    computeCoeffs();
}


/*******************************************************************************************/
void minJerkVelCtrlForIdealPlant::computeCoeffs()
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
    Vector num(3);
    Vector den(3);

    double c1=twoOnTs*(twoOnTs-c)-b;
    double c2=-a/c1;

    num[0]=c2;
    num[1]=2.0*c2;
    num[2]=c2;

    den[0]=1.0;
    den[1]=-2.0*(twoOnTs*twoOnTs+b)/c1;
    den[2]=(twoOnTs*(twoOnTs+c)-b)/c1;

    if (F==NULL)
    {
        Vector e0(dim); e0=0.0;
        F=new Filter(num,den,e0);
    }
    else
        F->adjustCoeffs(num,den);
}


/*******************************************************************************************/
Vector minJerkVelCtrlForIdealPlant::computeCmd(const double _T, const Vector &e)
{
    if (T!=_T)
    {    
        T=_T;
        computeCoeffs();
    }

    return F->filt(e);
}


/*******************************************************************************************/
void minJerkVelCtrlForIdealPlant::reset(const Vector &u0)
{
    F->init(u0);
}


/*******************************************************************************************/
minJerkVelCtrlForIdealPlant::~minJerkVelCtrlForIdealPlant()
{
    delete F;
}


/*******************************************************************************************/
minJerkVelCtrlForNonIdealPlant::minJerkVelCtrlForNonIdealPlant(const double _Ts, const int _dim) :
                                                               Ts(_Ts), dim(_dim), T(1.0)
{
    Kp.resize(dim,1.0);
    Tz.resize(dim,0.0);
    Tw.resize(dim,0.0);
    Zeta.resize(dim,0.0);

    for (int i=0; i<dim; i++)
        F.push_back(NULL);

    computeCoeffs();
}


/*******************************************************************************************/
void minJerkVelCtrlForNonIdealPlant::computeCoeffs()
{
    Vector num(4);
    Vector den(4);

    double T2=T*T;
    double T3=T2*T;

    // 90% of steady-state value in t=T
    // transient extinguished for t>=1.5*T
    double a=-150.765868956161/T3;
    double b=-84.9812819469538/T2;
    double c=-15.9669610709384/T;

    double Ts2=Ts*Ts;
    double Ts3=Ts2*Ts;
    double _num_0=3.0*Ts3;
    double _den_0=4.0*Ts;
    double _den_1=2.0*Ts2;
    double _den_2=_den_1*c;
    double _den_3=Ts3*b;
    double _den_4=3.0*_den_3;    

    for (int i=0; i<dim; i++)
    {
        // implementing F(s)=-a/(s^2-c*s-b) * ((1/Kp)*(1+2*Zeta*Tw*s+(Tw*s)^2)/(1+Tz*s))
        double _num_1=4.0*Zeta[i]*Ts2*Tw[i];
        double _num_2=4.0*Ts*Tw[i]*Tw[i];
        num[0]=a * (Ts3    + _num_1 + _num_2);
        num[1]=a * (_num_0 + _num_1 - _num_2);
        num[2]=a * (_num_0 - _num_1 - _num_2);
        num[3]=a * (Ts3    - _num_1 + _num_2);
        
        double _den_5=_den_1*Tz[i]*b;
        double _den_6=_den_0*Tz[i]*c;
        double _den_7=8.0*Tz[i];
        double _den_8=3.0*_den_7;
        den[0]=Kp[i] * (_den_3 - _den_7 - _den_0 + _den_2 + _den_5 + _den_6);
        den[1]=Kp[i] * (_den_0 + _den_8 + _den_4 + _den_2 + _den_5 - _den_6);
        den[2]=Kp[i] * (_den_0 - _den_8 + _den_4 - _den_2 - _den_5 - _den_6);
        den[3]=Kp[i] * (_den_7 - _den_0 + _den_3 - _den_2 - _den_5 + _den_6);

        if (F[i]==NULL)
        {
            Vector e0(1); e0=0.0;
            F[i]=new Filter(num,den,e0);
        }
        else
            F[i]->adjustCoeffs(num,den);
    }
}


/*******************************************************************************************/
Vector minJerkVelCtrlForNonIdealPlant::computeCmd(const double _T, const Vector &e)
{
    if (T!=_T)
    {    
        T=_T;
        computeCoeffs();
    }

    Vector y(dim), _e(1);
    for (int i=0; i<dim; i++)
    {
        _e[0]=e[i];
        Vector _y=F[i]->filt(_e);
        y[i]=_y[0];
    }

    return y;
}


/*******************************************************************************************/
void minJerkVelCtrlForNonIdealPlant::reset(const Vector &u0)
{
    Vector _u0(1);
    for (int i=0; i<dim; i++)
    {
        _u0[0]=u0[i];
        F[i]->init(_u0);
    }
}


/*******************************************************************************************/
void minJerkVelCtrlForNonIdealPlant::setPlantParameters(const Property &parameters,
                                                        const string &entryTag,
                                                        const Bottle &ordering)
{
    // default values
    Kp.resize(dim,1.0);
    Tz.resize(dim,0.0);
    Tw.resize(dim,0.0);
    Zeta.resize(dim,0.0);    

    int len=ordering.size()==0?dim:ordering.size();
    for (int i=0; i<len; i++)
    {        
        ostringstream entry;
        entry<<entryTag<<"_"<<(ordering.size()==0?i:ordering.get(i).asInt());
        if (parameters.check(entry.str().c_str()))
        {
            if (Bottle *options=parameters.find(entry.str().c_str()).asList())
            {
                if (options->check("Kp"))
                    Kp[i]=options->find("Kp").asDouble();

                if (options->check("Tz"))
                    Tz[i]=options->find("Tz").asDouble();

                if (options->check("Tw"))
                    Tw[i]=options->find("Tw").asDouble();

                if (options->check("Zeta"))
                    Zeta[i]=options->find("Zeta").asDouble();
            }
        }
    }
    
    computeCoeffs();
}


/*******************************************************************************************/
void minJerkVelCtrlForNonIdealPlant::getPlantParameters(Property &parameters,
                                                        const string &entryTag)
{
    ostringstream entry;
    for (int i=0; i<dim; i++)
    {                
        entry<<"("<<entryTag<<"_"<<i<<" (";

        Property prop;
        prop.put("Kp",Kp[i]);
        prop.put("Tz",Tz[i]);
        prop.put("Tw",Tw[i]);
        prop.put("Zeta",Zeta[i]);

        entry<<prop.toString().c_str()<<")) ";
    }

    parameters.fromString(entry.str().c_str());
}


/*******************************************************************************************/
minJerkVelCtrlForNonIdealPlant::~minJerkVelCtrlForNonIdealPlant()
{
    for (size_t i=0; i<F.size(); i++)
        delete F[i];

    F.clear();
}


/*******************************************************************************************/
minJerkTrajGen::minJerkTrajGen(const unsigned int _dim, const double _Ts, const double _T)
    :dim(_dim), Ts(_Ts), T(_T)
{
    posFilter = velFilter = accFilter = NULL;
    pos = vel = acc = lastRef = zeros(dim);
    computeCoeffs();
}


/*******************************************************************************************/
minJerkTrajGen::minJerkTrajGen(const Vector &y0, const double _Ts, const double _T)
    :dim(y0.size()), Ts(_Ts), T(_T)
{
    posFilter = velFilter = accFilter = NULL;
    lastRef = pos = y0;
    vel = acc = zeros(dim);
    computeCoeffs();
}


/*******************************************************************************************/
minJerkTrajGen::minJerkTrajGen(const minJerkTrajGen &z)
{
    pos = z.pos;
    vel = z.vel;
    acc = z.acc;
    lastRef = z.lastRef;
    T = z.T;
    Ts = z.Ts;
    dim = z.dim;

    computeCoeffs();
}


/*******************************************************************************************/
minJerkTrajGen::~minJerkTrajGen()
{
    delete posFilter;
    delete velFilter;
    delete accFilter;
}


/*******************************************************************************************/
minJerkTrajGen& minJerkTrajGen::operator=(const minJerkTrajGen &z)
{
    pos = z.pos;
    vel = z.vel;
    acc = z.acc;
    lastRef = z.lastRef;
    T = z.T;
    Ts = z.Ts;
    dim = z.dim;
    
    computeCoeffs();

    return *this;
}


/*******************************************************************************************/
void minJerkTrajGen::computeCoeffs()
{
    // 90% of steady-state value in t=T
    // transient extinguished for t>=1.5*T
    double a = -150.765868956161/(T*T*T);
    double b = -84.9812819469538/(T*T);
    double c = -15.9669610709384/T;
    double m = 4.0*c*Ts;
    double n = 2.0*b*Ts*Ts;
    double p = a*Ts*Ts*Ts;

    // implementing F(s)=-a/(s^3-c*s^2-b*s-a)
    Vector num(4);
    Vector den(4);

    num[0] = p;
    num[1] = 3.0*p;
    num[2] = 3.0*p;
    num[3] = p;

    den[0] = m+n+p-8.0;
    den[1] = -m+n+3.0*p+24.0;
    den[2] = -m-n+3.0*p-24.0;
    den[3] = m-n+p+8.0;

    if (posFilter==NULL)
        posFilter=new Filter(num,den,pos);
    else
        posFilter->adjustCoeffs(num,den);

    // implementing F(s)=-a*s/(s^3-c*s^2-b*s-a)
    p = 2.0*a*Ts*Ts;
    num[0] = p;
    num[1] = p;
    num[2] = -p;
    num[3] = -p;

    if (velFilter==NULL)
        velFilter=new Filter(num,den,vel);
    else
        velFilter->adjustCoeffs(num,den);
    velFilter->init(zeros(dim), pos);  //init filter to avoid spikes at the start

    // implementing F(s)=-a*s^2/(s^3-c*s^2-b*s-a)
    p = 4.0*a*Ts;
    num[0] = p;
    num[1] = -p;
    num[2] = -p;
    num[3] = p;

    if (accFilter==NULL)
        accFilter=new Filter(num,den,acc);
    else
        accFilter->adjustCoeffs(num,den);
    accFilter->init(zeros(dim), pos);  //init filter to avoid spikes at the start
}


/*******************************************************************************************/
void minJerkTrajGen::computeNextValues(const Vector &ref)
{
    lastRef = ref;

    if (posFilter!=NULL)
        pos = posFilter->filt(ref);

    if (velFilter!=NULL)
        vel = velFilter->filt(ref);

    if (accFilter!=NULL)
        acc = accFilter->filt(ref);
}


/*******************************************************************************************/
bool minJerkTrajGen::setT(const double _T)
{
    if(_T<=0.0)
        return false;
    T = _T;
    computeCoeffs();
    return true;
}


/*******************************************************************************************/
bool minJerkTrajGen::setTs(const double _Ts)
{
    if(_Ts<=0.0)
        return false;
    Ts = _Ts;
    computeCoeffs();
    return true;
}


/*******************************************************************************************/
void minJerkTrajGen::init(const Vector &y0)
{
    // save initial state y0, so that if setT() or setTs() are called afterwards
    // the vel and acc filters are initialized with the right value (i.e. y0) 
    lastRef = pos = y0; 
    if (posFilter!=NULL)
        posFilter->init(y0);

    if (velFilter!=NULL)
        velFilter->init(zeros(dim), y0);

    if (accFilter!=NULL)
        accFilter->init(zeros(dim), y0);
}


/*******************************************************************************************/
minJerkRefGen::minJerkRefGen(const unsigned int _dim, const double _Ts, const double _T)
    :dim(_dim), Ts(_Ts), T(_T)
{
    posFilter = velFilter = accFilter = NULL;
    pos = vel = acc = lastRef = zeros(dim);
    computeCoeffs();
}


/*******************************************************************************************/
minJerkRefGen::minJerkRefGen(const Vector &y0, const double _Ts, const double _T)
    :dim(y0.size()), Ts(_Ts), T(_T)
{
    posFilter = velFilter = accFilter = NULL;
    lastRef = pos = y0;
    vel = acc = zeros(dim);
    computeCoeffs();
}


/*******************************************************************************************/
minJerkRefGen::minJerkRefGen(const minJerkRefGen &z)
{
    pos = z.pos;
    vel = z.vel;
    acc = z.acc;
    lastRef = z.lastRef;
    T = z.T;
    Ts = z.Ts;
    dim = z.dim;

    computeCoeffs();
}


/*******************************************************************************************/
minJerkRefGen::~minJerkRefGen()
{
    delete posFilter;
    delete velFilter;
    delete accFilter;
}


/*******************************************************************************************/
minJerkRefGen& minJerkRefGen::operator=(const minJerkRefGen &z)
{
    pos = z.pos;
    vel = z.vel;
    acc = z.acc;
    lastRef = z.lastRef;
    T = z.T;
    Ts = z.Ts;
    dim = z.dim;
    
    computeCoeffs();

    return *this;
}


/*******************************************************************************************/
void minJerkRefGen::computeCoeffs()
{
    // 90% of steady-state value in t=T
    // transient extinguished for t>=1.5*T
    double a = -150.765868956161/(T*T*T);
    double b = -84.9812819469538/(T*T);
    double c = -15.9669610709384/T;

    // implementing F(s)=-a/(s^3-c*s^2-b*s-a)
    double m = 4.0*c*Ts;
    double n = 2.0*b*Ts*Ts;
    double p = a*Ts*Ts*Ts;
    Vector num = cat(p, 3.0*p, 3.0*p, p);
    Vector den = cat( m+n+p-8.0, -m+n+3.0*p+24.0, -m-n+3.0*p-24.0, m-n+p+8.0);
    if (posFilter==NULL)
        posFilter=new Filter(num,den,pos);
    else
        posFilter->adjustCoeffs(num,den);

    // implementing F(s)=-a/(s^2-c*s-b)
    double twoOnTs=2.0/Ts;
    double c1=twoOnTs*(twoOnTs-c)-b;
    double c2=-a/c1;
    num = cat(c2, 2.0*c2, c2);
    den = cat(1.0, -2.0*(twoOnTs*twoOnTs+b)/c1, (twoOnTs*(twoOnTs+c)-b)/c1);
    if (velFilter==NULL)
        velFilter = new Filter(num,den,zeros(dim));
    else
        velFilter->adjustCoeffs(num,den);
    velFilter->init(zeros(dim), pos);       //init filter to avoid spikes at the start

    // implementing F(s)=-a*s/(s^2-c*s-b)
    m = 2.0*c*Ts;
    n = b*Ts*Ts;
    p = 2.0*a*Ts;
    num = cat(-p, 0.0, p);
    den = cat(4.0-m-n, -8.0+m-2.0*n, 4.0-n);
    if (accFilter==NULL)
        accFilter=new Filter(num,den,acc);
    else
        accFilter->adjustCoeffs(num,den);
    accFilter->init(zeros(dim), pos);       //init filter to avoid spikes at the start
}


/*******************************************************************************************/
void minJerkRefGen::computeNextValues(const yarp::sig::Vector &y)
{
    if (posFilter!=NULL)
        pos = posFilter->filt(lastRef);
    
    // rotate pos around lastRef so that it lies along the distance y-lastRef
    /*double n = yarp::math::norm(y-lastRef);
    if(n!=0.0)
        pos = lastRef + yarp::math::norm(pos-lastRef)*(y-lastRef)/n;*/

    if (velFilter!=NULL)
        vel = velFilter->filt(lastRef-y);

    if (accFilter!=NULL)
        acc = accFilter->filt(lastRef-y);
}


/*******************************************************************************************/
void minJerkRefGen::computeNextValues(const yarp::sig::Vector &y, const yarp::sig::Vector &yd)
{
    lastRef = yd;
    computeNextValues(y);
}


/*******************************************************************************************/
bool minJerkRefGen::setT(const double _T)
{
    if(_T<=0.0)
        return false;
    T = _T;
    computeCoeffs();
    return true;
}


/*******************************************************************************************/
bool minJerkRefGen::setTs(const double _Ts)
{
    if(_Ts<=0.0)
        return false;
    Ts = _Ts;
    computeCoeffs();
    return true;
}


/*******************************************************************************************/
void minJerkRefGen::init(const Vector &y0)
{
    lastRef = pos = y0;

    if (posFilter!=NULL)
        posFilter->init(y0);

    if (velFilter!=NULL)
        velFilter->init(zeros(dim), y0);

    if (accFilter!=NULL)
        accFilter->init(zeros(dim), y0);
}


