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
    if (F!=NULL)
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
    Property params=const_cast<Property&>(parameters);
    for (int i=0; i<len; i++)
    {        
        ostringstream entry;
        entry<<entryTag<<"_"<<(ordering.size()==0?i:ordering.get(i).asInt());
        if (params.check(entry.str().c_str()))
        {
            if (Bottle *options=params.find(entry.str().c_str()).asList())
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
        if (F[i]!=NULL)
            delete F[i];

    F.clear();
}




