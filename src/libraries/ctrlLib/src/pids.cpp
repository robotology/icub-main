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

#include <gsl/gsl_math.h>

#include <yarp/math/Math.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>

#define PID_SAT(x,L,H)      ((x)>(H)?(H):((x)<(L)?(L):(x)))

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


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
void Integrator::allocate(const Integrator &I)
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
    applySat=_applySat;
    if (applySat)
        y=saturate(y);
}


/************************************************************************/
void Integrator::setTs(const double _Ts)
{
    if (_Ts>0.0)
        Ts=_Ts;
}


/************************************************************************/
void Integrator::setLim(const Matrix &_lim)
{
    lim=_lim;

    if (applySat)
        y=saturate(y);
}


/************************************************************************/
const Vector& Integrator::integrate(const Vector &x)
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
void helperPID::addVectorToOption(Bottle &option, const char *key, const Vector &val)
{
    Bottle &bKey=option.addList();
    bKey.addString(key);
    Bottle &bKeyContent=bKey.addList();
    for (size_t i=0; i<val.length(); i++)
        bKeyContent.addDouble(val[i]);
}


/************************************************************************/
bool helperPID::getVectorFromOption(const Bottle &options, const char *key,
                                    Vector &val, int &size)
{
    if (options.check(key))
    {
        if (Bottle *b=options.find(key).asList())
        {
            int len=(int)val.length();
            int bSize=b->size();

            size=bSize<len?bSize:len;
            for (int i=0; i<size; i++)
                val[i]=b->get(i).asDouble();

            return true;
        }
    }

    return false;
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

    uSat.resize(dim,0.0);
    for (unsigned int i=0; i<dim; i++)
        uSat[i]=PID_SAT(u[i],satLim(i,0),satLim(i,1));

    Int=new Integrator(Ts,uSat);

    Vector u0(1,0.0);
    for (unsigned int i=0; i<dim; i++)
    {
        double tau=1e12;
        if ((Kp[i]!=0.0) && (N[i]!=0.0))
            tau=Kd[i]/(Kp[i]*N[i]);

        Vector num(2),den(2);
        num[0]=2.0;        num[1]=-2.0;
        den[0]=Ts+2.0*tau; den[1]=Ts-2.0*tau;
        Der.push_back(new Filter(num,den,u0));
    }

    P.resize(dim,0.0);
    I.resize(dim,0.0);
    D.resize(dim,0.0);
}


/************************************************************************/
const Vector& parallelPID::compute(const Vector &ref, const Vector &fb)
{
    // proportional part
    P=Kp*(Wp*ref-fb);

    // integral part
    I=Int->integrate(Ki*(Wi*ref-fb)+(uSat-u)/Tt);

    // derivative part
    Vector inputD=Kd*(Wd*ref-fb);
    for (unsigned int i=0; i<dim; i++)
    {
        Vector inputDi(1,inputD[i]);
        Vector outputDi=Der[i]->filt(inputDi);
        D[i]=outputDi[0];
    }

    // cumul output
    u=P+I+D;

    // saturation stage
    for (unsigned int i=0; i<dim; i++)
        uSat[i]=PID_SAT(u[i],satLim(i,0),satLim(i,1));

    return uSat;
}


/************************************************************************/
void parallelPID::reset(const Vector &u0)
{
    int len=u0.length()>(int)dim?dim:u0.length();

    Vector y=Int->get();
    Vector z1(1,0.0);
    for (int i=0; i<len; i++)
    {
        y[i]=u0[i];
        Der[i]->init(z1);
    }

    Int->reset(y);
}


/************************************************************************/
void parallelPID::getOptions(Bottle &options)
{
    Vector satLimVect(satLim.rows()*satLim.cols());
    for (int r=0; r<satLim.rows(); r++)
        for (int c=0; c<satLim.cols(); c++)
            satLimVect[r*satLim.cols()+c]=satLim(r,c);

    options.clear();
    addVectorToOption(options,"Kp",Kp);
    addVectorToOption(options,"Ki",Ki);
    addVectorToOption(options,"Kd",Kd);
    addVectorToOption(options,"Wp",Wp);
    addVectorToOption(options,"Wi",Wi);
    addVectorToOption(options,"Wd",Wd);
    addVectorToOption(options,"N",N);
    addVectorToOption(options,"Tt",Tt);
    addVectorToOption(options,"satLim",satLimVect);
        
    Bottle &bTs=options.addList();
    bTs.addString("Ts");
    bTs.addDouble(Ts);
}


/************************************************************************/
void parallelPID::setOptions(const Bottle &options)
{
    Vector satLimVect(satLim.rows()*satLim.cols());
    for (int r=0; r<satLim.rows(); r++)
        for (int c=0; c<satLim.cols(); c++)
            satLimVect[r*satLim.cols()+c]=satLim(r,c);

    bool recomputeQuantities=false;
    int size;
    getVectorFromOption(options,"Ki",Ki,size);
    getVectorFromOption(options,"Wp",Wp,size);
    getVectorFromOption(options,"Wi",Wi,size);
    getVectorFromOption(options,"Wd",Wd,size);
    getVectorFromOption(options,"Tt",Tt,size);

    if (getVectorFromOption(options,"Kp",Kp,size))
        recomputeQuantities=true;    

    if (getVectorFromOption(options,"Kd",Kd,size))
        recomputeQuantities=true;    

    if (getVectorFromOption(options,"N",N,size))
        recomputeQuantities=true;

    if (getVectorFromOption(options,"satLim",satLimVect,size))
    {
        for (int r=0; r<satLim.rows(); r++)
            for (int c=0; c<satLim.cols(); c++)
                satLim(r,c)=satLimVect[r*satLim.cols()+c];

        recomputeQuantities=true;
    }
    
    if (options.check("Ts"))
    {
        double _Ts=options.find("Ts").asDouble();
        if (_Ts>0.0)
        {
            Ts=_Ts;
            recomputeQuantities=true;
        }
    }

    if (recomputeQuantities)
    {
        for (unsigned int i=0; i<dim; i++)
            uSat[i]=PID_SAT(uSat[i],satLim(i,0),satLim(i,1));
    
        Vector u0(1,0.0);
        for (unsigned int i=0; i<dim; i++)
        {
            double tau=1e12;
            if ((Kp[i]!=0.0) && (N[i]!=0.0))
                tau=Kd[i]/(Kp[i]*N[i]);

            Vector num(2),den(2);
            num[0]=2.0;        num[1]=-2.0;
            den[0]=Ts+2.0*tau; den[1]=Ts-2.0*tau;
            Der[i]->adjustCoeffs(num,den);
            Der[i]->init(u0);
        }

        Int->setTs(Ts);
    }

    Vector v(dim);
    if (getVectorFromOption(options,"reset",v,size))
    {
        Vector u0(size);
        for (int i=0; i<size; i++)
            u0[i]=v[i];

        reset(u0);
    }
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

    uSat.resize(dim,0.0);
    for (unsigned int i=0; i<dim; i++)
        uSat[i]=PID_SAT(u[i],satLim(i,0),satLim(i,1));

    Vector u0(1,0.0);
    for (unsigned int i=0; i<dim; i++)
    {
        Vector num(2),den(2);

        num[0]=Ts;           num[1]=Ts;
        den[0]=Ts+2.0*Ti[i]; den[1]=Ts-2.0*Ti[i];
        Int.push_back(new Filter(num,den,u0));

        double tau=1e12;
        if ((Kp[i]!=0.0) && (N[i]!=0.0))
            tau=Kd[i]/(Kp[i]*N[i]);

        num[0]=2.0;        num[1]=-2.0;
        den[0]=Ts+2.0*tau; den[1]=Ts-2.0*tau;
        Der.push_back(new Filter(num,den,u0));
    }

    e.resize(dim,0.0);
    P.resize(dim,0.0);
    I.resize(dim,0.0);
    D.resize(dim,0.0);
}


/************************************************************************/
const Vector& seriesPID::compute(const Vector &ref, const Vector &fb)
{
    // compute error
    e=ref-fb;

    // derivative part
    for (unsigned int i=0; i<dim; i++)
    {
        Vector inputDi(1,Kd[i]*e[i]);
        Vector outputDi=Der[i]->filt(inputDi);
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
        uSat[i]=PID_SAT(u[i],satLim(i,0),satLim(i,1));

    return uSat;
}


/************************************************************************/
void seriesPID::reset()
{
    Vector u0(1,0.0);
    for (unsigned int i=0; i<dim; i++)
    {
        Int[i]->init(u0);    
        Der[i]->init(u0);
    }
}


/************************************************************************/
void seriesPID::getOptions(Bottle &options)
{
    Vector satLimVect(satLim.rows()*satLim.cols());
    for (int r=0; r<satLim.rows(); r++)
        for (int c=0; c<satLim.cols(); c++)
            satLimVect[r*satLim.cols()+c]=satLim(r,c);

    options.clear();
    addVectorToOption(options,"Kp",Kp);
    addVectorToOption(options,"Ti",Ti);
    addVectorToOption(options,"Kd",Kd);
    addVectorToOption(options,"N",N);
    addVectorToOption(options,"satLim",satLimVect);
        
    Bottle &bTs=options.addList();
    bTs.addString("Ts");
    bTs.addDouble(Ts);
}


/************************************************************************/
void seriesPID::setOptions(const Bottle &options)
{
    Vector satLimVect(satLim.rows()*satLim.cols());
    for (int r=0; r<satLim.rows(); r++)
        for (int c=0; c<satLim.cols(); c++)
            satLimVect[r*satLim.cols()+c]=satLim(r,c);

    bool recomputeQuantities=false;
    int size;
    if (getVectorFromOption(options,"Kp",Kp,size))
        recomputeQuantities=true;    

    if (getVectorFromOption(options,"Ti",Ti,size))
        recomputeQuantities=true;    

    if (getVectorFromOption(options,"Kd",Kd,size))
        recomputeQuantities=true;    

    if (getVectorFromOption(options,"N",N,size))
        recomputeQuantities=true;    

    if (getVectorFromOption(options,"satLim",satLimVect,size))
    {
        for (int r=0; r<satLim.rows(); r++)
            for (int c=0; c<satLim.cols(); c++)
                satLimVect[r*satLim.cols()+c]=satLim(r,c);

        recomputeQuantities=true;
    }

    if (options.check("Ts"))
    {
        double _Ts=options.find("Ts").asDouble();
        if (_Ts>0.0)
        {
            Ts=_Ts;
            recomputeQuantities=true;
        }
    }

    if (recomputeQuantities)
    {
        for (unsigned int i=0; i<dim; i++)
            uSat[i]=PID_SAT(uSat[i],satLim(i,0),satLim(i,1));
    
        Vector u0(1,0.0);
        for (unsigned int i=0; i<dim; i++)
        {
            Vector num(2),den(2);

            num[0]=Ts;           num[1]=Ts;
            den[0]=Ts+2.0*Ti[i]; den[1]=Ts-2.0*Ti[i];
            Int[i]->adjustCoeffs(num,den);
            Int[i]->init(u0);

            double tau=1e12;
            if ((Kp[i]!=0.0) && (N[i]!=0.0))
                tau=Kd[i]/(Kp[i]*N[i]);

            num[0]=2.0;        num[1]=-2.0;
            den[0]=Ts+2.0*tau; den[1]=Ts-2.0*tau;
            Der[i]->adjustCoeffs(num,den);
            Der[i]->init(u0);
        }
    }

    if (options.check("reset"))
        reset();
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


