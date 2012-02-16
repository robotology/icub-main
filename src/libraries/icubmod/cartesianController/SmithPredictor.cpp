/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include <stdio.h>
#include <math.h>

#include <yarp/os/Bottle.h>
#include <yarp/math/Math.h>

#include "SmithPredictor.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;


/************************************************************************/
SmithPredictor::SmithPredictor()
{
    I=NULL;
    enabled=false;
}


/************************************************************************/
SmithPredictor::~SmithPredictor()
{
    dealloc();
}


/************************************************************************/
void SmithPredictor::dealloc()
{    
    if (I!=NULL)
        delete I;

    for (size_t i=0; i<F.size(); i++)
        delete F[i];

    for (size_t i=0; i<tappedDelays.size(); i++)
        delete tappedDelays[i];

    F.clear();
    tappedDelays.clear();
}


/************************************************************************/
void SmithPredictor::configure(Property &options, iKinChain &chain)
{
    enabled=options.check("smith_predictor",Value("off")).asString()=="on";
    if (!enabled)
        return;

    dealloc();

    // default values
    Vector Kp(chain.getDOF());   Kp=1.0;
    Vector Tz(chain.getDOF());   Tz=0.0;
    Vector Tw(chain.getDOF());   Tw=0.0;
    Vector Zeta(chain.getDOF()); Zeta=0.0;
    for (unsigned int i=0; i<chain.getDOF(); i++)
        tappedDelays.push_back(new deque<double>);

    double Ts=options.check("Ts",Value(0.01)).asDouble();
    Vector y0(chain.getDOF());
    Matrix lim(chain.getDOF(),2);

    // we're forced to cycle by joints and
    // not by dofs since the configuration
    // parameters are given with joints ordering
    int i=0;
    for (unsigned int j=0; j<chain.getN(); j++)
    {
        if (!chain[j].isBlocked())
        {
            y0[i]=chain[j].getAng();
            lim(i,0)=chain[j].getMin();
            lim(i,1)=chain[j].getMax();

            char entry[255];
            sprintf(entry,"joint_%d",j);
            if (options.check(entry))
            {
                if (Bottle *params=options.find(entry).asList())
                {
                    if (params->check("Kp"))
                        Kp[i]=params->find("Kp").asDouble();

                    if (params->check("Tz"))
                        Tz[i]=params->find("Tz").asDouble();

                    if (params->check("Tw"))
                        Tw[i]=params->find("Tw").asDouble();

                    if (params->check("Zeta"))
                        Zeta[i]=params->find("Zeta").asDouble();

                    if (params->check("Td"))
                    {
                        int depth=(int)ceil(params->find("Td").asDouble()/Ts);
                        tappedDelays[i]->assign(depth,y0[i]);
                    }
                }
            }

            i++;
        }        
    }    
    
    // create integrator
    I=new Integrator(Ts,y0,lim);

    // account for possible internal saturation
    Vector _y0=I->get();

    // create filters
    Vector num(3);
    Vector den(3);
    Vector y01(1);
    double Ts2=Ts*Ts;
    double twoTs2=2.0*Ts2;
    for (unsigned int i=0; i<chain.getDOF(); i++)
    {
        // implementing F(s)=Kp*(1+Tz*s)/(1+2*Zeta*Tw*s+(Tw*s)^2)
        double _num_0=2.0*Tz[i]*Ts;
        num[0]=Kp[i] * (Ts2 + _num_0);
        num[1]=Kp[i] * twoTs2;
        num[2]=Kp[i] * (Ts2 - _num_0);

        double _den_0=4.0*Tw[i]*Tw[i];
        double _den_1=2.0*_den_0;
        double _den_2=4.0*Zeta[i]*Ts*Tw[i];
        den[0]=Ts2    + _den_2 + _den_0;
        den[1]=twoTs2 - _den_1;
        den[2]=Ts2    - _den_2 + _den_0;
        
        y01[0]=_y0[i];
        F.push_back(new Filter(num,den,y01));
    }
}


/************************************************************************/
void SmithPredictor::restart(const Vector &y0)
{
    if (enabled && (tappedDelays.size()==y0.length()))
    {
        // init the content of tapped delay lines
        for (size_t i=0; i<tappedDelays.size(); i++)
            for (size_t j=0; j<tappedDelays[i]->size(); j++)
                tappedDelays[i]->at(j)=y0[i];

        // init the integral part
        I->reset(y0);

        // account for possible internal saturation
        Vector _y0=I->get();

        // init the filters
        Vector y01(1);
        for (size_t i=0; i<F.size(); i++)
        {
            y01[0]=_y0[i];
            F[i]->init(y01);
        }
    }
}


/************************************************************************/
Vector SmithPredictor::computeCmd(const Vector &u)
{
    if (enabled && (tappedDelays.size()==u.length()))
    {
        Vector _u(F.size());
        for (size_t i=0; i<F.size(); i++)
        {
            Vector u1(1); u1[0]=u[i];
            Vector _y=F[i]->filt(u1);
            _u[i]=_y[i];
        }

        Vector y=I->integrate(_u);
        Vector out(y.length());
        for (size_t i=0; i<out.length(); i++)
        {
            tappedDelays[i]->push_back(y[i]);
            out[i]=y[i]-tappedDelays[i]->front();
            tappedDelays[i]->pop_front();
        }

        return out;
    }
    else
    {
        Vector zeros(u.length());
        zeros=0.0;
        return zeros;
    }
}


