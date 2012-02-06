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

    for (size_t i=0; i<tappedDelays.size(); i++)
        delete tappedDelays[i];

    tappedDelays.clear();
}


/************************************************************************/
void SmithPredictor::configure(Property &options, iKinChain &chain)
{    
    enabled=options.check("use_prediction",Value("false")).asString()=="true";
    if (!enabled)
        return;

    dealloc();
    
    // default values
    gains.resize(chain.getDOF(),1.0);
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
                    gains[i]=params->get(0).asDouble();
                    if (params->size()>1)
                    {
                        int depth=(int)ceil(params->get(1).asDouble()/Ts);
                        tappedDelays[i]->assign(depth,y0[i]);
                    }
                }
            }

            i++;
        }        
    }    
    
    I=new Integrator(Ts,y0,lim);
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
    }
}


/************************************************************************/
Vector SmithPredictor::computeCmd(const Vector &u)
{
    if (enabled && (tappedDelays.size()==u.length()))
    {
        Vector y=gains*I->integrate(u);
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


/************************************************************************/
Vector SmithPredictor::getGains()
{
    return gains;
}

