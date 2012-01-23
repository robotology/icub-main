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

#include <sstream>
#include <math.h>

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
    gains.resize(1,0.0);
    configured=false;
}


/************************************************************************/
bool SmithPredictor::configure(const Property &options, iKinChain &chain)
{
    if (configured)
        return false;

    // "options" is meant to be properly formed
    // in order to make lots of checks unnecessary and
    // speed up the configuration stage
    Property &_options=const_cast<Property&>(options);
    double Ts=_options.find("Ts").asDouble();
    Vector y0(chain.getDOF());
    Matrix lim(chain.getDOF(),2);
    gains.resize(chain.getDOF(),1.0);

    // we're forced to cycle by joints and
    // not by dofs since the configuration
    // params are given with joints ordering
    int i=0;
    for (unsigned int j=0; j<chain.getN(); j++)
    {
        if (!chain[j].isBlocked())
        {
            y0[i]=chain[j].getAng();
            lim(i,0)=chain[j].getMin();
            lim(i,1)=chain[j].getMax();

            ostringstream tag;
            tag<<"joint_"<<j;
            if (_options.check(tag.str().c_str()))
            {
                if (Bottle *params=_options.find(tag.str().c_str()).asList())
                {
                    gains[i]=params->get(0).asDouble();
                    if (params->size()>1)
                    {
                        deque<double> delay;
                        int depth=(int)ceil(params->get(1).asDouble()/Ts);

                        for (int k=0; k<depth; k++)
                            delay.push_back(y0[i]);

                        tappedDelays[i]=delay;
                    }
                }
            }

            i++;
        }        
    }
    
    I=new Integrator(Ts,y0,lim);

    return configured=true;
}


/************************************************************************/
bool SmithPredictor::init(const Vector &y0)
{
    if (configured && (tappedDelays.size()==y0.length()))
    {
        // init the content of tapped delay lines
        for (size_t i=0; i<tappedDelays.size(); i++)
            for (size_t j=0; j<tappedDelays[i].size(); j++)
                tappedDelays[i][j]=y0[i];

        // init the integral part
        I->reset(y0);
        return true;
    }
    else
        return false;
}


/************************************************************************/
Vector SmithPredictor::compute(const Vector &u)
{
    if (configured && (tappedDelays.size()==u.length()))
    {
        Vector y=gains*I->integrate(u);
        Vector out(y.length());
        for (size_t i=0; i<out.length(); i++)
        {
            tappedDelays[i].push_back(y[i]);
            out[i]=y[i]-tappedDelays[i].front();
            tappedDelays[i].pop_front();
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



