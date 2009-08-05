// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Lorenzo Natale
 * email:   lorenzo.natale@robotcub.org
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

#include <math.h>
#ifndef __FPSSTATS__
#define __FPSSTATS__

class FpsStats
{
public:
    double average;
    double max;
    double min;
    unsigned int iterations;
    double now;
    double prev;
    double accDt;
    double acc2Dt;
    
    FpsStats()
    {
        reset();
    }

    void reset()
    {
        average=0;
        max=-1e20;
        min=1e20;
        iterations=0;
        prev=0;
        now=0;
        accDt=0;
        acc2Dt=0;
    }

    void update(double nt)
    {
        if (iterations>=0)
        {
            double tmp=nt;
            accDt+=tmp;
            acc2Dt+=(tmp*tmp);
            if (tmp>max)
                max=tmp;
            if (tmp<min)
                min=tmp;
        }

        iterations++;
    }

    void getStats(double &av, double &m, double &M)
    {
        if (iterations<2)
        {
            av=0;
            m=0;
            M=0;
        }
        else
        {
            av=accDt/(iterations-1);
            m=min;
            M=max;
        }
    }
    
    void getStats(double &av, double &sd, double &m, double &M)
    {
        if (iterations<2)
        {
            av=0;
            sd=0;
            m=0;
            M=0;
        }
        else
        {
            sd=sqrt(acc2Dt - (accDt*accDt/iterations))/sqrt(iterations); 
            av=accDt/(iterations);
            m=min;
            M=max;
        }
    }

    void getStats(double &av)
    {
        if (iterations<2)
            av=0;
        else
            av=accDt/(iterations-1);
    }
};

#endif
