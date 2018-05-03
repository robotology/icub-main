// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2012  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Valentina Gaggero
 * email: valentina.gaggero@iit.it
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


#ifndef __statExt__
#define __statExt__

// System includes
#include <yarp/os/Ping.h>
#include <float.h>



//this class contains all info about pkts received from pc104
class StatExt: public yarp::os::Stat
{
private:
    double min;
    double max;
public:

    StatExt() {
        clear();
    }

    void clear() {
        min = DBL_MAX;
        max = 0;
        yarp::os::Stat::clear();
    }

    void add(double val) {
        if(val < min)
            min = val;

        if(val > max)
            max = val;

        yarp::os::Stat::add(val);
    }

    bool addAndCheckLimits(double val) {
        bool updatedLimits=false;
        if(val < min)
        {
            min = val;
            updatedLimits=true;
        }

        if(val > max)
        {
            max = val;
            updatedLimits=true;
        }

        yarp::os::Stat::add(val);
        return updatedLimits;
    }

//    void add(const StatExt& alt) {
//
//        if( alt.min < min)
//            min = alt.min;
//
//        if(alt.max > max)
//            max = alt.max;
//
//        Stat::add((Stat*)alt);
//    }



//    double mean();
//
//    double deviation();
//
//    double count();


    using yarp::os::Stat::mean;
    using yarp::os::Stat::deviation;
    using yarp::os::Stat::count;



    double getMin(void){
        return min;
    }


    double getMax(void){
        return max;
    }
};





#endif

// eof


