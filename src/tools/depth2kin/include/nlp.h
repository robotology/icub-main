/*
 * Copyright (C) 2014 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#ifndef __DEPTH2KIN_NLP_H__
#define __DEPTH2KIN_NLP_H__

#include <string>
#include <deque>

#include <yarp/sig/all.h>

#include <IpIpoptApplication.hpp>

#define ALIGN_IPOPT_MAX_ITER    300


/****************************************************************/
yarp::sig::Matrix computeH(const yarp::sig::Vector &x);


/****************************************************************/
class EyeAligner
{
protected:
    yarp::sig::Vector min;
    yarp::sig::Vector max;
    yarp::sig::Vector x0;
    yarp::sig::Matrix Prj;

    std::deque<yarp::sig::Vector> p2d;
    std::deque<yarp::sig::Vector> p3d;

    double evalError(const yarp::sig::Matrix &H);

public:
    EyeAligner();
    bool setProjection(const yarp::sig::Matrix &Prj);
    yarp::sig::Matrix getProjection() const;
    void setBounds(const yarp::sig::Vector &min, const yarp::sig::Vector &max);
    bool addPoints(const yarp::sig::Vector &p2di, const yarp::sig::Vector &p3di);
    void clearPoints();
    size_t getNumPoints() const;
    bool setInitialGuess(const yarp::sig::Matrix &H);
    bool calibrate(yarp::sig::Matrix &H, double &error, const int max_iter=ALIGN_IPOPT_MAX_ITER,
                   const int print_level=0, const std::string &derivative_test="none");
};


#endif

