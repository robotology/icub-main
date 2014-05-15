/*
 * Copyright (C) 2013 iCub Facility - Istituto Italiano di Tecnologia
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

/** 
 * @defgroup optimization optimization
 * @ingroup icub_libraries 
 *  
 * Tools designed to deal with optimization tasks.
 *  
 * @author Ugo Pattacini
 *  
 * @defgroup GenericAlgorithms Generic Algorithms
 * @ingroup optimization
 *
 * Algorithms of general utility.
 *  
 * @author Ugo Pattacini 
 */ 


#ifndef __ICUB_OPT_ALGORITHMS_H__
#define __ICUB_OPT_ALGORITHMS_H__

#include <deque>
#include <yarp/sig/all.h>

namespace iCub
{

namespace optimization
{

/**
* \ingroup GenericAlgorithms
*
* Find the minimum volume ellipsoide (MVEE) of a set of N 
* d-dimensional data points. 
* @param points the set of N d-dimensional data points. 
* @param tol the tolerance of the algorithm given in the points 
*            metrics.
* @param A the dxd matrix of the ellipsoid equation in the 
*          center form: (x-c)'*A*(x-c)=1.
* @param c the d-dimensional vector representing the ellipsoid's 
*          center.
* @return true/false on success/fail.
*/
bool minVolumeEllipsoid(const std::deque<yarp::sig::Vector> &points,
                        const double tol, yarp::sig::Matrix &A,
                        yarp::sig::Vector &c);

}
 
}
 
#endif


