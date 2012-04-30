/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @defgroup calibration calibration
 * @ingroup icub_libraries 
 *  
 * Tools designed to deal with calibration tasks.
 *  
 * @author Ugo Pattacini
 *  
 * @defgroup References References Calibration 
 * @ingroup calibration
 *
 * Given two sets of 3D points, the aim is to find out the 
 * transformation matrix between them. 
 */ 


#ifndef __CALIBREFERENCE_H__
#define __CALIBREFERENCE_H__

#include <deque>
#include <yarp/sig/all.h>

namespace iCub
{

namespace calibration
{

/**
* @ingroup References
*
* A class that deals with the problem of determining the 
* roto-translation matrix H and scaling factors S between two 
* sets of matching 3D points employing IpOpt. 
*  
* The problem solved is of the form: 
* 
* H^*=\arg\min_{H\in SE\left(3\right)}\left(\frac{1}{2N}\sum_{i=1}^{N} \left \| p_i^{O_1}-S \cdot H \cdot p_i^{O_2} \right \|^2 \right)\right.
*/
class CalibReferenceWithMatchedPoints
{
protected:
    yarp::sig::Vector min, min_s;
    yarp::sig::Vector max, max_s;
    yarp::sig::Vector x0,  s0;

    std::deque<yarp::sig::Vector> p0;
    std::deque<yarp::sig::Vector> p1;

    double evalError(const yarp::sig::Matrix &H);

public:
    /**
    * Default Constructor. 
    */
    CalibReferenceWithMatchedPoints();

    /**
    * Allow specifying the bounding box for both the translation 
    * part (given in meters) and the rotation part (given in 
    * radians) within which solution is seeked. 
    * @param min the 6x1 Vector containining the minimum bounds. The 
    *            first 3 dimensions account for the translation
    *            part, the last 3 for the rotation in Euler angles
    *            representation.
    * @param max the 6x1 Vector containining the maximums bounds. 
    *            The first 3 dimensions account for the translation
    *            part, the last 3 for the rotation in Euler angles
    *            representation.
    *  
    * @note by default min=(-1.0,-1.0,-1.0,-M_PI,-M_PI,-M_PI) and 
    *       max=(1.0,1.0,1.0,M_PI,M_PI,M_PI).
    */
    void setBounds(const yarp::sig::Vector &min, const yarp::sig::Vector &max);

    /**
    * Allow specifying the bounds for the 3D scaling factors.
    * @param min the 3x1 Vector containining the minimum bounds.
    * @param max the 3x1 Vector containining the maximums bounds. 
    *  
    * @note by default min=(0.1,0.1,0.1) and max=(10.0,10.0,10.0). 
                                                                 */
    void setScalingBounds(const yarp::sig::Vector &min, const yarp::sig::Vector &max);

    /**
    * Add to the internal databse the 3D-point p0 and the 3D-point 
    * p1 which is supposed to correspond to H*p0, whose matrix H has 
    * to be found. 
    * @param p0 the free 3D-point.
    * @param p1 the 3D-point which corresponds to H*p0. 
    * @return true/false on success/fail. 
    */
    bool addPoints(const yarp::sig::Vector &p0, const yarp::sig::Vector &p1);

    /**
    * Clear the internal database of 3D points.
    */
    void clearPoints();

    /**
    * Allow specifiying the initial guess for the roto-translation 
    * matrix we seek for.
    * @param H the 4x4 homogeneous matrix used as initial guess.
    * @return true/false on success/fail. 
    */
    bool setInitialGuess(const yarp::sig::Matrix &H);

    /**
    * Allow specifiying the initial guess for the scaling factors.
    * @param s the 3x1 vector of scaling factors.
    * @return true/false on success/fail. 
    */
    bool setScalingInitialGuess(const yarp::sig::Vector &s);

    /**
    * Perform reference calibration to determine the matrix H. 
    * @param H the final roto-translation matrix that links the two 
    *          reference frames of 3D points.
    * @param error returns the residual error computed as 
    *              norm(p1[i]-H*p0[i]) over the whole set of points
    *              pairs.
    * @return true/false on success/fail. 
    */
    bool calibrate(yarp::sig::Matrix &H, double &error);

    /**
    * Perform reference calibration to determine the matrix H and 
    * the scaling factors S. 
    * @param H the final roto-translation matrix that links the two 
    *          reference frames of 3D points.
    * @param s the 3x1 vector containing the found scaling factors.
    * @param error returns the residual error computed as 
    *              norm(p1[i]-S*H*p0[i]) over the whole set of
    *              points pairs, where S is the diagonal 3x3 matrix
    *              containing the scaling factors.
    * @return true/false on success/fail. 
    */
    bool calibrate(yarp::sig::Matrix &H, yarp::sig::Vector &s, double &error);
};

}
 
}
 
#endif


