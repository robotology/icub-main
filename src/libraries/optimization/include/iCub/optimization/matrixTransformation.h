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
 * @defgroup MatrixTransformations Finding Matrix Transformations
 * @ingroup optimization
 *
 * Given two sets of 3D points, the aim is to find out the 
 * matrix transformation between them. 
 *  
 * @author Ugo Pattacini 
 */ 


#ifndef __ICUB_OPT_MATRIXTRANSFORMATION_H__
#define __ICUB_OPT_MATRIXTRANSFORMATION_H__

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

namespace iCub
{

namespace optimization
{

/**
* @ingroup MatrixTransformations
*
* A class interface to deal with the problem of determining the
* matrix transformation between two sets of matching 3D points.
*/
class MatrixTransformationWithMatchedPoints
{
public:
    /**
    * Allow specifying the minimum and maximum bounds of the 
    * elements belonging to the transformation.
    * @param min the 4x4 Matrix containining the minimum bounds.
    * @param max the 4x4 Matrix containining the maximum bounds. 
    */
    virtual void setBounds(const yarp::sig::Matrix &min, const yarp::sig::Matrix &max) = 0;

    /**
    * Add to the internal database the 3D-point p0 and the 
    * corresponding 3D-point p1.
    * @param p0 the free 3D-point.
    * @param p1 the 3D-point which transforms p0.
    * @return true/false on success/fail. 
    */
    virtual bool addPoints(const yarp::sig::Vector &p0, const yarp::sig::Vector &p1) = 0;

    /**
    * Return the number of 3D-points pairs currently contained into 
    * the internal database. 
    * @return the number of pairs. 
    */
    virtual size_t getNumPoints() const = 0;

    /**
    * Retrieve copies of the database of 3D-points pairs.
    * @param p0 the list of free 3D-points.
    * @param p1 the list of 3D-points which trasnform p0. 
    *  
    * @note points are retrived in 4x1 homogeneous format. 
    */
    virtual void getPoints(std::deque<yarp::sig::Vector> &p0, std::deque<yarp::sig::Vector> &p1) const = 0;

    /**
    * Clear the internal database of 3D points.
    */
    virtual void clearPoints() = 0;

    /**
    * Allow specifiying the initial guess for the matrix 
    * transformation we seek for. 
    * @param M the 4x4 homogeneous matrix used as initial guess.
    * @return true/false on success/fail. 
    */
    virtual bool setInitialGuess(const yarp::sig::Matrix &M) = 0;

    /**
    * Allow setting further options used during calibration.
    * @param options a Property-like object accounting for 
    *               calibration options.
    * @return true/false on success/fail. 
    */
    virtual bool setCalibrationOptions(const yarp::os::Property &options) { return true; }

    /**
    * Perform optimization to determine the matrix transformation. 
    * @param M the final matrix that links the two clouds of 3D 
    *          points.
    * @param error returns the residual error.
    * @return true/false on success/fail. 
    */
    virtual bool calibrate(yarp::sig::Matrix &M, double &error) = 0;

    /**
    * Destructor.
    */
    virtual ~MatrixTransformationWithMatchedPoints() { }
};

}
 
}
 
#endif


