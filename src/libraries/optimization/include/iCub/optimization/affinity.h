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
 * @defgroup Affinity Affine Transformations
 * @ingroup MatrixTransformations
 *
 * Given two sets of 3D points, the aim is to find out the 
 * affine transformation matrix between them. 
 *  
 * @author Ugo Pattacini 
 */ 


#ifndef __ICUB_OPT_AFFINITY_H__
#define __ICUB_OPT_AFFINITY_H__

#include <deque>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/optimization/matrixTransformation.h>

namespace iCub
{

namespace optimization
{

/**
* @ingroup Affinity
*
* A class that deals with the problem of determining the affine 
* transformation matrix A between two sets of matching 3D points
* employing IpOpt. 
*/
class AffinityWithMatchedPoints : public MatrixTransformationWithMatchedPoints
{
protected:
    yarp::sig::Matrix min, max;    
    yarp::sig::Matrix A0;

    int max_iter;
    double tol;
    
    std::deque<yarp::sig::Vector> p0;
    std::deque<yarp::sig::Vector> p1;

    double evalError(const yarp::sig::Matrix &A);

public:
    /**
    * Default Constructor. 
    */
    AffinityWithMatchedPoints();

    /**
    * Allow specifying the minimum and maximum bounds of the 
    * elements belonging to the affine transformation.
    * @param min the 4x4 Matrix containining the minimum bounds.
    * @param max the 4x4 Matrix containining the maximum bounds. 
    *  
    * @note the last row is always expected to be (0 0 0 1).
    */
    virtual void setBounds(const yarp::sig::Matrix &min, const yarp::sig::Matrix &max);

    /**
    * Add to the internal database the 3D-point p0 and the 3D-point 
    * p1 which is supposed to correspond to A*p0, whose matrix A has
    * to be found. 
    * @param p0 the free 3D-point.
    * @param p1 the 3D-point which corresponds to A*p0.
    * @return true/false on success/fail. 
    */
    virtual bool addPoints(const yarp::sig::Vector &p0, const yarp::sig::Vector &p1);

    /**
    * Return the number of 3D-points pairs currently contained into 
    * the internal database. 
    * @return the number of pairs. 
    */
    virtual size_t getNumPoints() const { return p0.size(); }

    /**
    * Retrieve copies of the database of 3D-points pairs.
    * @param p0 the list of free 3D-points.
    * @param p1 the list of 3D-points which correspond to A*p0. 
    *  
    * @note points are retrived in 4x1 homogeneous format. 
    */
    virtual void getPoints(std::deque<yarp::sig::Vector> &p0, std::deque<yarp::sig::Vector> &p1) const;

    /**
    * Clear the internal database of 3D points.
    */
    virtual void clearPoints();

    /**
    * Allow specifiying the initial guess for the affine 
    * transformation matrix we seek for. 
    * @param A the 4x4 homogeneous matrix used as initial guess.
    * @return true/false on success/fail. 
    */
    virtual bool setInitialGuess(const yarp::sig::Matrix &A);

    /**
    * Allow setting further options used during calibration.
    * @param options a Property-like object accounting for 
    *               calibration options.
    * @return true/false on success/fail. 
    */
    virtual bool setCalibrationOptions(const yarp::os::Property &options);

    /**
    * Perform optimization to determine the affine matrix A. 
    * @param A the final affine matrix that links the two clouds of 
    *          3D points.
    * @param error returns the residual error computed as 
    *              norm(p1[i]-A*p0[i]) over the whole set of points
    *              pairs.
    * @return true/false on success/fail. 
    */
    virtual bool calibrate(yarp::sig::Matrix &A, double &error);

    /**
    * Destructor.
    */
    virtual ~AffinityWithMatchedPoints() { }
};

}
 
}
 
#endif


